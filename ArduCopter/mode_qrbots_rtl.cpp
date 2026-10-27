#include "Copter.h"
#include "mode_qrbots_rtl.h"

constexpr float ModeQRBotsRTL::MAX_PROFILE_ACCEL_CMSS;

// ---------------------------------------------------------------------------
// Scheduled RTL request
//
// The RTL command only LATCHES the target show time. The vehicle keeps flying
// the native DroneShow mode (trajectory, LEDs, everything) untouched. When the
// show clock reaches the target, check_pending() switches into this mode, whose
// init() therefore always starts at WAITING_5S.
// ---------------------------------------------------------------------------

bool ModeQRBotsRTL::request(uint32_t target_show_time_ms)
{
    const uint32_t now_ms = AP_HAL::millis();

    if (!motors->armed() || !AP::ahrs().home_is_set()) {
        gcs().send_text(MAV_SEVERITY_ERROR,
            "[%lu ms] QRBots RTL Request REJECTED: Motors disarmed or Home position lost",
            (unsigned long)now_ms);
        return false;
    }

    // Already running the RTL sequence: nothing to schedule.
    if (copter.flightmode == this) {
        return true;
    }

    _target_show_time_ms = target_show_time_ms;
    _origin_mode = copter.flightmode->mode_number();
    _pending = true;

    const int32_t current_show_time_ms = copter.g2.drone_show_manager.get_elapsed_time_since_start_msec();

    gcs().send_text(MAV_SEVERITY_INFO,
        "[%lu ms] QRBots RTL Request: latched TargetShowTime=%lu ms, CurrentShowTime=%ld ms (show continues)",
        (unsigned long)now_ms,
        (unsigned long)_target_show_time_ms,
        (long)current_show_time_ms);

    return true;
}

void ModeQRBotsRTL::cancel()
{
    if (!_pending) {
        return;
    }
    _pending = false;
    gcs().send_text(MAV_SEVERITY_INFO,
        "[%lu ms] QRBots RTL Request: cancelled", (unsigned long)AP_HAL::millis());
}

void ModeQRBotsRTL::check_pending()
{
    if (!_pending) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // Drop the request if we disarmed
    if (!motors->armed()) {
        _pending = false;
        gcs().send_text(MAV_SEVERITY_WARNING,
            "[%lu ms] QRBots RTL Request dropped: disarmed", (unsigned long)now_ms);
        return;
    }

    // Drop the request if the pilot/GCS changed mode after the command
    if (copter.flightmode->mode_number() != _origin_mode) {
        _pending = false;
        gcs().send_text(MAV_SEVERITY_WARNING,
            "[%lu ms] QRBots RTL Request dropped: flight mode changed", (unsigned long)now_ms);
        return;
    }

    const int32_t current_show_time_ms = copter.g2.drone_show_manager.get_elapsed_time_since_start_msec();
    if (current_show_time_ms < (int32_t)_target_show_time_ms) {
        return;   // not yet - the show keeps running untouched
    }

    // Target time reached: hand over to QRBots RTL
    _pending = false;

    gcs().send_text(MAV_SEVERITY_INFO,
        "[%lu ms] QRBots RTL Request due: CurrentShowTime=%ld ms >= TargetShowTime=%lu ms, switching mode",
        (unsigned long)now_ms,
        (long)current_show_time_ms,
        (unsigned long)_target_show_time_ms);

    if (copter.set_mode(mode_number(), ModeReason::GCS_COMMAND)) {
        return;
    }

    gcs().send_text(MAV_SEVERITY_ERROR,
        "[%lu ms] QRBots RTL switch FAILED, falling back to LAND", (unsigned long)now_ms);

    if (!copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND)) {
        gcs().send_text(MAV_SEVERITY_ERROR,
            "[%lu ms] QRBots RTL fallback to LAND FAILED", (unsigned long)now_ms);
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Current position as NEU [cm] from the EKF origin. Uses the NED getter (the
// one the drone show code already relies on) and converts the frame.
bool ModeQRBotsRTL::get_current_pos_neu_cm(Vector3f &pos_neu_cm) const
{
    Vector3f ned_m;
    if (!AP::ahrs().get_relative_position_NED_origin(ned_m)) {
        return false;
    }
    pos_neu_cm.x =  ned_m.x * 100.0f;
    pos_neu_cm.y =  ned_m.y * 100.0f;
    pos_neu_cm.z = -ned_m.z * 100.0f;
    return true;
}

void ModeQRBotsRTL::hold_position()
{
    Vector2p pos_xy = _hold_pos_cm.xy();
    Vector2f vel_xy;
    Vector2f acc_xy;
    float pos_z = _hold_pos_cm.z;
    float vel_z = 0.0f;

    pos_control->input_pos_vel_accel_xy(pos_xy, vel_xy, acc_xy, false);
    pos_control->input_pos_vel_accel_z(pos_z, vel_z, 0.0f, false);
}

// Trapezoidal (or triangular on short legs) velocity profile along the straight line:
//   accel phase  : 0 .. _ramp_time_s
//   cruise phase : at _cruise_cms (== V_i on normal legs)
//   decel phase  : last _ramp_time_s
// Returns position, velocity and acceleration feed-forward at time t_s.
void ModeQRBotsRTL::run_trajectory(float t_s, Vector3p &pos_cm, Vector3f &vel_cms, Vector3f &acc_cmss) const
{
    float s, v, a;

    if (t_s <= 0.0f) {
        s = 0.0f;
        v = 0.0f;
        a = _accel_cmss;
    } else if (t_s < _ramp_time_s) {
        s = 0.5f * _accel_cmss * t_s * t_s;
        v = _accel_cmss * t_s;
        a = _accel_cmss;
    } else if (t_s < _flight_time_s - _ramp_time_s) {
        s = _ramp_dist_cm + _cruise_cms * (t_s - _ramp_time_s);
        v = _cruise_cms;
        a = 0.0f;
    } else if (t_s < _flight_time_s) {
        const float t_left = _flight_time_s - t_s;
        s = _path_len_cm - 0.5f * _accel_cmss * t_left * t_left;
        v = _accel_cmss * t_left;
        a = -_accel_cmss;
    } else {
        s = _path_len_cm;
        v = 0.0f;
        a = 0.0f;
    }

    pos_cm   = _start_pos_cm + (_dir * s).topostype();
    vel_cms  = _dir * v;
    acc_cmss = _dir * a;
}

// Safety exit: hold at the current position (or the last commanded point if
// position is unavailable) and hand over to LAND through the normal path.
void ModeQRBotsRTL::abort_to_land(const char *reason, const Vector3f &cur_neu_cm, bool have_pos)
{
    if (have_pos) {
        _hold_pos_cm = cur_neu_cm.topostype();
    }
    _state = State::EXECUTE_LAND;
    gcs().send_text(MAV_SEVERITY_CRITICAL,
        "[%lu ms] QRBots RTL ABORT: %s, EXECUTE_LAND", (unsigned long)AP_HAL::millis(), reason);
}


// ---------------------------------------------------------------------------
// Mode
// ---------------------------------------------------------------------------

bool ModeQRBotsRTL::init(bool ignore_checks)
{
    const uint32_t now_ms = AP_HAL::millis();

    if (!motors->armed() || !AP::ahrs().home_is_set()) {
        gcs().send_text(MAV_SEVERITY_ERROR,
            "[%lu ms] QRBots RTL Init FAILED: Motors disarmed or Home position lost", (unsigned long)now_ms);
        return false;
    }

    Vector3f pos_neu_cm;
    if (!get_current_pos_neu_cm(pos_neu_cm)) {
        gcs().send_text(MAV_SEVERITY_ERROR,
            "[%lu ms] QRBots RTL Init FAILED: no position estimate", (unsigned long)now_ms);
        return false;
    }

    // Any latched request is consumed by entering the mode
    _pending = false;

    const int32_t current_show_time_ms = copter.g2.drone_show_manager.get_elapsed_time_since_start_msec();

    gcs().send_text(MAV_SEVERITY_INFO,
        "[%lu ms] QRBots RTL Init: TargetShowTime=%lu ms, CurrentShowTime=%ld ms",
        (unsigned long)now_ms,
        (unsigned long)_target_show_time_ms,
        (long)current_show_time_ms);

    if (current_show_time_ms < (int32_t)_target_show_time_ms) {
        gcs().send_text(MAV_SEVERITY_ERROR,
            "[%lu ms] QRBots RTL Init REJECTED: Target show time not reached (%ld < %lu ms)",
            (unsigned long)now_ms,
            (long)current_show_time_ms,
            (unsigned long)_target_show_time_ms);
        return false; // Blocks early manual entry completely
    }

    _wait_start_ms = now_ms;
    _last_land_retry_ms = 0;
    _err_active = false;
    _state = State::WAITING_5S;

    // Position controller: static hold at the current position
    _hold_pos_cm = pos_neu_cm.topostype();
    pos_control->init_xy_controller();
    pos_control->init_z_controller();

    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    gcs().send_text(MAV_SEVERITY_INFO,
        "[%lu ms] QRBots RTL State: WAITING_5S", (unsigned long)now_ms);

    return true;
}

void ModeQRBotsRTL::exit()
{
    // restore_position_limits();
}

void ModeQRBotsRTL::run()
{
    const uint32_t now_ms = AP_HAL::millis();

    // --- 1. Mode Switch Handoff ---
    if (_state == State::EXECUTE_LAND) {
        if (copter.set_mode(Mode::Number::LAND, ModeReason::UNKNOWN)) {
            gcs().send_text(MAV_SEVERITY_NOTICE,
                "[%lu ms] QRBots RTL: Handed off to LAND mode successfully", (unsigned long)now_ms);
            copter.flightmode->run();
            return;
        }

        // LAND was refused: keep holding position below and retry next loop
        if (now_ms - _last_land_retry_ms >= 2000) {
            _last_land_retry_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_ERROR,
                "[%lu ms] QRBots RTL: LAND handoff FAILED, holding and retrying", (unsigned long)now_ms);
        }
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // --- 2. State machine ---
    if (_state == State::WAITING_5S) {
        if (now_ms - _wait_start_ms >= 10000) {
            Vector3f target_pos_m;
            float target_vel_ms = 0.0f;
            uint32_t fetch_time_ms = _target_show_time_ms;
            Location target_loc;
            Vector3f target_neu_cm;
            Vector3f cur_neu_cm;

            bool ok = copter.g2.drone_show_manager.get_qrbots_rth_target(fetch_time_ms, target_pos_m, target_vel_ms);

            if (ok) {
                // get_qrbots_rth_target returns SHOW coordinates, so convert_show_pos_to_location IS correct here
                if (!copter.g2.drone_show_manager.convert_show_pos_to_location(target_pos_m, target_loc)) {   //NEED TO FIX
                    gcs().send_text(MAV_SEVERITY_CRITICAL,
                            "[%lu ms] QRBots RTL ABORT: Failed to convert show pos to location",
                            (unsigned long)now_ms);
                        _state = State::EXECUTE_LAND; // Immediately switch to LAND in place
                        return;
                }
                ok = target_loc.get_vector_from_origin_NEU(target_neu_cm) &&   // cm
                     get_current_pos_neu_cm(cur_neu_cm) &&
                     target_vel_ms > 0.1f;
            }

            if (ok) {
                _start_pos_cm = cur_neu_cm.topostype();
                _end_pos_cm   = target_neu_cm.topostype();

                const Vector3f delta = (_end_pos_cm - _start_pos_cm).tofloat();
                _path_len_cm = delta.length();

                if (_path_len_cm < 1.0f) {             //NEED TO FIX. ALL SHOULD LNAD IN SAME TIME. ELSE COLLISIONS CAN OCCUR. 
                    // Already at the target
                    _state = State::EXECUTE_LAND;
                    gcs().send_text(MAV_SEVERITY_INFO,
                        "[%lu ms] QRBots RTL: already at target, EXECUTE_LAND", (unsigned long)now_ms);
                } else {
                    _dir = delta / _path_len_cm;

                    // ---- Trapezoidal profile parameters ----
                    const float v_i = target_vel_ms * 100.0f;                        // V_i [cm/s]
                    // _accel_cmss = MAX(MIN(wp_nav->get_wp_acceleration(), MAX_PROFILE_ACCEL_CMSS), 10.0f);
                    // _accel_cmss = wp_nav->get_wp_acceleration();
                    _accel_cmss = MAX_PROFILE_ACCEL_CMSS;
                    

                    if (_path_len_cm >= v_i * v_i / _accel_cmss) {
                        // Full trapezoid: accel -> cruise at exactly V_i -> decel
                        _cruise_cms = v_i;
                        _flight_time_s = _path_len_cm / v_i + v_i / _accel_cmss;
                    } else {
                        // Short leg: triangular profile with a lower peak speed
                        _cruise_cms = sqrtf(_accel_cmss * _path_len_cm);
                        _flight_time_s = 2.0f * _cruise_cms / _accel_cmss;
                    }
                    _ramp_time_s = _cruise_cms / _accel_cmss;
                    _ramp_dist_cm = 0.5f * _accel_cmss * _ramp_time_s * _ramp_time_s;
                    _fly_start_ms = now_ms;
                    _err_active = false;


                    _state = State::FLY_TO_TARGET;
                    gcs().send_text(MAV_SEVERITY_INFO,
                        "[%lu ms] QRBots RTL State: FLY_TO_TARGET dist=%.1f m Vcruise=%.2f m/s a=%.2f m/s2 T=%.1f s",
                        (unsigned long)now_ms,
                        (double)(_path_len_cm * 0.01f), (double)(_cruise_cms * 0.01f),
                        (double)(_accel_cmss * 0.01f), (double)_flight_time_s);
                }
            } else {
                _state = State::EXECUTE_LAND;
                gcs().send_text(MAV_SEVERITY_WARNING,
                    "[%lu ms] QRBots RTL: no valid RTH target, landing", (unsigned long)now_ms);
            }
        }
    }

    // --- 3. Setpoint generation ---
    if (_state == State::FLY_TO_TARGET) {
        const float t_s = (now_ms - _fly_start_ms) * 0.001f;

        Vector3p pos_cm;
        Vector3f vel_cms;
        Vector3f acc_cmss;
        run_trajectory(t_s, pos_cm, vel_cms, acc_cmss);

        // --- Safety guards (evaluated every loop while following the schedule) ---
        Vector3f cur_neu_cm;
        const bool have_pos = get_current_pos_neu_cm(cur_neu_cm);
        bool aborted = false;

        if (!have_pos) {
            // Lost the position estimate: stop following a moving schedule
            abort_to_land("position estimate lost", cur_neu_cm, false);
            aborted = true;
        } else {
            const float track_err_cm = (cur_neu_cm - pos_cm.tofloat()).length();

            if (track_err_cm > MAX_TRACK_ERR_CM) {
                if (!_err_active) {
                    _err_active = true;
                    _err_start_ms = now_ms;
                    gcs().send_text(MAV_SEVERITY_WARNING,
                        "[%lu ms] QRBots RTL: tracking error %.1f m", (unsigned long)now_ms, (double)(track_err_cm * 0.01f));
                } else if (now_ms - _err_start_ms >= MAX_TRACK_ERR_TIME_MS) {
                    abort_to_land("tracking error too large", cur_neu_cm, true);
                    aborted = true;
                }
            } else {
                _err_active = false;
            }
        }

        if (aborted) {
            hold_position();   // hold the current point this loop; LAND handoff happens next loop
        } else {
            Vector2p pos_xy = pos_cm.xy();
            Vector2f vel_xy = vel_cms.xy();
            Vector2f acc_xy = acc_cmss.xy();
            float pos_z = pos_cm.z;
            float vel_z = vel_cms.z;

            // limit_output = false: follow the schedule strictly
            pos_control->input_pos_vel_accel_xy(pos_xy, vel_xy, acc_xy, false);
            pos_control->input_pos_vel_accel_z(pos_z, vel_z, acc_cmss.z, false);

            _hold_pos_cm = pos_cm;   // a LAND retry holds the last commanded point

            // Arrived: schedule finished AND physically close (or safety timeout)
            if (t_s >= _flight_time_s) {
                const float err_cm = (cur_neu_cm - _end_pos_cm.tofloat()).length();
                if (err_cm < ARRIVAL_RADIUS_CM || t_s > _flight_time_s + ARRIVAL_TIMEOUT_S) {
                    _state = State::EXECUTE_LAND;
                    gcs().send_text(MAV_SEVERITY_INFO,
                        "[%lu ms] QRBots RTL State: EXECUTE_LAND (err=%.0f cm)",
                        (unsigned long)now_ms, (double)err_cm);
                }
            }
        }
    } else {
        hold_position();   // WAITING_5S and LAND-retry hold
    }

    // --- 4. Position controller update ---
    pos_control->update_xy_controller();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}