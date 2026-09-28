#include "Copter.h"
#include "mode_qrbots_rtl.h"

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

    // Any latched request is consumed by entering the mode
    _pending = false;

    const int32_t current_show_time_ms = copter.g2.drone_show_manager.get_elapsed_time_since_start_msec();

    gcs().send_text(MAV_SEVERITY_INFO,
        "[%lu ms] QRBots RTL Init: TargetShowTime=%lu ms, CurrentShowTime=%ld ms",
        (unsigned long)now_ms,
        (unsigned long)_target_show_time_ms,
        (long)current_show_time_ms);

    if (current_show_time_ms < (int32_t)_target_show_time_ms) {
        // Only happens if the mode was selected directly (not via request()).
        // A direct selection is an explicit override, so the sequence starts now.
        gcs().send_text(MAV_SEVERITY_WARNING,
            "[%lu ms] QRBots RTL Init: entered %ld ms before target show time, starting immediately",
            (unsigned long)now_ms,
            (long)((int32_t)_target_show_time_ms - current_show_time_ms));
    }

    _wait_start_ms = now_ms;
    _last_land_retry_ms = 0;
    _state = State::WAITING_5S;

    // Initialize wp_nav for a static position hold
    wp_nav->wp_and_spline_init();

    gcs().send_text(MAV_SEVERITY_INFO,
        "[%lu ms] QRBots RTL State: WAITING_5S", (unsigned long)now_ms);

    return true;
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

    // --- 2. State Machine Logic ---
    if (_state == State::WAITING_5S) {
        const uint32_t elapsed_wait_ms = now_ms - _wait_start_ms;

        if (elapsed_wait_ms >= 5000) {
            Vector3f target_pos_m;
            float target_vel_ms;
            uint32_t fetch_time_ms = _target_show_time_ms;

            // get_qrbots_rth_target returns SHOW coordinates, so convert_show_pos_to_location IS correct here
            if (copter.g2.drone_show_manager.get_qrbots_rth_target(fetch_time_ms, target_pos_m, target_vel_ms)) {
                wp_nav->set_speed_xy(target_vel_ms * 100.0f);
                Location target_loc;

                if (copter.g2.drone_show_manager.convert_show_pos_to_location(target_pos_m, target_loc)) {
                    wp_nav->set_wp_destination_loc(target_loc);
                } else {
                    target_loc = ahrs.get_home();
                    target_loc.offset(target_pos_m.x, target_pos_m.y);
                    target_loc.set_alt_cm((int32_t)(target_pos_m.z * 100.0f), Location::AltFrame::ABOVE_HOME);
                    wp_nav->set_wp_destination_loc(target_loc);
                }

                _state = State::FLY_TO_TARGET;
                gcs().send_text(MAV_SEVERITY_INFO,
                    "[%lu ms] QRBots RTL State: FLY_TO_TARGET", (unsigned long)now_ms);
            } else {
                _state = State::EXECUTE_LAND;
                gcs().send_text(MAV_SEVERITY_WARNING,
                    "[%lu ms] QRBots RTL: no RTH target available, landing", (unsigned long)now_ms);
            }
        }
    }
    else if (_state == State::FLY_TO_TARGET) {
        if (wp_nav->reached_wp_destination() && wp_nav->get_wp_distance_to_destination() < 50.0f) {
            _state = State::EXECUTE_LAND;
            gcs().send_text(MAV_SEVERITY_INFO,
                "[%lu ms] QRBots RTL State: EXECUTE_LAND", (unsigned long)now_ms);
        }
    }

    // --- 3. Standard waypoint navigation for WAITING_5S and FLY_TO_TARGET ---
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}