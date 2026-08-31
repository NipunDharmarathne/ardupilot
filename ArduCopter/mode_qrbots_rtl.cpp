#include "Copter.h"
#include "mode_qrbots_rtl.h"

// 1. Determine current show timestamp
uint32_t ModeQRBotsRTL::get_show_time_ms() const {
    // Calculates how long the show has been running based on system clock
    return AP_HAL::millis() - _show_start_time_ms;
}

bool ModeQRBotsRTL::init(bool ignore_checks)
{
    if (!motors->armed() || !AP::ahrs().home_is_set()) {
        return false;
    }

    wp_nav->wp_and_spline_init();
    
    uint32_t current_show_time = get_show_time_ms();
    _target_idx = -1;

    // 2. Find next trajectory timestamp
    for (uint8_t i = 0; i < _schedule_length; i++) {
        // 6. Handle exact timestamp match
        if (_schedule[i].time_ms == current_show_time) {
            _state = State::HOLD_POSITION;
            _hold_start_ms = AP_HAL::millis();
            
            // Command it to hold exactly where it is right now
            wp_nav->set_wp_destination_loc(copter.current_loc);
            return true;
        } 
        // Find the strictly NEXT timestamp
        else if (_schedule[i].time_ms > current_show_time) {
            _target_idx = i;
            break;
        }
    }

    // 7. No future timestamp available
    if (_target_idx == -1) {
        // The show is already over. Go straight to standard RTL.
        _state = State::EXECUTE_RTL;
        return true;
    }

    // 3. Continue trajectory
    _state = State::CONTINUE_TRAJECTORY;
    setup_target(_target_idx);
    
    return true;
}

void ModeQRBotsRTL::setup_target(uint8_t idx)
{
    // Apply velocity
    wp_nav->set_speed_xy(_schedule[idx].velocity_cms);

    // Convert local X/Y/Z offsets to global GPS navigation target
    Location target_loc = ahrs.get_home();
    target_loc.offset(_schedule[idx].pos_cm.x / 100.0f, _schedule[idx].pos_cm.y / 100.0f);
    target_loc.alt += _schedule[idx].pos_cm.z; 

    wp_nav->set_wp_destination_loc(target_loc);
}

void ModeQRBotsRTL::run()
{
    // 5. Transition to RTL
    if (_state == State::EXECUTE_RTL) {
        // Exit this custom mode and trigger standard ArduCopter RTL
        copter.set_mode(Mode::Number::RTL, ModeReason::UNKNOWN);
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    if (_state == State::CONTINUE_TRAJECTORY) {
        uint32_t current_show_time = get_show_time_ms();
        bool reached_wp = wp_nav->reached_wp_destination();
        bool time_expired = current_show_time >= _schedule[_target_idx].time_ms;

        // If we physically reached it OR the show clock says it's time
        if (reached_wp || time_expired) {
            _state = State::HOLD_POSITION;
            _hold_start_ms = AP_HAL::millis();

            // If time expired but we weren't physically there yet, stop moving immediately
            if (time_expired && !reached_wp) {
                wp_nav->set_wp_destination_loc(copter.current_loc);
            }
        }
    } 
    // 4. Hold at the waypoint for configurable duration
    else if (_state == State::HOLD_POSITION) {
        if ((AP_HAL::millis() - _hold_start_ms) >= _hold_duration_ms) {
            _state = State::EXECUTE_RTL; // Time is up, hand over to RTL
        }
    }

    // --- Execute standard position and attitude controllers ---
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}