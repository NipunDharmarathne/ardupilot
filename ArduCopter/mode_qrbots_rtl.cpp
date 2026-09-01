#include "Copter.h"
#include "mode_qrbots_rtl.h"

bool ModeQRBotsRTL::init(bool ignore_checks)
{
    // Ensure we have GPS lock, home is set, and motors are armed
    if (!motors->armed() || !AP::ahrs().home_is_set()) {
        return false;
    }

    // Initialize waypoint navigation
    wp_nav->wp_and_spline_init();
    
    // 1. Apply the defined velocity
    wp_nav->set_speed_xy(_target_velocity_cms);

    // 2. Convert local X/Y/Z offsets to global GPS navigation target
    Location target_loc = ahrs.get_home();
    target_loc.offset(_target_pos_cm.x / 100.0f, _target_pos_cm.y / 100.0f);
    target_loc.alt += _target_pos_cm.z; 

    // Command the autopilot to fly to this newly calculated GPS location
    wp_nav->set_wp_destination_loc(target_loc);

    // 3. Set the state machine to fly to the target
    _state = State::FLY_TO_TARGET;
    
    return true;
}

void ModeQRBotsRTL::run()
{
    // 4. Transition to LAND mode
    if (_state == State::EXECUTE_LAND) {
        // Exit this custom mode and trigger standard ArduCopter LAND
        copter.set_mode(Mode::Number::LAND, ModeReason::UNKNOWN);
        return;
    }

    // Ensure motors are running at full flight authority
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 5. Check if we have arrived at the target point
    if (_state == State::FLY_TO_TARGET) {
        if (wp_nav->reached_wp_destination()) {
            _state = State::EXECUTE_LAND; // Trigger landing on the next loop iteration
        }
    } 

    // --- Execute standard position and attitude controllers ---
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}