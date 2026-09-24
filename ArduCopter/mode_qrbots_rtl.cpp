#include "Copter.h"
#include "mode_qrbots_rtl.h"

// Note: Ensure your show manager header is included if not already in Copter.h
// #include "AC_DroneShowManager/AC_DroneShowManager.h"

bool ModeQRBotsRTL::init(bool ignore_checks)
{
    // Ensure we have GPS lock, home is set, and motors are armed
    if (!motors->armed() || !AP::ahrs().home_is_set()) {
        return false;
    }

    // Initialize waypoint navigation (automatically sets destination to current position to stop/loiter)
    wp_nav->wp_and_spline_init();
    
    // Start the 30-second timer and set state
    _wait_start_ms = AP_HAL::millis();
    _state = State::WAITING_30S;
    
    gcs().send_text(MAV_SEVERITY_INFO, "QRBots RTL: Loitering for 30s...");
    return true;
}

void ModeQRBotsRTL::run()
{
    // Transition to LAND mode
    if (_state == State::EXECUTE_LAND) {
        copter.set_mode(Mode::Number::LAND, ModeReason::UNKNOWN);
        return;
    }

    // Ensure motors are running at full flight authority
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // --- State Machine ---
    if (_state == State::WAITING_30S) {
        // Wait until 30,000 milliseconds (30s) have passed since mode init
        if (AP_HAL::millis() - _wait_start_ms >= 30000) {
            
            Vector3f target_pos_m;
            float target_vel_ms;

            // Fetch the target from the show file. 
            // Depending on your ArduPilot fork, your show manager instance might be accessed via:
            // copter.drone_show, AP::drone_show(), or AC_DroneShowManager::get_singleton()
            
            // NOTE: If you want the target for exactly 30s into the show clock, pass 30000.
            // If you want the target for the *current* real-time show clock, pass the active show time.
            uint32_t fetch_time_ms = 60000; 

            if (copter.g2.drone_show_manager.get_qrbots_rth_target(fetch_time_ms, target_pos_m, target_vel_ms)) {
                
                // 1. Apply the defined velocity (convert m/s to cm/s)
                wp_nav->set_speed_xy(target_vel_ms * 100.0f);

                // 2. Convert local offsets (meters) to global GPS navigation target
                Location target_loc = ahrs.get_home();
                target_loc.offset(target_pos_m.x, target_pos_m.y);
                target_loc.alt += (target_pos_m.z * 100.0f); // Convert alt to cm

                // Command the autopilot to fly to this newly calculated GPS location
                wp_nav->set_wp_destination_loc(target_loc);
                _state = State::FLY_TO_TARGET;
                
                gcs().send_text(MAV_SEVERITY_INFO, "QRBots RTL: Flying to Show Target");
            } else {
                // Failsafe: If no RTH plan was loaded, just land immediately
                gcs().send_text(MAV_SEVERITY_WARNING, "QRBots RTL: No Plan Found, Landing");
                _state = State::EXECUTE_LAND;
            }
        }
    } 
    else if (_state == State::FLY_TO_TARGET) {
        // Check if we have arrived at the target point
        if (wp_nav->reached_wp_destination()) {
            _state = State::EXECUTE_LAND; // Trigger landing on the next loop iteration
        }
    } 

    // --- Execute standard position and attitude controllers ---
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}