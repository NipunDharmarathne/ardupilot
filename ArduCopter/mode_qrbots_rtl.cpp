#include "Copter.h"
#include "mode_qrbots_rtl.h"

bool ModeQRBotsRTL::init(bool ignore_checks)
{
    // Ensure GPS lock, home position, and armed motors
    if (!motors->armed() || !AP::ahrs().home_is_set()) {
        return false;
    }

    // Initialize waypoint navigation (loiter in place initially)
    wp_nav->wp_and_spline_init();
    
    // Start 5-second timer
    _wait_start_ms = AP_HAL::millis();
    _state = State::WAITING_5S; 
    
    gcs().send_text(MAV_SEVERITY_INFO, "QRBots RTL: Loitering for 5s...");
    return true;
}

void ModeQRBotsRTL::run()
{
    // --- 1. Mode Switch Handoff ---
    if (_state == State::EXECUTE_LAND) {
        if (copter.set_mode(Mode::Number::LAND, ModeReason::UNKNOWN)) {
            // Hand control off to standard LAND mode for this loop tick
            copter.flightmode->run();
            return;
        }
        // Fall through if set_mode fails so drone continues loitering
    }

    // Ensure motors maintain full flight authority
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // --- 2. State Machine Logic ---
    if (_state == State::WAITING_5S) {
        // Wait 5 seconds after RTL activation
        if (AP_HAL::millis() - _wait_start_ms >= 5000) {
            
            Vector3f target_pos_m;
            float target_vel_ms;
            uint32_t fetch_time_ms = 60000; // 60s target lookup time

            if (copter.g2.drone_show_manager.get_qrbots_rth_target(fetch_time_ms, target_pos_m, target_vel_ms)) {
                
                // Set horizontal waypoint velocity (m/s -> cm/s)
                wp_nav->set_speed_xy(target_vel_ms * 100.0f);

                Location target_loc;
                
                // 1. Convert local show grid coordinates (meters) to global GPS location via Skybrush origin system
                if (copter.g2.drone_show_manager.convert_show_pos_to_location(target_pos_m, target_loc)) {
                    
                    wp_nav->set_wp_destination_loc(target_loc);
                    _state = State::FLY_TO_TARGET;
                    gcs().send_text(MAV_SEVERITY_INFO, "QRBots RTL: Flying to Show Target");

                } else {
                    // Fallback: If show coordinate system is invalid, use Home location with local offsets
                    target_loc = ahrs.get_home();
                    target_loc.offset(target_pos_m.x, target_pos_m.y);
                    target_loc.set_alt_cm((int32_t)(target_pos_m.z * 100.0f), Location::AltFrame::ABOVE_HOME);

                    wp_nav->set_wp_destination_loc(target_loc);
                    _state = State::FLY_TO_TARGET;
                    
                    gcs().send_text(MAV_SEVERITY_WARNING, "QRBots RTL: Show origin missing, fallback to Home offset");
                }

            } else {
                // Failsafe: Land immediately if no RTH target plan is found
                gcs().send_text(MAV_SEVERITY_WARNING, "QRBots RTL: No Plan Found, Landing");
                _state = State::EXECUTE_LAND;
            }
        }
    } 
    else if (_state == State::FLY_TO_TARGET) {
        // Check arrival status and distance threshold (< 50 cm) to prevent premature landing handoff
        if (wp_nav->reached_wp_destination() && wp_nav->get_wp_distance_to_destination() < 50.0f) {
            gcs().send_text(MAV_SEVERITY_INFO, "QRBots RTL: Target Reached, Landing");
            _state = State::EXECUTE_LAND;
        }
    } 

    // --- 3. Execute Control Loops ---
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}