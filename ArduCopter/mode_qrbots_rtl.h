#pragma once

// NOTE: reconstructed from the .cpp. Merge with your existing header if it
// contains anything else (mode number enum entry, extra overrides, etc).
// This header is expected to be included from mode.h, like the other Copter modes.

class ModeQRBotsRTL : public Mode {

public:
    ModeQRBotsRTL(void) : Mode() {}

    Number mode_number() const override { return Number::QRBOTS_RTL; }   // adjust to your enum name

    // --- Scheduled request API (called from the MAVLink command handler / main loop) ---
    // Latches the target show time; the show keeps running untouched until it is reached.
    bool request(uint32_t target_show_time_ms);
    void cancel();
    // Must be called every loop (e.g. from a scheduler task) to fire a latched request.
    void check_pending();
    bool is_pending() const { return _pending; }

    // --- Mode interface ---
    bool init(bool ignore_checks) override;
    void run() override;
    void exit() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    bool is_autopilot() const override { return true; }

protected:
    const char *name() const override { return "QRBOTS_RTL"; }
    const char *name4() const override { return "QRTL"; }

private:
    enum class State : uint8_t {
        WAITING_5S,
        FLY_TO_TARGET,
        EXECUTE_LAND,
    };

    // --- helpers ---
    // Current position as NEU in cm from the EKF origin (same frame mode_guided gives pos_control)
    bool get_current_pos_neu_cm(Vector3f &pos_neu_cm) const;
    // Command a static position hold at _hold_pos_cm
    void hold_position();
    // Desired position/velocity on the straight line at time t since the start of the flight
    void run_trajectory(float t_s, Vector3p &pos_cm, Vector3f &vel_cms) const;
    // Restore pos_control limits to the waypoint-nav defaults
    void restore_position_limits();

    // --- scheduled request state ---
    bool           _pending = false;
    uint32_t       _target_show_time_ms = 0;
    Mode::Number   _origin_mode = Mode::Number::DRONE_SHOW;

    // --- state machine ---
    State          _state = State::WAITING_5S;
    uint32_t       _wait_start_ms = 0;
    uint32_t       _last_land_retry_ms = 0;

    // --- 4D trajectory (all NEU, cm from EKF origin) ---
    Vector3p       _start_pos_cm;
    Vector3p       _end_pos_cm;
    Vector3f       _dir;              // unit vector start -> end
    Vector3p       _hold_pos_cm;      // used while waiting and while LAND handoff is retried
    float          _speed_cms = 0.0f; // V_i
    float          _flight_time_s = 0.0f;
    uint32_t       _fly_start_ms = 0;
    bool           _limits_modified = false;
};