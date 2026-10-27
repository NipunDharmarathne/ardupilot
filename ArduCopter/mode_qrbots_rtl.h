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

    // ---- Tunables (start conservative, raise only after checking logs) ----
    static constexpr float    MAX_PROFILE_ACCEL_CMSS   = 400.0f;   // cap on ramp up/down acceleration [cm/s^2]

    static constexpr float    ARRIVAL_RADIUS_CM        = 20.0f;    // "arrived" distance to the end point
    static constexpr float    ARRIVAL_TIMEOUT_S        = 10.0f;    // give up waiting for arrival after schedule end
    static constexpr float    MAX_TRACK_ERR_CM         = 300.0f;   // tracking error that triggers the abort timer
    static constexpr uint32_t MAX_TRACK_ERR_TIME_MS    = 1000;     // how long the error must persist before aborting

    // --- helpers ---
    // Current position as NEU in cm from the EKF origin (same frame mode_guided gives pos_control)
    bool get_current_pos_neu_cm(Vector3f &pos_neu_cm) const;
    // Command a static position hold at _hold_pos_cm
    void hold_position();
    // Desired state on the trapezoidal 1D profile along the straight line at time t_s
    void run_trajectory(float t_s, Vector3p &pos_cm, Vector3f &vel_cms, Vector3f &acc_cmss) const;
    // Hold where we are and go to the LAND handoff (used by the safety guards)
    void abort_to_land(const char *reason, const Vector3f &cur_neu_cm, bool have_pos);
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
    Vector3f       _dir;                  // unit vector start -> end
    Vector3p       _hold_pos_cm;          // used while waiting and while LAND handoff is retried
    float          _path_len_cm = 0.0f;   // distance start -> end
    float          _accel_cmss = 0.0f;    // ramp acceleration
    float          _cruise_cms = 0.0f;    // V_i, or lower peak speed on short legs (triangular profile)
    float          _ramp_time_s = 0.0f;   // duration of accel (and decel) phase
    float          _ramp_dist_cm = 0.0f;  // distance covered during accel (and decel) phase
    float          _flight_time_s = 0.0f; // total duration of the profile
    uint32_t       _fly_start_ms = 0;

    // bool           _limits_modified = false;

    // --- tracking-error guard ---
    bool           _err_active = false;
    uint32_t       _err_start_ms = 0;
};