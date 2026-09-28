#pragma once

// No includes on purpose: like the other Copter modes, this is expected to be
// pulled in after class Mode is defined. If your tree includes it differently,
// keep your existing include arrangement.
//
// If you already have overrides here (allows_arming signature, name(), etc.),
// keep yours and only merge the "scheduled-RTL API" block and the private members.

class ModeQRBotsRTL : public Mode {

public:
    using Mode::Mode;

    Number mode_number() const override { return Number::QRBOTS_RTL; }   // adjust to your enum name

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    bool is_autopilot() const override { return true; }

    // ---- scheduled-RTL API -------------------------------------------------
    // Latch an RTL request. The vehicle stays in its current mode (native
    // DroneShow) and is only switched to this mode once the show clock reaches
    // target_show_time_ms. Returns false if the request cannot be accepted.
    bool request(uint32_t target_show_time_ms);

    // Drop a latched request without switching modes.
    void cancel();

    bool is_pending() const { return _pending; }

    // Call from the scheduler at >= 50 Hz. Performs the mode switch when due.
    void check_pending();
    // ------------------------------------------------------------------------

protected:
    const char *name() const override { return "QRBOTS_RTL"; }
    const char *name4() const override { return "QRTL"; }

private:
    enum class State : uint8_t {
        WAITING_5S,
        FLY_TO_TARGET,
        EXECUTE_LAND
    };

    State _state = State::WAITING_5S;

    uint32_t _target_show_time_ms = 0;
    uint32_t _wait_start_ms = 0;
    uint32_t _last_land_retry_ms = 0;

    // latched request
    bool _pending = false;
    Number _origin_mode = Number::STABILIZE;   // mode active when request() was accepted
};