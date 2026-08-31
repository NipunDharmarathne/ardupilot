#pragma once
#include "mode.h"

struct QRBotPlan {
    uint32_t time_ms;    // Timestamp relative to show start (e.g., 20000 for 20s)
    Vector3f pos_cm;     // Target X, Y, Z in cm relative to origin/home
    float velocity_cms;  // Target velocity in cm/s
};

class ModeQRBotsRTL : public Mode {
public:
    bool init(bool ignore_checks) override;
    void run() override;

    // --- ADD THESE 4 MISSING PURE VIRTUAL FUNCTIONS ---
    Number mode_number() const override { return Number::QRBOTS_RTL; }
    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    // --------------------------------------------------

    bool is_autopilot() const override { return true; }
    const char *name() const override { return "QRBOTS_RTL"; }
    const char *name4() const override { return "QRTL"; }

private:
    enum class State {
        CONTINUE_TRAJECTORY,
        HOLD_POSITION,
        EXECUTE_RTL
    };

    State _state;
    int16_t _target_idx;
    uint32_t _hold_start_ms;
    
    // Configurable hold duration (could be linked to an AP_Int16 parameter)
    const uint32_t _hold_duration_ms = 2000; 

    // Time the show started relative to system boot
    // In a real system, this is set by your light show coordinator
    uint32_t _show_start_time_ms = 0; 

    uint32_t get_show_time_ms() const;
    void setup_target(uint8_t idx);

    // Example trajectory schedule
    static const uint8_t _schedule_length = 3;
    QRBotPlan _schedule[_schedule_length] = {
        {10000, Vector3f(1000, 1000, 500), 500},
        {20000, Vector3f(2000, 2000, 500), 300},
        {30000, Vector3f(3000, 3000, 500), 400}
    };
};