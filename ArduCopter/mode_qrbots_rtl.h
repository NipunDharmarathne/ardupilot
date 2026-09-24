#pragma once
#include "mode.h"

class ModeQRBotsRTL : public Mode {
public:
    bool init(bool ignore_checks) override;
    void run() override;

    // Standard mode overrides
    Number mode_number() const override { return Number::QRBOTS_RTL; }
    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    
    bool is_autopilot() const override { return true; }
    const char *name() const override { return "QRBOTS_RTL"; }
    const char *name4() const override { return "QRTL"; }

private:
    enum class State {
        WAITING_30S,
        FLY_TO_TARGET,
        EXECUTE_LAND
    };

    State _state;
    uint32_t _wait_start_ms;
};