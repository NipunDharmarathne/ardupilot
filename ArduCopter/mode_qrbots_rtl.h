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
        FLY_TO_TARGET,
        EXECUTE_LAND
    };

    State _state;

    // --- DEFINED TARGET SETTINGS ---
    // Target X, Y, Z in cm relative to home (e.g., 1000cm = 10 meters)
    const Vector3f _target_pos_cm = Vector3f(1500, 1500, 500); 
    
    // Target velocity in cm/s (e.g., 300cm/s = 3 m/s)
    const float _target_velocity_cms = 300; 
};