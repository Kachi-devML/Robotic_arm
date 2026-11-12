#pragma once

#include <stdint.h>

// Two-stage base homing plus an immediate servo snap (AGENT_BUILD_SPEC §4).
//
// Pure sequence: the caller reads the switch, reads the step count, and applies
// the returned target. No GPIO and no delay().
//
//   1. Snap θ1/θ2/θ3 to 90° and open the gripper (one-shot flag).
//   2. Seek HOME_SW at kHomingSeekDegPerSec. Direction is toward decreasing steps.
//   3. Back off kHomingBackoffDeg so the switch releases.
//   4. Creep back at kHomingCreepDegPerSec until the switch closes again.
//   5. Caller zeroes the step counter when zeroNow is set.
//
// HOME_SW polarity (external pull-up, switch to GND): triggered == pressed == LOW.
// Pass that as `homeTriggered == true`. Timeout or excess travel → Failed.

namespace Control {

class HomingSequence {
 public:
    enum class Phase : uint8_t {
        Idle = 0,
        Snap,
        Seek,
        BackOff,
        Creep,
        Done,
        Failed,
        Aborted,
    };

    struct Status {
        Phase phase;
        int32_t targetStep;
        bool enableStepper;
        bool snapServos;
        bool zeroNow;
        const char* failReason;
    };

    void start(uint32_t now_ms);
    void abort();

    /// `homeTriggered` is true when the limit switch is pressed.
    Status tick(uint32_t now_ms, bool homeTriggered, int32_t currentStep);

 private:
    Phase phase_ = Phase::Idle;
    uint32_t startMs_ = 0;
    uint32_t phaseMs_ = 0;
    uint32_t lastMs_ = 0;
    int32_t commanded_ = 0;
    int32_t contact_ = 0;
    int32_t seekOrigin_ = 0;
    int32_t creepOrigin_ = 0;
    int32_t backoffSteps_ = 0;
    int32_t maxTravelSteps_ = 0;
    int32_t creepMaxSteps_ = 0;
    float accum_ = 0.0f;
    float stepsPerDeg_ = 0.0f;
    bool snapped_ = false;
    const char* failReason_ = nullptr;

    void enter(Phase phase, uint32_t now_ms, int32_t currentStep);
    void fail(const char* reason);
    int32_t integrate(uint32_t now_ms, float degPerSec, int sign);
    Status make(int32_t currentStep, bool enable, bool snap, bool zero) const;
};

}  // namespace Control
