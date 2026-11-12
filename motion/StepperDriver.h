#pragma once

#include <math.h>
#include <stdint.h>

#include "MotionConfig.h"

namespace Motion {

struct StepperDriver {
    static constexpr float kStepsPerOutputDeg = MotionConfig::STEPPER_STEPS_PER_OUTPUT_DEG;

    static inline int32_t stepsFromDeg(float angle_deg) {
        return static_cast<int32_t>(lroundf(angle_deg * kStepsPerOutputDeg));
    }

    static inline float degFromSteps(int32_t steps) {
        return static_cast<float>(steps) / kStepsPerOutputDeg;
    }

    static inline int32_t radiansToSteps(float radians) {
        return stepsFromDeg(radians * 180.0f / 3.14159265358979323846f);
    }

    static inline float stepsToRadians(int32_t steps) {
        return degFromSteps(steps) * 3.14159265358979323846f / 180.0f;
    }

    static void begin();
    static void stop();
    static void setTargetStep(int32_t step);
    static int32_t currentStep();
    static void zeroPosition();
    static void enable();
    static void release();
    static bool isReady();
    static void setEnabled(bool enable);
    static bool isEnabled();
    static void emergencyStop();
    static bool atTarget();
    static void setMicrosteps(uint16_t microsteps);
};

}  // namespace Motion
