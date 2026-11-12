#pragma once

#include <stdint.h>

#include "Pins.h"

namespace Motion {

struct ServoConfig {
    static constexpr uint32_t kPwmFrequencyHz = 50;
    static constexpr uint32_t kPwmResolutionBits = 14;
    static constexpr uint32_t kPwmPeriodUs = 20000;
    static constexpr uint32_t kPwmMaxDuty = (1u << kPwmResolutionBits) - 1u;

    struct Channel {
        uint8_t pin;
        uint16_t pulseMinUs;
        uint16_t pulseMaxUs;
        float angleRangeDeg;
        bool invert;
        int16_t trimUs;
    };

    static constexpr Channel kShoulder = {
        Pins::kServoShoulderPwm,
        600,
        2400,
        180.0f,
        false,
        0,
    };
    static constexpr Channel kElbow = {
        Pins::kServoElbowPwm,
        600,
        2400,
        180.0f,
        false,
        0,
    };
    static constexpr Channel kWrist = {
        Pins::kServoWristPwm,
        600,
        2400,
        180.0f,
        false,
        0,
    };
    static constexpr Channel kGripper = {
        Pins::kServoGripperPwm,
        600,
        2400,
        90.0f,
        false,
        0,
    };
};

}  // namespace Motion
