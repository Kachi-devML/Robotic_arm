#pragma once

#include <math.h>
#include <stdint.h>

#include "ServoConfig.h"

namespace Motion {

enum class ServoId : uint8_t {
    SHOULDER = 0,
    ELBOW = 1,
    WRIST = 2,
    GRIPPER = 3,
};

struct ServoDriver {
    struct PulseRange {
        uint16_t min_us;
        uint16_t max_us;
        float max_deg;
    };

    static constexpr PulseRange kShoulderPulse = {600, 2400, 180.0f};
    static constexpr PulseRange kElbowPulse = {600, 2400, 180.0f};
    static constexpr PulseRange kWristPulse = {600, 2400, 180.0f};
    static constexpr PulseRange kGripperPulse = {600, 2400, 90.0f};

    static constexpr uint8_t kChannelShoulder = static_cast<uint8_t>(ServoId::SHOULDER);
    static constexpr uint8_t kChannelElbow = static_cast<uint8_t>(ServoId::ELBOW);
    static constexpr uint8_t kChannelWrist = static_cast<uint8_t>(ServoId::WRIST);
    static constexpr uint8_t kChannelGripper = static_cast<uint8_t>(ServoId::GRIPPER);

    static inline uint16_t pulseToDuty(uint16_t pulse_us) {
        return static_cast<uint16_t>(
            round(static_cast<float>(pulse_us) * static_cast<float>(1u << ServoConfig::kPwmResolutionBits) /
                  20000.0f));
    }

    static inline uint16_t angleToPulseUs(float angle_deg, const PulseRange& range) {
        const float clamped = fminf(fmaxf(angle_deg, 0.0f), range.max_deg);
        const float frac = range.max_deg > 0.0f ? (clamped / range.max_deg) : 0.0f;
        return static_cast<uint16_t>(range.min_us + frac * (range.max_us - range.min_us) + 0.5f);
    }

    static bool begin();
    static void writeAngleDeg(ServoId id, float angle_deg);
    static void writeAngleDeg(uint8_t channel, float angle_deg,
                              const PulseRange& range);
    static void setAngleDeg(uint8_t channel, float angle_deg,
                            const PulseRange& range);
    static void setAngleRad(uint8_t channel, float angle_rad,
                            const PulseRange& range);
    static void setPulseUs(uint8_t channel, uint16_t pulse_us);
    static void freeze();
    static void freezeAll();
    static void disableAll();
};

}  // namespace Motion
