#include "ServoDriver.h"

#include <Arduino.h>

#include "JointLimits.h"
#include "Logger.h"
#include "SimMode.h"

namespace Motion {

namespace {

constexpr uint8_t kLedcChannels[] = {0, 1, 2, 3};
constexpr uint8_t kLedcPins[] = {
    ServoConfig::kShoulder.pin,
    ServoConfig::kElbow.pin,
    ServoConfig::kWrist.pin,
    ServoConfig::kGripper.pin,
};

static bool sInitialized = false;

static uint16_t pulseToDutyRaw(uint16_t pulse_us) {
    return static_cast<uint16_t>(
        round(static_cast<float>(pulse_us) * static_cast<float>(1u << ServoConfig::kPwmResolutionBits) /
              20000.0f));
}

static const ServoConfig::Channel& configForId(ServoId id) {
    switch (id) {
        case ServoId::SHOULDER: return ServoConfig::kShoulder;
        case ServoId::ELBOW: return ServoConfig::kElbow;
        case ServoId::WRIST: return ServoConfig::kWrist;
        case ServoId::GRIPPER: return ServoConfig::kGripper;
    }
    return ServoConfig::kShoulder;
}

static ServoDriver::PulseRange pulseRangeFor(const ServoConfig::Channel& config) {
    return {config.pulseMinUs, config.pulseMaxUs, config.angleRangeDeg};
}

static void logClamps(ServoId id, const ServoDriver::AngleConversion& conversion) {
    if (conversion.jointClamped) {
        LOG_WARN("[ServoDriver] ch%u joint angle clamped to soft limit %.2f",
                 static_cast<unsigned>(id), conversion.jointAngleDeg);
    }
    if (conversion.servoClamped) {
        LOG_WARN("[ServoDriver] ch%u mapped servo angle %.2f clamped to %.2f",
                 static_cast<unsigned>(id), conversion.mappedServoAngleDeg,
                 conversion.servoAngleDeg);
    }
}

}  // namespace

bool ServoDriver::begin() {
    if (SimMode::isEnabled()) {
        LOG_INFO("[ServoDriver] SIM: begin() — direct LEDC output setup skipped");
        sInitialized = true;
        return true;
    }

    for (size_t i = 0; i < 4; ++i) {
        const ServoId id = static_cast<ServoId>(i);
        const auto& config = configForId(id);
        const float homeJointDeg = id == ServoId::GRIPPER ? 0.0f : 90.0f;
        const AngleConversion home =
            convertAngleDeg(homeJointDeg, config, pulseRangeFor(config));
        ledcSetup(kLedcChannels[i], ServoConfig::kPwmFrequencyHz, ServoConfig::kPwmResolutionBits);
        ledcAttachPin(kLedcPins[i], kLedcChannels[i]);
        ledcWrite(kLedcChannels[i], pulseToDutyRaw(home.pulseUs));
        logClamps(id, home);
    }

    sInitialized = true;
    LOG_INFO("[ServoDriver] direct LEDC attached on GPIO4-7, 50 Hz, 14-bit resolution");
    return true;
}

void ServoDriver::writeAngleDeg(ServoId id, float angle_deg) {
    const auto& cfg = configForId(id);
    const AngleConversion conversion =
        convertAngleDeg(angle_deg, cfg, pulseRangeFor(cfg));
    logClamps(id, conversion);
    if (SimMode::isEnabled()) {
        LOG_DEBUG("[ServoDriver] SIM: ch%u joint=%.2f mapped=%.2f servo=%.2f",
                  static_cast<unsigned>(id), conversion.jointAngleDeg,
                  conversion.mappedServoAngleDeg, conversion.servoAngleDeg);
    }
    setPulseUs(static_cast<uint8_t>(id), conversion.pulseUs);
}

void ServoDriver::writeRawAngleDeg(ServoId id, float servo_angle_deg) {
    const auto& cfg = configForId(id);
    const PulseRange range = pulseRangeFor(cfg);
    const float servoAngle =
        fminf(fmaxf(servo_angle_deg, 0.0f), range.max_deg);
    if (servoAngle != servo_angle_deg) {
        LOG_WARN("[ServoDriver] raw ch%u angle %.2f clamped to %.2f",
                 static_cast<unsigned>(id), servo_angle_deg, servoAngle);
    }
    setPulseUs(static_cast<uint8_t>(id), angleToPulseUs(servoAngle, range));
}

void ServoDriver::writeAngleDeg(uint8_t channel, float angle_deg,
                               const PulseRange& range) {
    const ServoId id = static_cast<ServoId>(channel);
    const auto& cfg = configForId(id);
    const AngleConversion conversion = convertAngleDeg(angle_deg, cfg, range);
    logClamps(id, conversion);
    if (SimMode::isEnabled()) {
        LOG_DEBUG("[ServoDriver] SIM: ch%u joint=%.2f mapped=%.2f servo=%.2f",
                  channel, conversion.jointAngleDeg,
                  conversion.mappedServoAngleDeg, conversion.servoAngleDeg);
    }
    setPulseUs(channel, conversion.pulseUs);
}

void ServoDriver::setAngleDeg(uint8_t channel, float angle_deg,
                               const PulseRange& range) {
    writeAngleDeg(channel, angle_deg, range);
}

void ServoDriver::setAngleRad(uint8_t channel, float angle_rad,
                               const PulseRange& range) {
    setAngleDeg(channel, angle_rad * JointLimits::kRadToDeg, range);
}

void ServoDriver::setPulseUs(uint8_t channel, uint16_t pulse_us) {
    const uint16_t duty = pulseToDutyRaw(pulse_us);
    if (SimMode::isEnabled()) {
        LOG_DEBUG("[ServoDriver] SIM: ch%u pulse=%u us duty=%u", channel, pulse_us, duty);
        return;
    }

    if (!sInitialized) {
        LOG_ERROR("[ServoDriver] not initialized — call begin() first");
        return;
    }

    ledcWrite(channel, duty);
    LOG_DEBUG("[ServoDriver] ch%u pulse=%u us duty=%u", channel, pulse_us, duty);
}

void ServoDriver::freeze() {
    LOG_INFO("[ServoDriver] freeze() — last duty retained by LEDC hardware");
}

void ServoDriver::freezeAll() {
    freeze();
}

void ServoDriver::disableAll() {
    if (SimMode::isEnabled()) {
        LOG_INFO("[ServoDriver] SIM: disableAll()");
        return;
    }

    for (size_t i = 0; i < 4; ++i) {
        ledcWrite(kLedcChannels[i], 0);
    }
}

}  // namespace Motion
