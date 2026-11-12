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

static float clampf(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

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

}  // namespace

bool ServoDriver::begin() {
    if (SimMode::isEnabled()) {
        LOG_INFO("[ServoDriver] SIM: begin() — direct LEDC output setup skipped");
        sInitialized = true;
        return true;
    }

    for (size_t i = 0; i < 4; ++i) {
        ledcSetup(kLedcChannels[i], ServoConfig::kPwmFrequencyHz, ServoConfig::kPwmResolutionBits);
        ledcAttachPin(kLedcPins[i], kLedcChannels[i]);
        ledcWrite(kLedcChannels[i], pulseToDutyRaw(1500));
    }

    sInitialized = true;
    LOG_INFO("[ServoDriver] direct LEDC attached on GPIO4-7, 50 Hz, 14-bit resolution");
    return true;
}

void ServoDriver::writeAngleDeg(ServoId id, float angle_deg) {
    const auto& cfg = configForId(id);
    writeAngleDeg(static_cast<uint8_t>(id), angle_deg, {cfg.pulseMinUs, cfg.pulseMaxUs, cfg.angleRangeDeg});
}

void ServoDriver::writeAngleDeg(uint8_t channel, float angle_deg,
                               const PulseRange& range) {
    const float clamped = clampf(angle_deg, 0.0f, range.max_deg);
    const float frac = (range.max_deg > 0.0f) ? (clamped / range.max_deg) : 0.0f;
    const uint16_t pulse_us = static_cast<uint16_t>(range.min_us + frac * (range.max_us - range.min_us) + 0.5f);
    setPulseUs(channel, pulse_us);
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
    if (SimMode::isEnabled()) {
        LOG_DEBUG("[ServoDriver] SIM: ch%u → %u µs", channel, pulse_us);
        return;
    }

    if (!sInitialized) {
        LOG_ERROR("[ServoDriver] not initialized — call begin() first");
        return;
    }

    const uint16_t duty = pulseToDutyRaw(pulse_us);
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
