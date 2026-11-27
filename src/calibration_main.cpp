#include <Arduino.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "Logger.h"
#include "SafetyMonitor.h"
#include "ServoDriver.h"
#include "StepperDriver.h"

#ifndef CALIBRATION_MODE
#error "calibration_main.cpp must only be built with CALIBRATION_MODE"
#endif

namespace {

constexpr uint32_t kSerialBaud = 115200;
constexpr float kMaxSlewDegPerSec = 60.0f;
constexpr float kRawSafeMinDeg = 20.0f;
constexpr float kRawSafeMaxDeg = 160.0f;
constexpr size_t kCommandCapacity = 96;

Control::SafetyMonitor gSafety;
char gCommand[kCommandCapacity];
size_t gCommandLength = 0;
bool gCommandOverflow = false;
bool gEstopReported = false;
bool gSlewActive = false;
uint32_t gLastSlewMs = 0;
float gCurrentServoDeg[4] = {};
float gTargetServoDeg[4] = {};

constexpr Motion::ServoId kServoIds[] = {
    Motion::ServoId::SHOULDER,
    Motion::ServoId::ELBOW,
    Motion::ServoId::WRIST,
    Motion::ServoId::GRIPPER,
};

const char* servoName(Motion::ServoId id) {
    switch (id) {
        case Motion::ServoId::SHOULDER: return "shoulder";
        case Motion::ServoId::ELBOW: return "elbow";
        case Motion::ServoId::WRIST: return "wrist";
        case Motion::ServoId::GRIPPER: return "gripper";
    }
    return "unknown";
}

bool parseServo(const char* name, Motion::ServoId& id) {
    for (const auto candidate : kServoIds) {
        if (strcmp(name, servoName(candidate)) == 0) {
            id = candidate;
            return true;
        }
    }
    return false;
}

const Motion::ServoConfig::Channel& configFor(Motion::ServoId id) {
    switch (id) {
        case Motion::ServoId::SHOULDER: return Motion::ServoConfig::kShoulder;
        case Motion::ServoId::ELBOW: return Motion::ServoConfig::kElbow;
        case Motion::ServoId::WRIST: return Motion::ServoConfig::kWrist;
        case Motion::ServoId::GRIPPER: return Motion::ServoConfig::kGripper;
    }
    return Motion::ServoConfig::kShoulder;
}

Motion::ServoDriver::PulseRange rangeFor(
    const Motion::ServoConfig::Channel& config) {
    return {config.pulseMinUs, config.pulseMaxUs, config.angleRangeDeg};
}

void hardCut() {
    Motion::StepperDriver::emergencyStop();
}

void printHelp() {
    Serial.println("Commands:");
    Serial.println("  raw <shoulder|elbow|wrist|gripper> <servoDeg> [unsafe]");
    Serial.println("  home");
    Serial.println("  map <shoulder|elbow|wrist|gripper> <jointDeg>");
}

void setTargetRaw(Motion::ServoId id, float servoDeg) {
    gTargetServoDeg[static_cast<uint8_t>(id)] = servoDeg;
    gSlewActive = true;
    gLastSlewMs = millis();
    Serial.printf("raw %s target %.2f deg (slew cap %.0f deg/s)\n",
                  servoName(id), servoDeg, kMaxSlewDegPerSec);
}

void handleCommand(char* line) {
    char* tokens[5] = {};
    size_t count = 0;
    for (char* token = strtok(line, " \t"); token != nullptr;
         token = strtok(nullptr, " \t")) {
        if (count == sizeof(tokens) / sizeof(tokens[0])) {
            Serial.println("ERR: too many arguments");
            return;
        }
        tokens[count++] = token;
    }
    if (count == 0) return;

    if (strcmp(tokens[0], "help") == 0 && count == 1) {
        printHelp();
        return;
    }
    if (strcmp(tokens[0], "home") == 0 && count == 1) {
        for (const auto id : kServoIds) {
            const float logicalHome =
                id == Motion::ServoId::GRIPPER ? 0.0f : 90.0f;
            const auto& config = configFor(id);
            const auto conversion = Motion::ServoDriver::convertAngleDeg(
                logicalHome, config, rangeFor(config));
            if (conversion.servoClamped) {
                LOG_WARN("[calib] %s home mapped angle %.2f clamped to %.2f",
                         servoName(id), conversion.mappedServoAngleDeg,
                         conversion.servoAngleDeg);
            }
            gTargetServoDeg[static_cast<uint8_t>(id)] =
                conversion.servoAngleDeg;
        }
        gSlewActive = true;
        gLastSlewMs = millis();
        Serial.println("home target queued (gripper open; slew capped at 60 deg/s)");
        return;
    }
    if (strcmp(tokens[0], "raw") == 0 && (count == 3 || count == 4)) {
        Motion::ServoId id;
        if (!parseServo(tokens[1], id)) {
            Serial.println("ERR: unknown servo");
            return;
        }
        char* end = nullptr;
        const float servoDeg = strtof(tokens[2], &end);
        const auto& config = configFor(id);
        const bool unsafe = count == 4 && strcmp(tokens[3], "unsafe") == 0;
        const float safeMaxDeg = fminf(kRawSafeMaxDeg, config.angleRangeDeg);
        if (end == tokens[2] || *end != '\0' || !std::isfinite(servoDeg) ||
            (count == 4 && !unsafe)) {
            Serial.println("ERR: expected a finite angle and optional 'unsafe'");
            return;
        }
        if ((!unsafe && (servoDeg < kRawSafeMinDeg ||
                         servoDeg > safeMaxDeg)) ||
            servoDeg < 0.0f || servoDeg > config.angleRangeDeg) {
            Serial.printf("ERR: %s raw angle must be %.0f..%.0f deg%s\n",
                          servoName(id),
                          unsafe ? 0.0f : kRawSafeMinDeg,
                          unsafe ? config.angleRangeDeg : safeMaxDeg,
                          unsafe ? "" : " (append 'unsafe' to bypass the 20..160 restriction)");
            return;
        }
        setTargetRaw(id, servoDeg);
        return;
    }
    if (strcmp(tokens[0], "map") == 0 && count == 3) {
        Motion::ServoId id;
        if (!parseServo(tokens[1], id)) {
            Serial.println("ERR: unknown servo");
            return;
        }
        char* end = nullptr;
        const float jointDeg = strtof(tokens[2], &end);
        if (end == tokens[2] || *end != '\0' || !std::isfinite(jointDeg)) {
            Serial.println("ERR: joint angle must be finite");
            return;
        }
        const auto& config = configFor(id);
        const auto conversion = Motion::ServoDriver::convertAngleDeg(
            jointDeg, config, rangeFor(config));
        if (conversion.jointClamped) {
            LOG_WARN("[calib] %s joint angle clamped to soft limit %.2f",
                     servoName(id), conversion.jointAngleDeg);
        }
        if (conversion.servoClamped) {
            LOG_WARN("[calib] %s mapped angle %.2f clamped to %.2f",
                     servoName(id), conversion.mappedServoAngleDeg,
                     conversion.servoAngleDeg);
        }
        Serial.printf("%s joint=%.2f mappedServo=%.2f clampedServo=%.2f pulse=%u us duty=%u\n",
                      servoName(id), conversion.jointAngleDeg,
                      conversion.mappedServoAngleDeg, conversion.servoAngleDeg,
                      conversion.pulseUs, conversion.duty);
        return;
    }
    Serial.println("ERR: invalid command; type 'help'");
}

void pollSerial() {
    while (Serial.available() > 0) {
        const char value = static_cast<char>(Serial.read());
        if (value == '\r') continue;
        if (value == '\n') {
            if (gCommandOverflow) {
                Serial.println("ERR: command too long");
            } else {
                gCommand[gCommandLength] = '\0';
                handleCommand(gCommand);
            }
            gCommandLength = 0;
            gCommandOverflow = false;
        } else if (gCommandLength + 1 < sizeof(gCommand)) {
            gCommand[gCommandLength++] = value;
        } else {
            gCommandOverflow = true;
        }
    }
}

void updateSlew(uint32_t nowMs) {
    if (!gSlewActive) return;
    const uint32_t elapsedMs = nowMs - gLastSlewMs;
    gLastSlewMs = nowMs;
    if (elapsedMs == 0) return;

    const float maxStep =
        kMaxSlewDegPerSec * static_cast<float>(elapsedMs) / 1000.0f;
    bool stillMoving = false;
    for (const auto id : kServoIds) {
        const uint8_t index = static_cast<uint8_t>(id);
        const float delta = gTargetServoDeg[index] - gCurrentServoDeg[index];
        if (fabsf(delta) <= maxStep) {
            if (delta != 0.0f) {
                gCurrentServoDeg[index] = gTargetServoDeg[index];
                Motion::ServoDriver::writeRawAngleDeg(
                    id, gCurrentServoDeg[index]);
            }
        } else {
            gCurrentServoDeg[index] += delta > 0.0f ? maxStep : -maxStep;
            Motion::ServoDriver::writeRawAngleDeg(id, gCurrentServoDeg[index]);
            stillMoving = true;
        }
    }
    gSlewActive = stillMoving;
}

}  // namespace

void setup() {
    Logger::begin(kSerialBaud);
    gSafety.setHardCut(&hardCut);
    Motion::StepperDriver::begin();
    gSafety.attachHardware();
    Motion::ServoDriver::begin();

    for (const auto id : kServoIds) {
        const float logicalHome =
            id == Motion::ServoId::GRIPPER ? 0.0f : 90.0f;
        const auto& config = configFor(id);
        const auto home = Motion::ServoDriver::convertAngleDeg(
            logicalHome, config, rangeFor(config));
        const uint8_t index = static_cast<uint8_t>(id);
        gCurrentServoDeg[index] = home.servoAngleDeg;
        gTargetServoDeg[index] = home.servoAngleDeg;
    }
    gLastSlewMs = millis();
    Serial.println("Servo calibration mode ready; stepper remains disabled.");
    printHelp();
}

void loop() {
    if (gSafety.isLatched()) {
        if (!gEstopReported) {
            Motion::StepperDriver::emergencyStop();
            Motion::ServoDriver::freezeAll();
            gSlewActive = false;
            Serial.println("ESTOP: outputs frozen; reset required.");
            gEstopReported = true;
        }
        return;
    }
    pollSerial();
    if (gSafety.isLatched()) return;
    updateSlew(millis());
}
