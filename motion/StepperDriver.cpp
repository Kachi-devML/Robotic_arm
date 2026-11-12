#include "StepperDriver.h"

#include <Arduino.h>
#include <math.h>

#include "JointLimits.h"
#include "Logger.h"
#include "Pins.h"
#include "SimMode.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "driver/gpio.h"
#include "esp_attr.h"
#endif

namespace Motion {

namespace {

volatile int32_t sCurrent = 0;
volatile int32_t sTarget = 0;
volatile bool sEnabled = false;
volatile bool sDirHigh = false;
volatile bool sSim = false;
volatile uint32_t sEnableStartedMs = 0;
bool sInitialized = false;
hw_timer_t* sTimer = nullptr;

constexpr uint32_t kStepPulseUs = 2;

void writeEnablePin(bool enable) {
    if (sSim) {
        return;
    }
    digitalWrite(Pins::kNemaEn, enable ? LOW : HIGH);
}

void IRAM_ATTR pulseStepIsr() {
#if defined(ARDUINO_ARCH_ESP32)
    gpio_set_level(static_cast<gpio_num_t>(Pins::kNemaStep), 1);
    ets_delay_us(kStepPulseUs);
    gpio_set_level(static_cast<gpio_num_t>(Pins::kNemaStep), 0);
#else
    digitalWrite(Pins::kNemaStep, HIGH);
    delayMicroseconds(kStepPulseUs);
    digitalWrite(Pins::kNemaStep, LOW);
#endif
}

void IRAM_ATTR setDirIsr(bool high) {
    const bool target = MotionConfig::STEPPER_DIR_INVERT ? !high : high;
    if (sDirHigh == target) {
        return;
    }
    sDirHigh = target;
#if defined(ARDUINO_ARCH_ESP32)
    gpio_set_level(static_cast<gpio_num_t>(Pins::kNemaDir), target ? 1 : 0);
#else
    digitalWrite(Pins::kNemaDir, target ? HIGH : LOW);
#endif
}

void IRAM_ATTR onStepperTick() {
    if (!sEnabled) {
        return;
    }

    const int32_t target = sTarget;
    const int32_t current = sCurrent;
    if (target == current) {
        return;
    }

    const bool forward = target > current;
    if (sSim) {
        sCurrent = current + (forward ? 1 : -1);
        return;
    }

    setDirIsr(forward);
    pulseStepIsr();
    sCurrent = current + (forward ? 1 : -1);
}

void startTimer() {
    if (sTimer != nullptr) {
        return;
    }

    const uint32_t period_us = 1000000UL / MotionConfig::STEPPER_ISR_HZ;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    sTimer = timerBegin(1000000);
    timerAttachInterrupt(sTimer, &onStepperTick);
    timerAlarm(sTimer, period_us, true, 0);
#else
    sTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(sTimer, &onStepperTick, true);
    timerAlarmWrite(sTimer, period_us, true);
    timerAlarmEnable(sTimer);
#endif
}

void stopTimer() {
    if (sTimer == nullptr) {
        return;
    }
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    timerEnd(sTimer);
#else
    timerAlarmDisable(sTimer);
    timerDetachInterrupt(sTimer);
    timerEnd(sTimer);
#endif
    sTimer = nullptr;
}

}  // namespace

void StepperDriver::begin() {
    sCurrent = 0;
    sTarget = 0;
    sEnabled = false;
    sDirHigh = false;
    sSim = SimMode::isEnabled();
    sEnableStartedMs = 0;

    if (!sSim) {
        pinMode(Pins::kNemaStep, OUTPUT);
        pinMode(Pins::kNemaDir, OUTPUT);
        pinMode(Pins::kNemaEn, OUTPUT);
        pinMode(Pins::kNemaM0, OUTPUT);
        pinMode(Pins::kNemaM1, OUTPUT);
        pinMode(Pins::kNemaM2, OUTPUT);
        digitalWrite(Pins::kNemaStep, LOW);
        digitalWrite(Pins::kNemaDir, LOW);
        digitalWrite(Pins::kNemaEn, HIGH);
        setMicrosteps(MotionConfig::STEPPER_MICROSTEP_DIV);
    }

    startTimer();
    sInitialized = true;

    LOG_INFO("[StepperDriver] begin ready=%u us sim=%d dir_invert=%d",
             MotionConfig::STEPPER_ISR_HZ, SimMode::isEnabled() ? 1 : 0,
             MotionConfig::STEPPER_DIR_INVERT ? 1 : 0);
}

void StepperDriver::stop() {
    setEnabled(false);
    stopTimer();
    sInitialized = false;
    LOG_INFO("[StepperDriver] stop()");
}

void StepperDriver::setTargetStep(int32_t step) {
    sTarget = step;
}

int32_t StepperDriver::currentStep() {
    return sCurrent;
}

void StepperDriver::zeroPosition() {
    sCurrent = 0;
    sTarget = 0;
}

void StepperDriver::enable() {
    if (sEnabled) {
        return;
    }
    sEnabled = true;
    sEnableStartedMs = millis();
    writeEnablePin(true);
    LOG_INFO("[StepperDriver] enable() EN low; settle=%lu ms", static_cast<unsigned long>(MotionConfig::STEPPER_ENABLE_SETTLE_MS));
}

void StepperDriver::release() {
    sEnabled = false;
    writeEnablePin(false);
    LOG_INFO("[StepperDriver] release() EN high");
}

bool StepperDriver::isReady() {
    return sEnabled && (millis() - sEnableStartedMs >= MotionConfig::STEPPER_ENABLE_SETTLE_MS);
}

void StepperDriver::setEnabled(bool enabled) {
    if (enabled) {
        StepperDriver::enable();
    } else {
        StepperDriver::release();
    }
}

bool StepperDriver::isEnabled() {
    return sEnabled;
}

void StepperDriver::emergencyStop() {
    sEnabled = false;
    sTarget = sCurrent;
    writeEnablePin(false);
}

bool StepperDriver::atTarget() {
    return sCurrent == sTarget;
}

void StepperDriver::setMicrosteps(uint16_t microsteps) {
    uint8_t m0 = 0, m1 = 0, m2 = 0;
    switch (microsteps) {
        case 1:
            break;
        case 2:
            m0 = 1;
            break;
        case 4:
            m1 = 1;
            break;
        case 8:
            m0 = 1;
            m1 = 1;
            break;
        case 16:
            m2 = 1;
            break;
        case 32:
            m0 = 1;
            m2 = 1;
            break;
        default:
            LOG_WARN("[StepperDriver] unsupported microsteps %u, using 16", microsteps);
            m2 = 1;
            break;
    }

    if (!sSim) {
        digitalWrite(Pins::kNemaM0, m0);
        digitalWrite(Pins::kNemaM1, m1);
        digitalWrite(Pins::kNemaM2, m2);
    }
}

}  // namespace Motion
