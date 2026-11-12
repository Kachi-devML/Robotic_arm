#include "SafetyMonitor.h"

#include "MotionConfig.h"
#include "Pins.h"
#include "SimMode.h"

#if defined(ARDUINO)
#include <Arduino.h>
#endif

namespace Control {

SafetyMonitor* SafetyMonitor::isrInstance_ = nullptr;

void SafetyMonitor::isrThunk() {
    if (isrInstance_ != nullptr) {
        isrInstance_->onEstopEdge();
    }
}

void SafetyMonitor::attachHardware() {
#if defined(ARDUINO)
    if (SimMode::isEnabled()) {
        // Dry-run: do not sample a floating or unwired e-stop pin.
        return;
    }
    isrInstance_ = this;
    // External pull-up only. Enabling the internal pull would hide an open wire.
    pinMode(Pins::kEstopSw, INPUT);
    attachInterrupt(digitalPinToInterrupt(Pins::kEstopSw), &SafetyMonitor::isrThunk, RISING);
    // Already open at boot is a trigger, not only a later edge.
    if (digitalRead(Pins::kEstopSw) == HIGH) {
        onEstopEdge();
    }
#else
    (void)0;
#endif
}

void SafetyMonitor::onEstopEdge() {
    latched_ = true;
    if (hardCut_ != nullptr) {
        hardCut_();
    }
}

void SafetyMonitor::releaseLatch() {
    latched_ = false;
}

void SafetyMonitor::startLinkSupervision(uint32_t now_ms) {
    linkSupervised_ = true;
    linkDown_ = false;
    lastLinkMs_ = now_ms;
}

void SafetyMonitor::noteLinkActivity(uint32_t now_ms) {
    lastLinkMs_ = now_ms;
    linkDown_ = false;
}

bool SafetyMonitor::pollLink(uint32_t now_ms) {
    if (!linkSupervised_) {
        return false;
    }
    const bool down = (now_ms - lastLinkMs_) >= MotionConfig::kLinkTimeoutMs;
    const bool edge = down && !linkDown_;
    linkDown_ = down;
    return edge;
}

void SafetyMonitor::kick(uint32_t now_ms) {
    lastKickMs_ = now_ms;
    watchdogArmed_ = true;
    watchdogDown_ = false;
}

bool SafetyMonitor::pollWatchdog(uint32_t now_ms) {
    if (!watchdogArmed_) {
        return false;
    }
    const bool down = (now_ms - lastKickMs_) >= MotionConfig::kWatchdogTimeoutMs;
    const bool edge = down && !watchdogDown_;
    watchdogDown_ = down;
    return edge;
}

}  // namespace Control
