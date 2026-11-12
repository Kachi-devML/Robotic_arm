#pragma once

#include <stdint.h>

// E-stop latch, link timeout, and control-loop watchdog (AGENT_BUILD_SPEC §5.1–§5.2).
//
// The e-stop line is normally-closed to GND with an external pull-up:
//   LOW  = safe / closed circuit
//   HIGH = triggered (button pressed or wire open)
// attachHardware() arms a rising-edge interrupt. The ISR only latches and runs
// the hard-cut callback (disable the stepper). It does not clear itself.
//
// There is no relay cutting motor power. Releasing the switch must not resume
// motion; StateMachine::requestClearEstop plus a successful re-home does that.
// Call releaseLatch() only after that re-home has been accepted and the line
// reads safe. A later rising edge latches again.
//
// Link supervision starts when networking calls startLinkSupervision(). Until
// then the arm can boot and home with no WebSocket. Timeout is two missed
// 1 s heartbeats (MotionConfig::kLinkTimeoutMs).
//
// The watchdog watches the control loop, not trajectory length. Kick it every
// loop. The callback passed to onEstopEdge must be safe to run from an ISR
// (no Serial, no I2C).

namespace Control {

class SafetyMonitor {
 public:
    using HardCutFn = void (*)();

    void setHardCut(HardCutFn cut) { hardCut_ = cut; }

    /// ESP32: INPUT (external pull-up only), RISING interrupt, and an immediate
    /// latch if the pin is already HIGH. No-op on the native test build and in
    /// compile-time sim mode.
    void attachHardware();

    /// ISR entry and the test entry. Latches and calls the hard-cut once per
    /// rising edge while already latched still calls the hard-cut again.
    void onEstopEdge();

    bool isLatched() const { return latched_; }

    /// Drops the software latch. Does not enable motors.
    void releaseLatch();

    void startLinkSupervision(uint32_t now_ms);
    void noteLinkActivity(uint32_t now_ms);
    bool linkSupervised() const { return linkSupervised_; }

    /// True only on the tick that the link newly times out.
    bool pollLink(uint32_t now_ms);

    void kick(uint32_t now_ms);

    /// True only on the tick that the watchdog newly expires.
    bool pollWatchdog(uint32_t now_ms);

 private:
    static void isrThunk();

    HardCutFn hardCut_ = nullptr;
    volatile bool latched_ = false;
    bool linkSupervised_ = false;
    bool linkDown_ = false;
    uint32_t lastLinkMs_ = 0;
    bool watchdogArmed_ = false;
    bool watchdogDown_ = false;
    uint32_t lastKickMs_ = 0;

    static SafetyMonitor* isrInstance_;
};

}  // namespace Control
