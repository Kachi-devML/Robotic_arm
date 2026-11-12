#include "HomingSequence.h"

#include "MotionConfig.h"

#include <cmath>

namespace Control {

void HomingSequence::start(uint32_t now_ms) {
    stepsPerDeg_ = MotionConfig::STEPPER_STEPS_PER_OUTPUT_DEG;
    backoffSteps_ = static_cast<int32_t>(
        std::lround(MotionConfig::kHomingBackoffDeg * stepsPerDeg_));
    maxTravelSteps_ = static_cast<int32_t>(
        std::lround(MotionConfig::kHomingMaxTravelDeg * stepsPerDeg_));
    const int32_t extra = static_cast<int32_t>(
        std::lround(MotionConfig::kHomingCreepExtraDeg * stepsPerDeg_));
    creepMaxSteps_ = backoffSteps_ + extra;
    if (backoffSteps_ < 1) {
        backoffSteps_ = 1;
    }
    if (maxTravelSteps_ < 1) {
        maxTravelSteps_ = 1;
    }

    failReason_ = nullptr;
    snapped_ = false;
    accum_ = 0.0f;
    commanded_ = 0;
    contact_ = 0;
    enter(Phase::Snap, now_ms, 0);
    startMs_ = now_ms;
}

void HomingSequence::abort() {
    if (phase_ == Phase::Done || phase_ == Phase::Failed || phase_ == Phase::Idle) {
        return;
    }
    phase_ = Phase::Aborted;
    failReason_ = "aborted";
}

void HomingSequence::enter(Phase phase, uint32_t now_ms, int32_t currentStep) {
    phase_ = phase;
    phaseMs_ = now_ms;
    lastMs_ = now_ms;
    accum_ = 0.0f;
    commanded_ = currentStep;
    if (phase == Phase::Seek) {
        seekOrigin_ = currentStep;
    }
    if (phase == Phase::Creep) {
        creepOrigin_ = currentStep;
    }
}

void HomingSequence::fail(const char* reason) {
    phase_ = Phase::Failed;
    failReason_ = reason;
}

int32_t HomingSequence::integrate(uint32_t now_ms, float degPerSec, int sign) {
    const uint32_t dt = now_ms - lastMs_;
    lastMs_ = now_ms;
    if (dt == 0 || degPerSec <= 0.0f || sign == 0) {
        return 0;
    }
    accum_ += static_cast<float>(sign) * degPerSec * stepsPerDeg_ *
              (static_cast<float>(dt) / 1000.0f);
    const auto whole = static_cast<int32_t>(accum_);
    accum_ -= static_cast<float>(whole);
    return whole;
}

HomingSequence::Status HomingSequence::make(int32_t currentStep, bool enable, bool snap,
                                            bool zero) const {
    Status out{};
    out.phase = phase_;
    out.targetStep = enable ? commanded_ : currentStep;
    out.enableStepper = enable;
    out.snapServos = snap;
    out.zeroNow = zero;
    out.failReason = failReason_;
    return out;
}

HomingSequence::Status HomingSequence::tick(uint32_t now_ms, bool homeTriggered,
                                            int32_t currentStep) {
    if (phase_ == Phase::Idle || phase_ == Phase::Done || phase_ == Phase::Failed ||
        phase_ == Phase::Aborted) {
        return make(currentStep, false, false, false);
    }

    if (now_ms - startMs_ >= MotionConfig::kHomingTimeoutMs) {
        fail("timeout");
        return make(currentStep, false, false, false);
    }

    switch (phase_) {
        case Phase::Snap: {
            const bool snapNow = !snapped_;
            snapped_ = true;
            commanded_ = currentStep;
            if (now_ms - phaseMs_ >= MotionConfig::kHomingServoSettleMs) {
                if (homeTriggered) {
                    contact_ = currentStep;
                    enter(Phase::BackOff, now_ms, currentStep);
                } else {
                    enter(Phase::Seek, now_ms, currentStep);
                }
            }
            return make(currentStep, false, snapNow, false);
        }
        case Phase::Seek: {
            commanded_ += integrate(now_ms, MotionConfig::kHomingSeekDegPerSec, -1);
            const int32_t limit = seekOrigin_ - maxTravelSteps_;
            if (commanded_ < limit) {
                commanded_ = limit;
            }
            if (homeTriggered) {
                contact_ = currentStep;
                enter(Phase::BackOff, now_ms, currentStep);
                return make(currentStep, true, false, false);
            }
            if (seekOrigin_ - currentStep >= maxTravelSteps_) {
                fail("max rotation");
                return make(currentStep, false, false, false);
            }
            return make(currentStep, true, false, false);
        }
        case Phase::BackOff: {
            // Travel far enough for a clean re-approach. If the switch is still
            // closed after twice that distance, it is stuck or the wiring is wrong.
            const int32_t openTarget = contact_ + backoffSteps_;
            const int32_t giveUp = contact_ + (backoffSteps_ * 2);
            commanded_ += integrate(now_ms, MotionConfig::kHomingSeekDegPerSec, +1);
            const int32_t cap = homeTriggered ? giveUp : openTarget;
            if (commanded_ > cap) {
                commanded_ = cap;
            }
            if (currentStep >= giveUp && homeTriggered) {
                fail("switch stuck");
                return make(currentStep, false, false, false);
            }
            if (currentStep >= openTarget && !homeTriggered) {
                enter(Phase::Creep, now_ms, currentStep);
                return make(currentStep, true, false, false);
            }
            return make(currentStep, true, false, false);
        }
        case Phase::Creep: {
            commanded_ += integrate(now_ms, MotionConfig::kHomingCreepDegPerSec, -1);
            const int32_t limit = creepOrigin_ - creepMaxSteps_;
            if (commanded_ < limit) {
                commanded_ = limit;
            }
            if (homeTriggered) {
                phase_ = Phase::Done;
                commanded_ = currentStep;
                return make(currentStep, false, false, true);
            }
            if (creepOrigin_ - currentStep >= creepMaxSteps_) {
                fail("creep missed switch");
                return make(currentStep, false, false, false);
            }
            return make(currentStep, true, false, false);
        }
        default:
            break;
    }

    return make(currentStep, false, false, false);
}

}  // namespace Control
