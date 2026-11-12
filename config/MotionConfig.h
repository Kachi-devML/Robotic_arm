#pragma once

#include <stdint.h>

#include "JointLimits.h"

// Per-joint velocity / acceleration / jerk and payload derating.
// Values are conservative bench starting points (50–60% of datasheet no-load
// figures), not raw maxima. Tune on hardware; keep derivations in the comments.

namespace MotionConfig {

constexpr float kSafetyDerate = 0.55f;  // 55% of theoretical no-load max (spec: start 50–60%)

// LOADED profile multiplies EMPTY v/a/jerk by this factor. Operator-declared; no load cell.
constexpr float kLoadedDerate = 0.50f;

constexpr uint32_t kStepperLoopHz = 1000;  // trajectory update cadence
constexpr uint32_t kServoUpdateHz = 50;     // direct LEDC servo updates are 50 Hz and are jitter-free
constexpr uint32_t kStepperIsrHz = 20000;    // dedicated timer-ISR rate for the base step generator

// Revision 2 stepper / gearbox constants.
constexpr float STEPPER_FULL_STEPS_PER_REV = 200.0f;
constexpr float STEPPER_GEAR_RATIO = 5.0f;
constexpr uint8_t STEPPER_MICROSTEP_DIV = 16;
constexpr float STEPPER_STEPS_PER_OUTPUT_DEG =
    (STEPPER_FULL_STEPS_PER_REV * static_cast<float>(STEPPER_MICROSTEP_DIV) * STEPPER_GEAR_RATIO) / 360.0f;
constexpr uint32_t STEPPER_ISR_HZ = 20000;
constexpr float BASE_MAX_VEL_OUT_DPS = 90.0f;

// Keep the older naming that the rest of the project uses, but derive it from the
// output-shaft base speed limit so the planner matches the gearbox.
constexpr uint16_t kBaseStepsPerRevMotor = static_cast<uint16_t>(STEPPER_FULL_STEPS_PER_REV);
constexpr uint16_t kMicrosteps = STEPPER_MICROSTEP_DIV;
constexpr uint16_t kBaseStepsPerRev = static_cast<uint16_t>(
    STEPPER_FULL_STEPS_PER_REV * static_cast<float>(STEPPER_MICROSTEP_DIV));
static_assert(BASE_MAX_VEL_OUT_DPS * STEPPER_STEPS_PER_OUTPUT_DEG * 4.0f <= STEPPER_ISR_HZ,
              "Step ISR too slow for base max velocity at this microstep/gear ratio");

// Homing (spec §4). Fixed slow seek, then a slower re-approach. Bench starting points.
constexpr float kHomingSeekDegPerSec = 30.0f;
constexpr float kHomingCreepDegPerSec = 8.0f;
constexpr float kHomingBackoffDeg = 5.0f;
constexpr float kHomingCreepExtraDeg = 3.0f;
constexpr float kHomingMaxTravelDeg = 380.0f;
constexpr uint32_t kHomingTimeoutMs = 25000;
constexpr uint32_t kHomingServoSettleMs = 300;

// Idle-release policy from §6.5 for the DRV8825.
constexpr bool STEPPER_IDLE_RELEASE_ENABLED = true;
constexpr uint32_t STEPPER_IDLE_RELEASE_DELAY_MS = 500;
constexpr uint32_t STEPPER_ENABLE_SETTLE_MS = 5;
constexpr bool STEPPER_DIR_INVERT = false;

// Link supervision (spec §5.2): 1 s heartbeat, 2 missed = disconnected.
constexpr uint32_t kHeartbeatIntervalMs = 1000;
constexpr uint32_t kHeartbeatMisses = 2;
constexpr uint32_t kLinkTimeoutMs = kHeartbeatIntervalMs * kHeartbeatMisses;

// Control-loop watchdog. Trips if the main loop stops kicking, not if a move is long.
constexpr uint32_t kWatchdogTimeoutMs = 500;

inline constexpr float deratedDegPerSec(float datasheetDegPerSec) {
  return datasheetDegPerSec * kSafetyDerate;
}

// --- θ0 NEMA17 + DRV8825 + 5:1 gearbox ---
constexpr float kTheta0MaxVel_radps = JointLimits::degToRad(BASE_MAX_VEL_OUT_DPS);
constexpr float kTheta0MaxAcc_radps2 = JointLimits::degToRad(180.0f);
constexpr float kTheta0MaxJerk_radps3 = JointLimits::degToRad(720.0f);

// --- θ1 YSIDO 35 kg·cm shoulder servo ---
constexpr float kTheta1MaxVel_radps = JointLimits::degToRad(deratedDegPerSec(60.0f / 0.13f));
constexpr float kTheta1MaxAcc_radps2 = JointLimits::degToRad(800.0f);
constexpr float kTheta1MaxJerk_radps3 = JointLimits::degToRad(3200.0f);

// --- θ2 RCXAZ 14.5 kg·cm elbow servo ---
constexpr float kTheta2MaxVel_radps = JointLimits::degToRad(deratedDegPerSec(60.0f / 0.14f));
constexpr float kTheta2MaxAcc_radps2 = JointLimits::degToRad(700.0f);
constexpr float kTheta2MaxJerk_radps3 = JointLimits::degToRad(2800.0f);

// --- θ3 MG90S wrist ---
constexpr float kTheta3MaxVel_radps = JointLimits::degToRad(deratedDegPerSec(60.0f / 0.10f));
constexpr float kTheta3MaxAcc_radps2 = JointLimits::degToRad(1200.0f);
constexpr float kTheta3MaxJerk_radps3 = JointLimits::degToRad(4800.0f);

// --- gripper SG90 ---
constexpr float kGripperMaxVel_radps = JointLimits::degToRad(deratedDegPerSec(60.0f / 0.10f));
constexpr float kGripperMaxAcc_radps2 = JointLimits::degToRad(800.0f);
constexpr float kGripperMaxJerk_radps3 = JointLimits::degToRad(3200.0f);

}  // namespace MotionConfig
