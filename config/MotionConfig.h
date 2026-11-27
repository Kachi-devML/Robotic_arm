#pragma once

#include <stdint.h>

#include "JointLimits.h"

// Per-actuator limits use radians, radians/s, radians/s^2, and radians/s^3
// where the suffix states the unit. Datasheet inputs and base limits are
// stated in degrees and degrees/s at their respective motor/output shafts.
namespace MotionConfig {

constexpr float kSafetyDerate = 0.55f;
constexpr float kLoadedDerate = 0.50f;

constexpr uint32_t kStepperLoopHz = 1000;
constexpr uint32_t kServoUpdateHz = 50;

// PROPOSED — NOT SPEC-MANDATED; requires user approval:
// derive acceleration as derated max velocity / 0.5 s, and jerk as max
// acceleration / 0.25 s. Assumes these ramp times are conservative bench
// starting points; no acceleration or jerk datasheet figures were available.
constexpr float kAssumedAccelerationRampSeconds = 0.5f;
constexpr float kAssumedJerkRampSeconds = 0.25f;

constexpr float accelerationLimitDegPerSec2(float maxVelocityDegPerSec) {
  return maxVelocityDegPerSec / kAssumedAccelerationRampSeconds;
}

constexpr float jerkLimitDegPerSec3(float maxAccelerationDegPerSec2) {
  return maxAccelerationDegPerSec2 / kAssumedJerkRampSeconds;
}

// Stepper gearing and microstepping (§6.4). Angles and velocities are at the
// gearbox output. Verify the assumed motor figures and measured gear ratio on
// the bench before relying on these limits.
constexpr float STEPPER_FULL_STEPS_PER_REV = 200.0f;
constexpr float STEPPER_GEAR_RATIO = 5.0f;
constexpr uint8_t STEPPER_MICROSTEP_DIV = 16;
constexpr float STEPPER_STEPS_PER_OUTPUT_DEG =
    (STEPPER_FULL_STEPS_PER_REV * STEPPER_MICROSTEP_DIV * STEPPER_GEAR_RATIO) / 360.0f;
constexpr uint32_t STEPPER_ISR_HZ = 20000;

constexpr bool STEPPER_DIR_INVERT = false;
constexpr bool STEPPER_IDLE_RELEASE_ENABLED = true;
constexpr uint32_t STEPPER_IDLE_RELEASE_DELAY_MS = 500;
constexpr uint32_t STEPPER_ENABLE_SETTLE_MS = 5;

// NEMA17 datasheet basis: no-load speed UNVERIFIED: assumed 136.36 rpm
// (818.18 deg/s at the motor shaft); holding torque UNVERIFIED: assumed
// 0.40 N*m for an unspecified motor. Bench-tuning starting point; exact
// motor model and datasheet are unavailable.
constexpr float kStepperMotorNoLoadSpeedDps = 818.18f;
constexpr float STEPPER_MOTOR_TORQUE_NM = 0.40f;
// Assumption from §6.1: gearbox efficiency is approximately 0.9.
constexpr float STEPPER_GEARBOX_EFFICIENCY = 0.9f;
constexpr float STEPPER_OUTPUT_TORQUE_NM =
    STEPPER_MOTOR_TORQUE_NM * STEPPER_GEAR_RATIO * STEPPER_GEARBOX_EFFICIENCY;

constexpr float BASE_MAX_VEL_OUT_DPS =
    (kStepperMotorNoLoadSpeedDps * kSafetyDerate) / STEPPER_GEAR_RATIO;
static_assert(BASE_MAX_VEL_OUT_DPS * STEPPER_STEPS_PER_OUTPUT_DEG * 4.0f <= STEPPER_ISR_HZ,
              "Step ISR too slow for base max velocity at this microstep/gear ratio");

// Link supervision (§5.2): 1 s heartbeat, 2 missed = disconnected.
constexpr uint32_t kHeartbeatIntervalMs = 1000;
constexpr uint32_t kHeartbeatMisses = 2;
constexpr uint32_t kLinkTimeoutMs = kHeartbeatIntervalMs * kHeartbeatMisses;

// Control-loop watchdog. Trips if the main loop stops kicking, not if a move is long.
constexpr uint32_t kWatchdogTimeoutMs = 500;

inline constexpr float deratedDegPerSec(float theoreticalDegPerSec) {
  return theoreticalDegPerSec * kSafetyDerate;
}

// NEMA17 limits are stored in gearbox-output units. Velocity is the
// derated motor-side no-load speed divided by STEPPER_GEAR_RATIO. The
// acceleration/jerk rule above is proposed for approval, not specified by §6.1.
constexpr float kTheta0MaxVel_radps = JointLimits::degToRad(BASE_MAX_VEL_OUT_DPS);
constexpr float kTheta0MaxAcc_radps2 = JointLimits::degToRad(
    accelerationLimitDegPerSec2(BASE_MAX_VEL_OUT_DPS));
constexpr float kTheta0MaxJerk_radps3 = JointLimits::degToRad(
    jerkLimitDegPerSec3(accelerationLimitDegPerSec2(BASE_MAX_VEL_OUT_DPS)));

// YSIDO 35KG datasheet basis: stall torque UNVERIFIED: assumed 35 kgf*cm;
// speed UNVERIFIED: assumed 60 deg/0.13 s (461.54 deg/s). Bench-tuning
// starting point; exact servo variant and datasheet are unavailable.
constexpr float kTheta1MaxVelDegPerSec = deratedDegPerSec(60.0f / 0.13f);
constexpr float kTheta1MaxAccDegPerSec2 =
    accelerationLimitDegPerSec2(kTheta1MaxVelDegPerSec);
constexpr float kTheta1MaxJerkDegPerSec3 =
    jerkLimitDegPerSec3(kTheta1MaxAccDegPerSec2);
constexpr float kTheta1MaxVel_radps = JointLimits::degToRad(kTheta1MaxVelDegPerSec);
constexpr float kTheta1MaxAcc_radps2 = JointLimits::degToRad(kTheta1MaxAccDegPerSec2);
constexpr float kTheta1MaxJerk_radps3 = JointLimits::degToRad(kTheta1MaxJerkDegPerSec3);

// RCXAZ 14.5KG datasheet basis: stall torque UNVERIFIED: assumed
// 14.5 kgf*cm; speed UNVERIFIED: assumed 60 deg/0.14 s (428.57 deg/s).
// Bench-tuning starting point; exact servo variant and datasheet are unavailable.
constexpr float kTheta2MaxVelDegPerSec = deratedDegPerSec(60.0f / 0.14f);
constexpr float kTheta2MaxAccDegPerSec2 =
    accelerationLimitDegPerSec2(kTheta2MaxVelDegPerSec);
constexpr float kTheta2MaxJerkDegPerSec3 =
    jerkLimitDegPerSec3(kTheta2MaxAccDegPerSec2);
constexpr float kTheta2MaxVel_radps = JointLimits::degToRad(kTheta2MaxVelDegPerSec);
constexpr float kTheta2MaxAcc_radps2 = JointLimits::degToRad(kTheta2MaxAccDegPerSec2);
constexpr float kTheta2MaxJerk_radps3 = JointLimits::degToRad(kTheta2MaxJerkDegPerSec3);

// MG90s datasheet basis: stall torque UNVERIFIED: assumed 1.8 kgf*cm;
// speed UNVERIFIED: assumed 60 deg/0.10 s (600 deg/s). Bench-tuning
// starting point; exact servo variant and datasheet are unavailable.
constexpr float kTheta3MaxVelDegPerSec = deratedDegPerSec(60.0f / 0.10f);
constexpr float kTheta3MaxAccDegPerSec2 =
    accelerationLimitDegPerSec2(kTheta3MaxVelDegPerSec);
constexpr float kTheta3MaxJerkDegPerSec3 =
    jerkLimitDegPerSec3(kTheta3MaxAccDegPerSec2);
constexpr float kTheta3MaxVel_radps = JointLimits::degToRad(kTheta3MaxVelDegPerSec);
constexpr float kTheta3MaxAcc_radps2 = JointLimits::degToRad(kTheta3MaxAccDegPerSec2);
constexpr float kTheta3MaxJerk_radps3 = JointLimits::degToRad(kTheta3MaxJerkDegPerSec3);

// SG90 datasheet basis: stall torque UNVERIFIED: assumed 1.8 kgf*cm;
// speed UNVERIFIED: assumed 60 deg/0.10 s (600 deg/s). Bench-tuning
// starting point; exact servo variant and datasheet are unavailable.
constexpr float kGripperMaxVelDegPerSec = deratedDegPerSec(60.0f / 0.10f);
constexpr float kGripperMaxAccDegPerSec2 =
    accelerationLimitDegPerSec2(kGripperMaxVelDegPerSec);
constexpr float kGripperMaxJerkDegPerSec3 =
    jerkLimitDegPerSec3(kGripperMaxAccDegPerSec2);
constexpr float kGripperMaxVel_radps = JointLimits::degToRad(kGripperMaxVelDegPerSec);
constexpr float kGripperMaxAcc_radps2 = JointLimits::degToRad(kGripperMaxAccDegPerSec2);
constexpr float kGripperMaxJerk_radps3 = JointLimits::degToRad(kGripperMaxJerkDegPerSec3);

}  // namespace MotionConfig
