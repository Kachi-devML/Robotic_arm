#pragma once

#include "SCurveProfile.h"

#include <stdint.h>

// Synchronized multi-joint trajectory planner (AGENT_BUILD_SPEC §6.2).
//
// Pure math — NO hardware dependencies (no GPIO, I2C, FreeRTOS, Arduino).
// Unit-testable on desktop/native.
//
// For a move:
//   1. Compute each joint's time-optimal S-curve duration under its own
//      v/a/jerk limits (after payload-profile derating).
//   2. Share one duration across all joints.
//   3. Drive a single progress variable s ∈ [0, 1] from a unit-distance
//      S-curve, and map s onto each joint's signed angle range:
//          θ_i(t) = θ_i_start + s(t) · (θ_i_end − θ_i_start)
//
// That guarantees every joint starts and stops together.  The unit S-curve
// uses the tightest normalized limits min_i(v_i/|Δθ_i|) (and the same for
// accel and jerk) so a shared shape cannot push any joint past its envelope.
// Shared duration is therefore ≥ max(per-joint time-optimal), and equal
// when one joint is tightest in velocity, acceleration, and jerk.

namespace Motion {

enum class PayloadProfile : uint8_t {
    Empty = 0,
    Loaded = 1,
};

static constexpr int kJointCount = 5;

enum class JointId : uint8_t {
    Theta0 = 0,
    Theta1 = 1,
    Theta2 = 2,
    Theta3 = 3,
    Gripper = 4,
};

/// Absolute joint angles (radians). θ0–θ3 are positioning joints; gripper
/// is jaw angle (0 = open).  Included so a move can finish gripper motion
/// on the same clock as the arm.
struct JointVector {
    float theta0;
    float theta1;
    float theta2;
    float theta3;
    float gripper;
};

inline float jointAt(const JointVector& v, int i) {
    switch (i) {
        case 0:
            return v.theta0;
        case 1:
            return v.theta1;
        case 2:
            return v.theta2;
        case 3:
            return v.theta3;
        default:
            return v.gripper;
    }
}

inline void setJoint(JointVector& v, int i, float value) {
    switch (i) {
        case 0:
            v.theta0 = value;
            break;
        case 1:
            v.theta1 = value;
            break;
        case 2:
            v.theta2 = value;
            break;
        case 3:
            v.theta3 = value;
            break;
        default:
            v.gripper = value;
            break;
    }
}

struct JointPlan {
    float start;          // rad
    float end;            // rad
    float delta;          // end − start (signed, rad)
    float distance;       // |delta| (rad)
    float time_optimal;   // s — duration if this joint moved alone
    float v_max;          // rad/s — derated limit used for this plan
    float a_max;          // rad/s²
    float j_max;          // rad/s³
};

struct TrajectoryPlan {
    PayloadProfile profile;
    JointPlan joints[kJointCount];

    /// Unit-distance S-curve whose position is s(t) ∈ [0, 1].
    SCurveProfile shape;

    /// Shared move duration (seconds).  0 if every joint is already there.
    float duration;

    /// Index of the joint with the largest time-optimal duration
    /// (0 if the move is empty).
    int limiting_joint;

    /// s(t) clamped to [0, 1].  t < 0 → 0; t ≥ duration → 1.
    /// Empty moves return 1 (already at the target).
    float progress(float t) const;

    JointVector anglesAt(float t) const;
    JointVector velocitiesAt(float t) const;
    JointVector accelerationsAt(float t) const;
};

struct TrajectoryPlanner {
    /// Build a synchronized plan from start → end under the given payload
    /// profile.  EMPTY uses MotionConfig limits as-is; LOADED multiplies
    /// v/a/jerk by MotionConfig::kLoadedDerate.
    static TrajectoryPlan plan(const JointVector& start,
                               const JointVector& end,
                               PayloadProfile profile);
};

}  // namespace Motion
