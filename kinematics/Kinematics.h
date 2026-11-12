#pragma once

#include <stdint.h>

// Pure kinematic functions for a 4-DoF robotic arm (base, shoulder, elbow, wrist).
//
// NO hardware dependencies — no GPIO, no I2C, no FreeRTOS, no Arduino.
// Inputs in, structured result out.  Unit-testable on desktop/native.
//
// All internal angle math is in RADIANS.  Degrees exist only at the
// driver/JSON boundary (handled by other modules, not here).
//
// Joint naming: θ0 base, θ1 shoulder, θ2 elbow, θ3 wrist.

namespace Kinematics {

// ---------- Data types ----------

/// Target position and approach angle fed into the IK solver.
struct IKInput {
    float x;    // cm — fingertip target x
    float y;    // cm — fingertip target y
    float z;    // cm — fingertip target z
    float phi;  // rad — approach/wrist angle (0 = horizontal, negative = angled down)
};

/// Error codes for individual posture solutions.
enum class ErrorCode : uint8_t {
    NONE = 0,
    TARGET_UNREACHABLE,
    JOINT_LIMIT_EXCEEDED,
};

/// One candidate IK posture (elbow-up or elbow-down).
struct PostureSolution {
    float theta0;  // rad — base twist
    float theta1;  // rad — shoulder
    float theta2;  // rad — elbow
    float theta3;  // rad — wrist
    bool valid;
    ErrorCode error;
};

/// One step in the IK derivation, for the educational panel.
/// `label` and `expression` point to string literals (static storage).
struct DerivationStep {
    const char* label;       // e.g. "base_twist"
    const char* expression;  // e.g. "θ0 = atan2(y, x)"
    float value;             // computed numeric result
};

static constexpr uint8_t kMaxDerivationSteps = 16;

/// Full IK result: both postures, derivation steps, reach info.
struct IKResult {
    IKInput input;

    PostureSolution elbow_up;
    PostureSolution elbow_down;

    DerivationStep steps[kMaxDerivationSteps];
    uint8_t step_count;

    // Reach info — always populated, even when unreachable,
    // so the UI can display max/min reach on error.
    float dw;       // actual wrist-center distance (cm)
    float dw_min;   // |L1 − L2| minimum reachable wrist distance (cm)
    float dw_max;   // L1 + L2 maximum reachable wrist distance (cm)
    bool reachable;  // true if dw_min ≤ Dw ≤ dw_max
};

/// 2D position in the arm plane (post-base-rotation side view).
struct Vec2 {
    float r;  // cm — radial distance from base vertical axis
    float z;  // cm — height
};

/// 3D Cartesian position.
struct Vec3 {
    float x;  // cm
    float y;  // cm
    float z;  // cm
};

/// Forward-kinematics result: every joint position, for visualization.
struct FKResult {
    // Arm-plane (r, z) positions — for the 2D side-view canvas visualizer.
    Vec2 base_2d;       // always (0, 0)
    Vec2 shoulder_2d;   // always (0, d1)
    Vec2 elbow_2d;
    Vec2 wrist_2d;
    Vec2 effector_2d;

    // 3D Cartesian positions (after θ0 base rotation).
    Vec3 base_3d;
    Vec3 shoulder_3d;
    Vec3 elbow_3d;
    Vec3 wrist_3d;
    Vec3 effector_3d;
};

// ---------- Public API ----------

/// Solve inverse kinematics for a target fingertip position.
///
/// Always computes BOTH elbow-up and elbow-down solutions with validity
/// flags and error codes (per spec §3.4).  Returns step-by-step derivation
/// data for the educational panel.
///
/// @param input  Target position (x,y,z in cm) and approach angle φ (rad).
/// @return       Full IK result with both postures and reach info.
IKResult solveIK(const IKInput& input);

/// Solve forward kinematics: given joint angles, compute all joint positions.
///
/// Used for the 2D canvas visualizer and for round-trip IK verification.
///
/// @param theta0  Base twist (rad).
/// @param theta1  Shoulder angle (rad).
/// @param theta2  Elbow angle (rad).
/// @param theta3  Wrist angle (rad).
/// @return        Joint positions in 2D arm plane and 3D Cartesian.
FKResult solveFK(float theta0, float theta1, float theta2, float theta3);

/// Check whether a set of joint angles falls within the soft limits
/// defined in JointLimits.h.
///
/// @return true if ALL four joints are within their respective soft limits.
bool checkJointLimits(float theta0, float theta1, float theta2, float theta3);

}  // namespace Kinematics
