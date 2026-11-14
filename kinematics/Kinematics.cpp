#include "Kinematics.h"

#include "JointLimits.h"

#include <cmath>
#include <limits>

namespace Kinematics {

// ---------- Internal helpers ----------

namespace {

/// Append a derivation step to the result.  No-op if the array is full.
void addStep(IKResult& result,
             const char* label,
             const char* expression,
             float value) {
    if (result.step_count < kMaxDerivationSteps) {
        result.steps[result.step_count] = {label, expression, value};
        ++result.step_count;
    }
}

bool withinSoftLimitTolerance(float angle, float min, float max, float tolerance) {
    return angle >= min - tolerance && angle <= max + tolerance;
}

float clampToSoftLimit(float angle, float min, float max) {
    return std::fmax(min, std::fmin(max, angle));
}

float wrapRadians(float angle) {
    const float twoPi = 2.0f * JointLimits::kPi;
    float wrapped = std::fmod(angle, twoPi);
    if (wrapped <= -JointLimits::kPi) {
        wrapped += twoPi;
    } else if (wrapped > JointLimits::kPi) {
        wrapped -= twoPi;
    }
    return wrapped;
}

/// Post-check: validate a single posture's angles against JointLimits.
/// Marks the posture invalid with JOINT_LIMIT_EXCEEDED if any angle is
/// outside its soft range.
void validatePosture(PostureSolution& posture) {
    if (!posture.valid) return;  // already invalid (e.g. unreachable)

    const float tolerance =
        JointLimits::degToRad(JointLimits::kSoftLimitTolerance_deg);
    if (!withinSoftLimitTolerance(posture.theta0,
                                  JointLimits::kTheta0SoftMin_rad,
                                  JointLimits::kTheta0SoftMax_rad,
                                  tolerance) ||
        !withinSoftLimitTolerance(posture.theta1,
                                  JointLimits::kTheta1Min_rad,
                                  JointLimits::kTheta1Max_rad,
                                  tolerance) ||
        !withinSoftLimitTolerance(posture.theta2,
                                  JointLimits::kTheta2Min_rad,
                                  JointLimits::kTheta2Max_rad,
                                  tolerance) ||
        !withinSoftLimitTolerance(posture.theta3,
                                  JointLimits::kTheta3Min_rad,
                                  JointLimits::kTheta3Max_rad,
                                  tolerance)) {
        posture.valid = false;
        posture.error = ErrorCode::JOINT_LIMIT_EXCEEDED;
        return;
    }

    posture.theta0 = clampToSoftLimit(posture.theta0,
                                      JointLimits::kTheta0SoftMin_rad,
                                      JointLimits::kTheta0SoftMax_rad);
    posture.theta1 = clampToSoftLimit(posture.theta1,
                                      JointLimits::kTheta1Min_rad,
                                      JointLimits::kTheta1Max_rad);
    posture.theta2 = clampToSoftLimit(posture.theta2,
                                      JointLimits::kTheta2Min_rad,
                                      JointLimits::kTheta2Max_rad);
    posture.theta3 = clampToSoftLimit(posture.theta3,
                                      JointLimits::kTheta3Min_rad,
                                      JointLimits::kTheta3Max_rad);
}

}  // anonymous namespace

// ---------- Public API ----------

PostureSolution mapIkToJointAngles(const PostureSolution& geometricAngles) {
    PostureSolution jointAngles = geometricAngles;
    jointAngles.theta2 = wrapRadians(JointLimits::kElbowSign *
                                     geometricAngles.theta2);
    jointAngles.theta3 = wrapRadians(JointLimits::kWristSign *
                                     geometricAngles.theta3);
    return jointAngles;
}

PostureSolution mapJointToGeometricAngles(const PostureSolution& jointAngles) {
    PostureSolution geometricAngles = jointAngles;
    geometricAngles.theta2 = wrapRadians(JointLimits::kElbowSign *
                                         jointAngles.theta2);
    geometricAngles.theta3 = wrapRadians(JointLimits::kWristSign *
                                         jointAngles.theta3);
    return geometricAngles;
}

IKResult solveIK(const IKInput& input) {
    IKResult result = {};
    if (!std::isfinite(input.x) || !std::isfinite(input.y) ||
        !std::isfinite(input.z) || !std::isfinite(input.phi)) {
        result.elbow_up = {0.0f, 0.0f, 0.0f, 0.0f, false,
                           ErrorCode::INVALID_INPUT};
        result.elbow_down = result.elbow_up;
        return result;
    }

    result.input = input;
    result.step_count = 0;

    const float L1 = JointLimits::kL1_cm;
    const float L2 = JointLimits::kL2_cm;
    const float L3 = JointLimits::kL3_cm;
    const float d1 = JointLimits::kD1_cm;

    result.dw_min = std::fabs(L1 - L2);   // |L1 − L2|
    result.dw_max = L1 + L2;              //  L1 + L2

    // ================================================================
    // Step 1 — Base twist
    // ================================================================
    const float theta0 = std::atan2(input.y, input.x);
    addStep(result, "base_twist",
            "\xCE\xB8\x30 = atan2(y, x)", theta0);  // θ0

    // ================================================================
    // Step 2 — Reduce to the arm plane
    // ================================================================
    const float r = std::hypot(input.x, input.y);
    addStep(result, "planar_reduction",
            "r = \xE2\x88\x9A(x\xC2\xB2 + y\xC2\xB2)", r);  // √(x² + y²)

    const float z_prime = input.z - d1;
    addStep(result, "planar_z_prime",
            "z' = z \xE2\x88\x92 d1", z_prime);  // z − d1

    // ================================================================
    // Step 3 — Wrist center (back off along approach angle φ by L3)
    // ================================================================
    const float rw = r - L3 * std::cos(input.phi);
    addStep(result, "wrist_center_r",
            "rw = r \xE2\x88\x92 L3\xC2\xB7""cos(\xCF\x86)", rw);

    const float zw = z_prime - L3 * std::sin(input.phi);
    addStep(result, "wrist_center_z",
            "zw = z' \xE2\x88\x92 L3\xC2\xB7""sin(\xCF\x86)", zw);

    // ================================================================
    // Step 4 — 2-link solve for the wrist center
    // ================================================================
    const float Dw = std::hypot(rw, zw);
    const float maxDwBeforeSquare = std::sqrt(std::numeric_limits<float>::max());
    const float Dw_sq = Dw > maxDwBeforeSquare
        ? std::numeric_limits<float>::max()
        : Dw * Dw;
    addStep(result, "wrist_dist_sq",
            "Dw\xC2\xB2 = rw\xC2\xB2 + zw\xC2\xB2", Dw_sq);

    result.dw = Dw;
    addStep(result, "wrist_dist",
            "Dw = \xE2\x88\x9A(Dw\xC2\xB2)", Dw);

    // ---- Reachability pre-check: |L1 − L2| ≤ Dw ≤ (L1 + L2) ----
    if (Dw < result.dw_min || Dw > result.dw_max) {
        result.reachable = false;

        // Both postures unreachable.  Still populate θ0 so the UI can
        // orient the base visualization.
        result.elbow_down = {theta0, 0, 0, 0, false,
                             ErrorCode::TARGET_UNREACHABLE};
        result.elbow_up   = {theta0, 0, 0, 0, false,
                             ErrorCode::TARGET_UNREACHABLE};
        return result;
    }
    result.reachable = true;

    // cos(θ2) — clamped to [−1, 1] for floating-point safety
    float cos_theta2 = (Dw_sq - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    addStep(result, "cos_theta2",
            "cos(\xCE\xB8\x32) = (Dw\xC2\xB2 \xE2\x88\x92 L1\xC2\xB2 \xE2\x88\x92 L2\xC2\xB2) / (2\xC2\xB7L1\xC2\xB7L2)",
            cos_theta2);

    cos_theta2 = std::fmax(-1.0f, std::fmin(1.0f, cos_theta2));
    const float sin_theta2 = std::sqrt(1.0f - cos_theta2 * cos_theta2);

    // ---- Elbow-down: +√  (spec §3.3: "+" gives elbow-down) ----
    const float theta2_down = std::atan2(sin_theta2, cos_theta2);
    addStep(result, "theta2_down",
            "\xCE\xB8\x32 = atan2(+\xE2\x88\x9A(1\xE2\x88\x92""cos\xC2\xB2\xCE\xB8\x32), cos \xCE\xB8\x32)",
            theta2_down);

    const float theta1_down = std::atan2(zw, rw)
        - std::atan2(L2 * std::sin(theta2_down),
                     L1 + L2 * std::cos(theta2_down));
    addStep(result, "theta1_down",
            "\xCE\xB8\x31 = atan2(zw, rw) \xE2\x88\x92 atan2(L2\xC2\xB7""sin \xCE\xB8\x32, L1+L2\xC2\xB7""cos \xCE\xB8\x32)",
            theta1_down);

    const float theta3_down = input.phi - theta1_down - theta2_down;
    addStep(result, "theta3_down",
            "\xCE\xB8\x33 = \xCF\x86 \xE2\x88\x92 \xCE\xB8\x31 \xE2\x88\x92 \xCE\xB8\x32",
            theta3_down);

    // ---- Elbow-up: −√  (spec §3.3: "−" gives elbow-up) ----
    const float theta2_up = std::atan2(-sin_theta2, cos_theta2);
    addStep(result, "theta2_up",
            "\xCE\xB8\x32 = atan2(\xE2\x88\x92\xE2\x88\x9A(1\xE2\x88\x92""cos\xC2\xB2\xCE\xB8\x32), cos \xCE\xB8\x32)",
            theta2_up);

    const float theta1_up = std::atan2(zw, rw)
        - std::atan2(L2 * std::sin(theta2_up),
                     L1 + L2 * std::cos(theta2_up));
    addStep(result, "theta1_up",
            "\xCE\xB8\x31 = atan2(zw, rw) \xE2\x88\x92 atan2(L2\xC2\xB7""sin \xCE\xB8\x32, L1+L2\xC2\xB7""cos \xCE\xB8\x32)",
            theta1_up);

    const float theta3_up = input.phi - theta1_up - theta2_up;
    addStep(result, "theta3_up",
            "\xCE\xB8\x33 = \xCF\x86 \xE2\x88\x92 \xCE\xB8\x31 \xE2\x88\x92 \xCE\xB8\x32",
            theta3_up);

    // ---- Populate solutions ----
    const PostureSolution geometricDown = {
        theta0, theta1_down, theta2_down, theta3_down,
        true, ErrorCode::NONE,
    };
    const PostureSolution geometricUp = {
        theta0, theta1_up, theta2_up, theta3_up,
        true, ErrorCode::NONE,
    };
    result.elbow_down = mapIkToJointAngles(geometricDown);
    result.elbow_up = mapIkToJointAngles(geometricUp);

    // With kElbowSign = -1, elbow_down maps positive IK theta2 negative;
    // it can satisfy the nonnegative joint limit only near full extension.
    // ---- Post-check: joint soft limits (spec §3.4 step 2) ----
    validatePosture(result.elbow_down);
    validatePosture(result.elbow_up);

    return result;
}

FKResult solveFK(float theta0, float theta1, float theta2, float theta3) {
    FKResult result = {};
    const PostureSolution geometric = mapJointToGeometricAngles(
        {theta0, theta1, theta2, theta3, true, ErrorCode::NONE});
    theta2 = geometric.theta2;
    theta3 = geometric.theta3;

    const float L1 = JointLimits::kL1_cm;
    const float L2 = JointLimits::kL2_cm;
    const float L3 = JointLimits::kL3_cm;
    const float d1 = JointLimits::kD1_cm;

    // ---- 2D arm-plane positions (r, z) ----
    result.base_2d     = {0.0f, 0.0f};
    result.shoulder_2d = {0.0f, d1};

    // Elbow = shoulder + L1 in direction θ1 from horizontal
    const float elbow_r = L1 * std::cos(theta1);
    const float elbow_z = d1 + L1 * std::sin(theta1);
    result.elbow_2d = {elbow_r, elbow_z};

    // Wrist = elbow + L2 in direction (θ1 + θ2)
    const float phi12 = theta1 + theta2;
    const float wrist_r = elbow_r + L2 * std::cos(phi12);
    const float wrist_z = elbow_z + L2 * std::sin(phi12);
    result.wrist_2d = {wrist_r, wrist_z};

    // Effector = wrist + L3 in direction (θ1 + θ2 + θ3) = φ
    const float phi123 = phi12 + theta3;
    const float eff_r = wrist_r + L3 * std::cos(phi123);
    const float eff_z = wrist_z + L3 * std::sin(phi123);
    result.effector_2d = {eff_r, eff_z};

    // ---- 3D positions: rotate r-component around the vertical axis by θ0 ----
    const float c0 = std::cos(theta0);
    const float s0 = std::sin(theta0);

    result.base_3d     = {0.0f, 0.0f, 0.0f};
    result.shoulder_3d = {0.0f, 0.0f, d1};    // shoulder is on the z-axis
    result.elbow_3d    = {elbow_r * c0,  elbow_r * s0,  elbow_z};
    result.wrist_3d    = {wrist_r * c0,  wrist_r * s0,  wrist_z};
    result.effector_3d = {eff_r   * c0,  eff_r   * s0,  eff_z};

    return result;
}

bool checkJointLimits(float theta0, float theta1, float theta2, float theta3) {
    return theta0 >= JointLimits::kTheta0SoftMin_rad &&
           theta0 <= JointLimits::kTheta0SoftMax_rad &&
           theta1 >= JointLimits::kTheta1Min_rad     &&
           theta1 <= JointLimits::kTheta1Max_rad     &&
           theta2 >= JointLimits::kTheta2Min_rad     &&
           theta2 <= JointLimits::kTheta2Max_rad     &&
           theta3 >= JointLimits::kTheta3Min_rad     &&
           theta3 <= JointLimits::kTheta3Max_rad;
}

}  // namespace Kinematics
