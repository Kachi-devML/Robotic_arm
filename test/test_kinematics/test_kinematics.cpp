// Unit tests for the Kinematics module (Milestone 2).
//
// Verifies solveIK(), solveFK(), and checkJointLimits() against known
// analytical results and round-trip (FK → IK) consistency.
//
// Run:  pio test -e native
//
// All angles in radians.  All distances in centimeters.

#include <unity.h>

#include <cmath>
#include <limits>

#include "JointLimits.h"
#include "Kinematics.h"

// ---- Tolerances ----

static constexpr float kAngleTol = 1e-4f;  // ~0.006° — generous for float32
static constexpr float kDistTol  = 1e-3f;  // cm

// ---- Helpers ----

static constexpr float kPi = JointLimits::kPi;

static constexpr float deg(float d) { return JointLimits::degToRad(d); }

Kinematics::IKInput targetFromJointAngles(float theta0,
                                          float theta1,
                                          float theta2,
                                          float theta3) {
    const Kinematics::PostureSolution jointAngles = {
        theta0, theta1, theta2, theta3, true, Kinematics::ErrorCode::NONE,
    };
    const auto geometric =
        Kinematics::mapJointToGeometricAngles(jointAngles);
    const auto fk = Kinematics::solveFK(theta0, theta1, theta2, theta3);
    return {
        fk.effector_3d.x,
        fk.effector_3d.y,
        fk.effector_3d.z,
        geometric.theta1 + geometric.theta2 + geometric.theta3,
    };
}

void setUp(void) {}
void tearDown(void) {}

bool allNumericFieldsFinite(const Kinematics::IKResult& result) {
    if (!std::isfinite(result.input.x) || !std::isfinite(result.input.y) ||
        !std::isfinite(result.input.z) || !std::isfinite(result.input.phi) ||
        !std::isfinite(result.elbow_up.theta0) ||
        !std::isfinite(result.elbow_up.theta1) ||
        !std::isfinite(result.elbow_up.theta2) ||
        !std::isfinite(result.elbow_up.theta3) ||
        !std::isfinite(result.elbow_down.theta0) ||
        !std::isfinite(result.elbow_down.theta1) ||
        !std::isfinite(result.elbow_down.theta2) ||
        !std::isfinite(result.elbow_down.theta3) ||
        !std::isfinite(result.dw) || !std::isfinite(result.dw_min) ||
        !std::isfinite(result.dw_max)) {
        return false;
    }
    for (const auto& step : result.steps) {
        if (!std::isfinite(step.value)) return false;
    }
    return true;
}

void assertInvalidInputResult(const Kinematics::IKResult& result) {
    TEST_ASSERT_FALSE(result.reachable);
    TEST_ASSERT_FALSE(result.elbow_up.valid);
    TEST_ASSERT_FALSE(result.elbow_down.valid);
    TEST_ASSERT_EQUAL(Kinematics::ErrorCode::INVALID_INPUT, result.elbow_up.error);
    TEST_ASSERT_EQUAL(Kinematics::ErrorCode::INVALID_INPUT, result.elbow_down.error);
    TEST_ASSERT_EQUAL_UINT8(0, result.step_count);
    TEST_ASSERT_TRUE(allNumericFieldsFinite(result));
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.input.x);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.input.y);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.input.z);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.input.phi);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.elbow_up.theta0);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.elbow_up.theta1);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.elbow_up.theta2);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.elbow_up.theta3);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.elbow_down.theta0);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.elbow_down.theta1);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.elbow_down.theta2);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.elbow_down.theta3);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.dw);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.dw_min);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, result.dw_max);
    for (const auto& step : result.steps) {
        TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, step.value);
        TEST_ASSERT_NULL(step.label);
        TEST_ASSERT_NULL(step.expression);
    }
}

void assertValidAnglesWithinLimits(const Kinematics::IKResult& result) {
    const Kinematics::PostureSolution* postures[] = {
        &result.elbow_up,
        &result.elbow_down,
    };
    for (const auto* posture : postures) {
        if (!posture->valid) continue;
        TEST_ASSERT_TRUE(posture->theta0 >= JointLimits::kTheta0SoftMin_rad);
        TEST_ASSERT_TRUE(posture->theta0 <= JointLimits::kTheta0SoftMax_rad);
        TEST_ASSERT_TRUE(posture->theta1 >= JointLimits::kTheta1Min_rad);
        TEST_ASSERT_TRUE(posture->theta1 <= JointLimits::kTheta1Max_rad);
        TEST_ASSERT_TRUE(posture->theta2 >= JointLimits::kTheta2Min_rad);
        TEST_ASSERT_TRUE(posture->theta2 <= JointLimits::kTheta2Max_rad);
        TEST_ASSERT_TRUE(posture->theta3 >= JointLimits::kTheta3Min_rad);
        TEST_ASSERT_TRUE(posture->theta3 <= JointLimits::kTheta3Max_rad);
    }
}

void assertPostureRoundTrips(const Kinematics::IKInput& input,
                            const Kinematics::PostureSolution& posture) {
    TEST_ASSERT_TRUE(posture.valid);
    TEST_ASSERT_TRUE(Kinematics::checkJointLimits(
        posture.theta0, posture.theta1, posture.theta2, posture.theta3));

    const auto fk = Kinematics::solveFK(
        posture.theta0, posture.theta1, posture.theta2, posture.theta3);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, input.x, fk.effector_3d.x);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, input.y, fk.effector_3d.y);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, input.z, fk.effector_3d.z);
}

void assertAllValidPosturesRoundTrip(const Kinematics::IKInput& input,
                                     const Kinematics::IKResult& result) {
    bool foundValidPosture = false;
    if (result.elbow_up.valid) {
        foundValidPosture = true;
        assertPostureRoundTrips(input, result.elbow_up);
    }
    if (result.elbow_down.valid) {
        foundValidPosture = true;
        assertPostureRoundTrips(input, result.elbow_down);
    }
    TEST_ASSERT_TRUE_MESSAGE(foundValidPosture,
                             "expected at least one valid posture");
    assertValidAnglesWithinLimits(result);
}

void test_joint_angle_mapping_and_inverse(void) {
    const Kinematics::PostureSolution geometric = {
        deg(35.0f), deg(90.0f), deg(120.0f), deg(240.0f),
        true, Kinematics::ErrorCode::NONE,
    };
    const auto joint = Kinematics::mapIkToJointAngles(geometric);

    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, geometric.theta0, joint.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, geometric.theta1, joint.theta1);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, deg(-120.0f), joint.theta2);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, deg(120.0f), joint.theta3);
    TEST_ASSERT_TRUE(joint.valid);
    TEST_ASSERT_EQUAL(Kinematics::ErrorCode::NONE, joint.error);

    const auto recovered = Kinematics::mapJointToGeometricAngles(joint);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, geometric.theta0, recovered.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, geometric.theta1, recovered.theta1);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, deg(120.0f), recovered.theta2);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, deg(-120.0f), recovered.theta3);
    TEST_ASSERT_TRUE(recovered.valid);
    TEST_ASSERT_EQUAL(Kinematics::ErrorCode::NONE, recovered.error);
}

// ====================================================================
//  IK — basic reachability and correctness
// ====================================================================

void test_ik_rejects_non_finite_inputs(void) {
    const float nonFiniteValues[] = {
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
    };
    float Kinematics::IKInput::* const fields[] = {
        &Kinematics::IKInput::x,
        &Kinematics::IKInput::y,
        &Kinematics::IKInput::z,
        &Kinematics::IKInput::phi,
    };

    for (const auto field : fields) {
        for (const float value : nonFiniteValues) {
            Kinematics::IKInput input = {20.0f, 0.0f, 15.0f, 0.0f};
            input.*field = value;
            assertInvalidInputResult(Kinematics::solveIK(input));
        }
    }
}

void test_ik_huge_finite_input_has_finite_outputs(void) {
    const Kinematics::IKInput input = {1.0e30f, 0.0f, 12.0f, 0.0f};
    const auto result = Kinematics::solveIK(input);

    TEST_ASSERT_FALSE(result.reachable);
    TEST_ASSERT_TRUE(allNumericFieldsFinite(result));
}

void test_ik_soft_limit_tolerance_clamps_joint_angles(void) {
    const float t0 = 0.0f;
    const float t1 = deg(30.0f);
    const float t2 = deg(45.0f);
    const auto wristInput =
        targetFromJointAngles(t0, t1, t2, deg(-0.25f));
    const auto wristResult = Kinematics::solveIK(wristInput);
    TEST_ASSERT_TRUE(wristResult.elbow_up.valid);
    TEST_ASSERT_EQUAL(Kinematics::ErrorCode::NONE,
                      wristResult.elbow_up.error);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, JointLimits::kTheta3Min_rad,
                             wristResult.elbow_up.theta3);
    assertValidAnglesWithinLimits(wristResult);

    const float baseAtLimitPlusTolerance = deg(170.25f);
    const auto baseInput = targetFromJointAngles(
        baseAtLimitPlusTolerance, t1, t2, 0.0f);
    const auto baseResult = Kinematics::solveIK(baseInput);
    TEST_ASSERT_TRUE(baseResult.elbow_up.valid);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, JointLimits::kTheta0SoftMax_rad,
                             baseResult.elbow_up.theta0);
    assertValidAnglesWithinLimits(baseResult);
}

void test_ik_soft_limit_outside_tolerance_is_rejected(void) {
    const float t0 = 0.0f;
    const float t1 = deg(30.0f);
    const float t2 = deg(45.0f);
    const auto wristInput = targetFromJointAngles(t0, t1, t2, deg(-0.6f));
    const auto wristResult = Kinematics::solveIK(wristInput);
    TEST_ASSERT_FALSE(wristResult.elbow_up.valid);
    TEST_ASSERT_EQUAL(Kinematics::ErrorCode::JOINT_LIMIT_EXCEEDED,
                      wristResult.elbow_up.error);

    const auto baseInput =
        targetFromJointAngles(deg(170.6f), t1, t2, 0.0f);
    const auto baseResult = Kinematics::solveIK(baseInput);
    TEST_ASSERT_FALSE(baseResult.elbow_up.valid);
    TEST_ASSERT_EQUAL(Kinematics::ErrorCode::JOINT_LIMIT_EXCEEDED,
                      baseResult.elbow_up.error);
}

void test_ik_full_extension_is_stable_and_in_limits(void) {
    const Kinematics::IKInput input = {35.0f, 0.0f, 12.0f, 0.0f};
    const auto result = Kinematics::solveIK(input);

    TEST_ASSERT_TRUE(result.reachable);
    TEST_ASSERT_TRUE(result.elbow_up.valid);
    TEST_ASSERT_TRUE(result.elbow_down.valid);
    TEST_ASSERT_TRUE(allNumericFieldsFinite(result));
    assertValidAnglesWithinLimits(result);
}

void test_ik_revision3_g1(void) {
    const Kinematics::IKInput input = {20.0f, 0.0f, 27.0f, 0.0f};
    const auto result = Kinematics::solveIK(input);

    TEST_ASSERT_TRUE(result.reachable);
    TEST_ASSERT_TRUE(result.elbow_up.valid);
    TEST_ASSERT_FALSE(result.elbow_down.valid);
    TEST_ASSERT_EQUAL(Kinematics::ErrorCode::JOINT_LIMIT_EXCEEDED,
                      result.elbow_down.error);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, result.elbow_up.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, deg(90.0f), result.elbow_up.theta1);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, deg(90.0f), result.elbow_up.theta2);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, result.elbow_up.theta3);
    assertAllValidPosturesRoundTrip(input, result);
}

void test_ik_revision3_g2(void) {
    const Kinematics::IKInput input = {35.0f, 0.0f, 12.0f, 0.0f};
    const auto result = Kinematics::solveIK(input);

    TEST_ASSERT_TRUE(result.reachable);
    TEST_ASSERT_TRUE(result.elbow_up.valid);
    TEST_ASSERT_TRUE(result.elbow_down.valid);
    const Kinematics::PostureSolution* postures[] = {
        &result.elbow_up, &result.elbow_down,
    };
    for (const auto* posture : postures) {
        TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, posture->theta0);
        TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, posture->theta1);
        TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, posture->theta2);
        TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, posture->theta3);
        assertPostureRoundTrips(input, *posture);
    }
}

void test_ik_revision3_g3(void) {
    const Kinematics::IKInput input = {20.0f, 5.0f, 15.0f, deg(-90.0f)};
    const auto result = Kinematics::solveIK(input);

    TEST_ASSERT_TRUE(result.reachable);
    TEST_ASSERT_TRUE(result.elbow_up.valid);
    TEST_ASSERT_FALSE(result.elbow_down.valid);
    TEST_ASSERT_EQUAL(Kinematics::ErrorCode::JOINT_LIMIT_EXCEEDED,
                      result.elbow_down.error);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 14.04f,
                             JointLimits::radToDeg(result.elbow_up.theta0));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 54.65f,
                             JointLimits::radToDeg(result.elbow_up.theta1));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 60.55f,
                             JointLimits::radToDeg(result.elbow_up.theta2));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 84.10f,
                             JointLimits::radToDeg(result.elbow_up.theta3));
    assertAllValidPosturesRoundTrip(input, result);
}

void test_ik_near_max_reach_matches_double_precision_reference(void) {
    // Float conditioning near cos(theta2) = 1.
    const double dwTarget = 27.0 - 1.0e-4;
    const double phi = static_cast<double>(deg(90.0f));
    const double wristCenterAngle = JointLimits::degToRad(30.0);
    const double l1 = JointLimits::kL1_cm;
    const double l2 = JointLimits::kL2_cm;
    const double l3 = JointLimits::kL3_cm;
    const double d1 = JointLimits::kD1_cm;
    const Kinematics::IKInput input = {
        static_cast<float>(dwTarget * std::cos(wristCenterAngle) +
                           l3 * std::cos(phi)),
        0.0f,
        static_cast<float>(d1 + dwTarget * std::sin(wristCenterAngle) +
                           l3 * std::sin(phi)),
        static_cast<float>(phi),
    };
    const auto result = Kinematics::solveIK(input);

    const double rw = static_cast<double>(input.x) - l3 * std::cos(input.phi);
    const double zw = static_cast<double>(input.z) - d1 -
                      l3 * std::sin(input.phi);
    const double dw = std::hypot(rw, zw);
    const double cosTheta2 =
        (dw * dw - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    const double theta2 = -std::acos(cosTheta2);
    const double theta1 = std::atan2(zw, rw) -
        std::atan2(l2 * std::sin(theta2), l1 + l2 * std::cos(theta2));
    const double theta0 = 0.0;
    const double theta3 = static_cast<double>(input.phi) - theta1 - theta2;
    const auto& solution = result.elbow_up;

    TEST_ASSERT_TRUE(result.reachable);
    // This near-extension target is reachable but its wrist angle is outside
    // the configured joint range; the solver still reports its finite angles.
    TEST_ASSERT_FLOAT_WITHIN(
        0.005f, static_cast<float>(theta0 * 180.0 / JointLimits::kPi),
        JointLimits::radToDeg(solution.theta0));
    TEST_ASSERT_FLOAT_WITHIN(
        0.005f, static_cast<float>(theta1 * 180.0 / JointLimits::kPi),
        JointLimits::radToDeg(solution.theta1));
    TEST_ASSERT_FLOAT_WITHIN(
        0.005f, static_cast<float>(-theta2 * 180.0 / JointLimits::kPi),
        JointLimits::radToDeg(solution.theta2));
    TEST_ASSERT_FLOAT_WITHIN(
        0.005f, static_cast<float>(-theta3 * 180.0 / JointLimits::kPi),
        JointLimits::radToDeg(solution.theta3));
}

/// Full extension forward at shoulder height.
/// All joints at 0 rad, arm stretched straight out.
/// Both postures collapse to the same singular solution.
void test_ik_full_extension_forward(void) {
    // Target = (L1+L2+L3, 0, d1) with φ=0 (horizontal approach)
    Kinematics::IKInput in = {35.0f, 0.0f, 12.0f, 0.0f};
    auto r = Kinematics::solveIK(in);

    TEST_ASSERT_TRUE_MESSAGE(r.reachable, "max-reach target should be reachable");

    // Elbow-down
    TEST_ASSERT_TRUE(r.elbow_down.valid);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, r.elbow_down.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, r.elbow_down.theta1);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, r.elbow_down.theta2);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, r.elbow_down.theta3);

    // Elbow-up collapses to the same angles at full extension (singular)
    TEST_ASSERT_TRUE(r.elbow_up.valid);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, r.elbow_up.theta2);
}

/// Target beyond max reach → TARGET_UNREACHABLE for both postures.
/// Reach info (dw_min, dw_max) must still be populated (spec §3.4).
void test_ik_unreachable_too_far(void) {
    Kinematics::IKInput in = {40.0f, 0.0f, 12.0f, 0.0f};
    auto r = Kinematics::solveIK(in);

    TEST_ASSERT_FALSE(r.reachable);
    TEST_ASSERT_FALSE(r.elbow_down.valid);
    TEST_ASSERT_FALSE(r.elbow_up.valid);
    TEST_ASSERT_EQUAL(Kinematics::ErrorCode::TARGET_UNREACHABLE, r.elbow_down.error);
    TEST_ASSERT_EQUAL(Kinematics::ErrorCode::TARGET_UNREACHABLE, r.elbow_up.error);

    // Reach info still populated
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 3.0f,  r.dw_min);   // |L1−L2| = |15−12|
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 27.0f, r.dw_max);   //  L1+L2  = 15+12
    TEST_ASSERT_TRUE(r.dw > r.dw_max);
}

/// Wrist center inside the dead zone (Dw < |L1−L2|).
void test_ik_unreachable_too_close(void) {
    // x=8.5, y=0, z=12, φ=0  →  rw = 0.5, zw = 0  →  Dw = 0.5 < 3
    Kinematics::IKInput in = {8.5f, 0.0f, 12.0f, 0.0f};
    auto r = Kinematics::solveIK(in);

    TEST_ASSERT_FALSE(r.reachable);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.5f, r.dw);
    TEST_ASSERT_TRUE(r.dw < r.dw_min);
}

/// Target in the y>0 half-plane → θ0 = π/4.
void test_ik_base_angle_positive_y(void) {
    const float d = 35.0f;
    const float x = d * std::cos(kPi / 4.0f);
    const float y = d * std::sin(kPi / 4.0f);
    Kinematics::IKInput in = {x, y, 12.0f, 0.0f};
    auto r = Kinematics::solveIK(in);

    TEST_ASSERT_TRUE(r.reachable);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, kPi / 4.0f, r.elbow_down.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, kPi / 4.0f, r.elbow_up.theta0);
}

/// Target in the y<0 half-plane → θ0 negative.
void test_ik_base_angle_negative_y(void) {
    const float d = 35.0f;
    const float x = d * std::cos(-kPi / 6.0f);
    const float y = d * std::sin(-kPi / 6.0f);
    Kinematics::IKInput in = {x, y, 12.0f, 0.0f};
    auto r = Kinematics::solveIK(in);

    TEST_ASSERT_TRUE(r.reachable);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, -kPi / 6.0f, r.elbow_down.theta0);
}

// ====================================================================
//  IK — both postures and joint-limit post-check
// ====================================================================

/// With this mapping, elbow_down usually maps positive geometric θ2 to a
/// negative joint angle, so it is almost never valid except near extension.
void test_ik_elbow_down_limit_violation(void) {
    const float t0 = 0.0f;
    const float t1 = deg(30.0f);
    const float t2 = deg(45.0f);
    const float t3 = 0.0f;
    const Kinematics::IKInput in = targetFromJointAngles(t0, t1, t2, t3);
    auto r = Kinematics::solveIK(in);

    TEST_ASSERT_TRUE(r.elbow_up.valid);
    TEST_ASSERT_FALSE(r.elbow_down.valid);
    TEST_ASSERT_EQUAL(Kinematics::ErrorCode::JOINT_LIMIT_EXCEEDED,
                      r.elbow_down.error);
}

/// Both posture structs are always populated (even when one is invalid).
void test_ik_both_postures_always_populated(void) {
    Kinematics::IKInput in = {20.0f, 0.0f, 20.0f, deg(45.0f)};
    auto r = Kinematics::solveIK(in);

    TEST_ASSERT_TRUE(r.reachable);
    // The Revision 3 sign map negates geometric θ2 for joint-space results.
    TEST_ASSERT_TRUE(r.elbow_down.theta2 < 0.0f);
    TEST_ASSERT_TRUE(r.elbow_up.theta2 > 0.0f);
}

// ====================================================================
//  IK — derivation steps for the educational panel
// ====================================================================

/// Reachable target must emit the full derivation (14 steps: 7 shared +
/// 3 elbow-down + 3 elbow-up + cos_theta2).
void test_ik_derivation_steps_reachable(void) {
    Kinematics::IKInput in = {20.0f, 0.0f, 20.0f, 0.0f};
    auto r = Kinematics::solveIK(in);

    TEST_ASSERT_TRUE(r.reachable);
    TEST_ASSERT_EQUAL_UINT8(14, r.step_count);

    // First step must be the base twist
    TEST_ASSERT_EQUAL_STRING("base_twist", r.steps[0].label);
    TEST_ASSERT_NOT_NULL(r.steps[0].expression);
    TEST_ASSERT_EQUAL_STRING("planar_reduction", r.steps[1].label);

    // Last step should be theta3_up
    TEST_ASSERT_EQUAL_STRING("theta3_up",
                             r.steps[r.step_count - 1].label);
}

/// Unreachable target stops after the shared derivation steps (before
/// per-posture angle calculations).
void test_ik_derivation_steps_unreachable(void) {
    Kinematics::IKInput in = {40.0f, 0.0f, 12.0f, 0.0f};
    auto r = Kinematics::solveIK(in);

    TEST_ASSERT_FALSE(r.reachable);
    // Exactly the seven shared derivation steps precede posture calculations.
    TEST_ASSERT_EQUAL_UINT8(7, r.step_count);
}

// ====================================================================
//  IK ↔ FK round-trip consistency
// ====================================================================

/// Pick known joint angles → FK to get effector position → IK back →
/// verify the elbow-up solution recovers the original angles.
void test_ik_fk_roundtrip_simple(void) {
    const float t0 = 0.0f;
    const float t1 = deg(30.0f);   // π/6
    const float t2 = deg(45.0f);   // π/4
    const float t3 = 0.0f;
    const Kinematics::IKInput in = targetFromJointAngles(t0, t1, t2, t3);
    auto ik = Kinematics::solveIK(in);

    TEST_ASSERT_TRUE(ik.reachable);
    TEST_ASSERT_TRUE(ik.elbow_up.valid);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t0, ik.elbow_up.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t1, ik.elbow_up.theta1);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t2, ik.elbow_up.theta2);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t3, ik.elbow_up.theta3);
}

/// Round-trip with a non-zero base angle and all four joints non-trivial.
void test_ik_fk_roundtrip_rotated(void) {
    const float t0 = deg(60.0f);
    const float t1 = deg(45.0f);
    const float t2 = deg(30.0f);
    const float t3 = deg(15.0f);
    const Kinematics::IKInput in = targetFromJointAngles(t0, t1, t2, t3);
    auto ik = Kinematics::solveIK(in);

    TEST_ASSERT_TRUE(ik.reachable);
    TEST_ASSERT_TRUE(ik.elbow_up.valid);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t0, ik.elbow_up.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t1, ik.elbow_up.theta1);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t2, ik.elbow_up.theta2);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t3, ik.elbow_up.theta3);
}

/// Round-trip with a negative base angle (third quadrant target).
void test_ik_fk_roundtrip_negative_base(void) {
    const float t0 = deg(-45.0f);
    const float t1 = deg(50.0f);
    const float t2 = deg(60.0f);
    const float t3 = deg(10.0f);
    const Kinematics::IKInput in = targetFromJointAngles(t0, t1, t2, t3);
    auto ik = Kinematics::solveIK(in);

    TEST_ASSERT_TRUE(ik.reachable);
    TEST_ASSERT_TRUE(ik.elbow_up.valid);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t0, ik.elbow_up.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t1, ik.elbow_up.theta1);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t2, ik.elbow_up.theta2);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t3, ik.elbow_up.theta3);
}

/// Round-trip at near-minimum reach (Dw just above |L1−L2| = 3 cm).
void test_ik_fk_roundtrip_near_min_reach(void) {
    // Large elbow angle → wrist center close to shoulder axis.
    const float t0 = 0.0f;
    const float t1 = deg(60.0f);
    const float t2 = deg(150.0f);  // nearly folded back
    const float t3 = deg(10.0f);
    const Kinematics::IKInput in = targetFromJointAngles(t0, t1, t2, t3);
    auto ik = Kinematics::solveIK(in);

    TEST_ASSERT_TRUE(ik.reachable);
    TEST_ASSERT_TRUE(ik.elbow_up.valid);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t0, ik.elbow_up.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t1, ik.elbow_up.theta1);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t2, ik.elbow_up.theta2);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, t3, ik.elbow_up.theta3);
    // Either way, reach info must be populated
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 3.0f,  ik.dw_min);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 27.0f, ik.dw_max);
}

// ====================================================================
//  FK — known configurations
// ====================================================================

/// Fully extended forward: θ0=θ1=θ2=θ3=0.
/// Arm lies along the +x axis at height d1.
void test_fk_fully_extended_forward(void) {
    auto fk = Kinematics::solveFK(0.0f, 0.0f, 0.0f, 0.0f);

    // 2D arm-plane positions
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.0f,  fk.base_2d.r);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.0f,  fk.base_2d.z);

    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.0f,  fk.shoulder_2d.r);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 12.0f, fk.shoulder_2d.z);

    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 15.0f, fk.elbow_2d.r);    // L1
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 12.0f, fk.elbow_2d.z);

    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 27.0f, fk.wrist_2d.r);    // L1+L2
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 12.0f, fk.wrist_2d.z);

    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 35.0f, fk.effector_2d.r);  // L1+L2+L3
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 12.0f, fk.effector_2d.z);

    // 3D — with θ0=0, all x=r, all y=0
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 35.0f, fk.effector_3d.x);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.0f,  fk.effector_3d.y);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 12.0f, fk.effector_3d.z);
}

/// Shoulder at 90° (straight up), all others 0 → arm pointing straight up.
void test_fk_shoulder_straight_up(void) {
    auto fk = Kinematics::solveFK(0.0f, kPi / 2.0f, 0.0f, 0.0f);

    // Elbow: straight up from shoulder
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.0f,  fk.elbow_2d.r);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 27.0f, fk.elbow_2d.z);   // d1 + L1

    // Wrist: φ12 = π/2, so continue upward
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.0f,  fk.wrist_2d.r);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 39.0f, fk.wrist_2d.z);   // d1+L1+L2

    // Effector: φ123 = π/2, continue up
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.0f,  fk.effector_2d.r);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 47.0f, fk.effector_2d.z); // d1+L1+L2+L3
}

/// Home position: θ0=0, θ1=θ2=θ3=90° in joint space.
/// The Revision 3 signs make the forearm point forward and tool point down.
void test_fk_home_position(void) {
    const float h = kPi / 2.0f;
    auto fk = Kinematics::solveFK(0.0f, h, h, h);

    // Shoulder → straight up → elbow at (0, 27)
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.0f,  fk.elbow_2d.r);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 27.0f, fk.elbow_2d.z);

    // The elbow sign maps θ2=90° to geometric -90°, so L2 points forward.
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 12.0f, fk.wrist_2d.r);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 27.0f,  fk.wrist_2d.z);

    // The wrist sign maps θ3=90° to geometric -90°, so L3 points down.
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 12.0f, fk.effector_2d.r);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 19.0f,  fk.effector_2d.z);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.0f, fk.elbow_3d.x);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.0f, fk.elbow_3d.y);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 27.0f, fk.elbow_3d.z);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 12.0f, fk.wrist_3d.x);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.0f, fk.wrist_3d.y);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 27.0f, fk.wrist_3d.z);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 12.0f, fk.effector_3d.x);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.0f, fk.effector_3d.y);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 19.0f, fk.effector_3d.z);
}

/// 3D rotation: θ0=90° should rotate the r-component into the y-axis.
void test_fk_3d_base_rotation(void) {
    auto fk = Kinematics::solveFK(kPi / 2.0f, 0.0f, 0.0f, 0.0f);

    // Fully extended along +x in arm-plane, but θ0 rotates to +y in 3D
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 0.0f,  fk.effector_3d.x);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 35.0f, fk.effector_3d.y);
    TEST_ASSERT_FLOAT_WITHIN(kDistTol, 12.0f, fk.effector_3d.z);
}

// ====================================================================
//  checkJointLimits()
// ====================================================================

/// All joints within soft limits.
void test_limits_all_within(void) {
    TEST_ASSERT_TRUE(Kinematics::checkJointLimits(
        0.0f, kPi / 2.0f, kPi / 2.0f, kPi / 2.0f));
}

/// θ0 outside ±170°.
void test_limits_theta0_out(void) {
    TEST_ASSERT_FALSE(Kinematics::checkJointLimits(
        deg(175.0f), kPi / 2.0f, kPi / 2.0f, kPi / 2.0f));
    TEST_ASSERT_FALSE(Kinematics::checkJointLimits(
        deg(-175.0f), kPi / 2.0f, kPi / 2.0f, kPi / 2.0f));
}

/// θ1 below 0° or above 180°.
void test_limits_theta1_out(void) {
    TEST_ASSERT_FALSE(Kinematics::checkJointLimits(
        0.0f, deg(-1.0f), kPi / 2.0f, kPi / 2.0f));
    TEST_ASSERT_FALSE(Kinematics::checkJointLimits(
        0.0f, deg(181.0f), kPi / 2.0f, kPi / 2.0f));
}

/// θ2 out of range.
void test_limits_theta2_out(void) {
    TEST_ASSERT_FALSE(Kinematics::checkJointLimits(
        0.0f, kPi / 2.0f, deg(-5.0f), kPi / 2.0f));
}

/// θ3 out of range.
void test_limits_theta3_out(void) {
    TEST_ASSERT_FALSE(Kinematics::checkJointLimits(
        0.0f, kPi / 2.0f, kPi / 2.0f, deg(185.0f)));
}

/// Boundary values: exactly at ±170° for base, exactly 0° and 180° for servos.
void test_limits_boundary(void) {
    // Exactly at limits → should be valid
    TEST_ASSERT_TRUE(Kinematics::checkJointLimits(
        deg(170.0f), 0.0f, 0.0f, 0.0f));
    TEST_ASSERT_TRUE(Kinematics::checkJointLimits(
        deg(-170.0f), kPi, kPi, kPi));
}

// ====================================================================
//  Entry point
// ====================================================================

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(test_joint_angle_mapping_and_inverse);

    // IK — reachability & correctness
    RUN_TEST(test_ik_rejects_non_finite_inputs);
    RUN_TEST(test_ik_huge_finite_input_has_finite_outputs);
    RUN_TEST(test_ik_soft_limit_tolerance_clamps_joint_angles);
    RUN_TEST(test_ik_soft_limit_outside_tolerance_is_rejected);
    RUN_TEST(test_ik_full_extension_is_stable_and_in_limits);
    RUN_TEST(test_ik_revision3_g1);
    RUN_TEST(test_ik_revision3_g2);
    RUN_TEST(test_ik_revision3_g3);
    RUN_TEST(test_ik_near_max_reach_matches_double_precision_reference);
    RUN_TEST(test_ik_full_extension_forward);
    RUN_TEST(test_ik_unreachable_too_far);
    RUN_TEST(test_ik_unreachable_too_close);
    RUN_TEST(test_ik_base_angle_positive_y);
    RUN_TEST(test_ik_base_angle_negative_y);

    // IK — posture validation
    RUN_TEST(test_ik_elbow_down_limit_violation);
    RUN_TEST(test_ik_both_postures_always_populated);

    // IK — derivation steps (educational panel)
    RUN_TEST(test_ik_derivation_steps_reachable);
    RUN_TEST(test_ik_derivation_steps_unreachable);

    // IK ↔ FK round-trips
    RUN_TEST(test_ik_fk_roundtrip_simple);
    RUN_TEST(test_ik_fk_roundtrip_rotated);
    RUN_TEST(test_ik_fk_roundtrip_negative_base);
    RUN_TEST(test_ik_fk_roundtrip_near_min_reach);

    // FK — known configurations
    RUN_TEST(test_fk_fully_extended_forward);
    RUN_TEST(test_fk_shoulder_straight_up);
    RUN_TEST(test_fk_home_position);
    RUN_TEST(test_fk_3d_base_rotation);

    // Joint limit checks
    RUN_TEST(test_limits_all_within);
    RUN_TEST(test_limits_theta0_out);
    RUN_TEST(test_limits_theta1_out);
    RUN_TEST(test_limits_theta2_out);
    RUN_TEST(test_limits_theta3_out);
    RUN_TEST(test_limits_boundary);

    return UNITY_END();
}
