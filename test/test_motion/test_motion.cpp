// Unit tests for motion profiling (Milestone 3).
//
// Covers SCurveProfile (7-segment jerk-limited) and TrajectoryPlanner
// (shared-duration, s ∈ [0,1] synchronization).
//
// Run:  pio test -e native
//
// Pure math — no Arduino / ESP32.

#include <unity.h>

#include <cmath>

#include "JointLimits.h"
#include "MotionConfig.h"
#include "SCurveProfile.h"
#include "ServoDriver.h"
#include "StepperDriver.h"
#include "TrajectoryPlanner.h"

static constexpr float kTimeTol = 1e-4f;
static constexpr float kPosTol = 1e-4f;
static constexpr float kVelTol = 2e-3f;

void setUp(void) {}
void tearDown(void) {}

static constexpr float deg(float d) { return JointLimits::degToRad(d); }

static int sampleCount(const Motion::SCurveProfile& p) {
    return (p.total_duration < 1e-9f) ? 1 : 401;
}

void test_servo_conversion_pulse_and_duty() {
    TEST_ASSERT_EQUAL_UINT16(1229, Motion::ServoDriver::pulseToDuty(1500));
    TEST_ASSERT_EQUAL_UINT16(410, Motion::ServoDriver::pulseToDuty(500));
    TEST_ASSERT_EQUAL_UINT16(2048, Motion::ServoDriver::pulseToDuty(2500));
}

void test_stepper_output_shaft_conversion() {
    TEST_ASSERT_EQUAL_INT32(4000, Motion::StepperDriver::stepsFromDeg(90.0f));
    TEST_ASSERT_EQUAL_INT32(16000, Motion::StepperDriver::stepsFromDeg(360.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 90.0f,
                             Motion::StepperDriver::degFromSteps(
                                 Motion::StepperDriver::stepsFromDeg(90.0f)));
}

// ====================================================================
//  SCurveProfile
// ====================================================================

void test_scurve_zero_distance(void) {
    auto p = Motion::SCurveProfile::compute(0.0f, 1.0f, 2.0f, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(kTimeTol, 0.0f, p.total_duration);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, 0.0f, p.position(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, 0.0f, p.position(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(kVelTol, 0.0f, p.velocity(0.1f));
}

void test_scurve_zero_limits(void) {
    auto p = Motion::SCurveProfile::compute(1.0f, 0.0f, 1.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(kTimeTol, 0.0f, p.total_duration);
}

void test_scurve_endpoints(void) {
    const float D = 2.0f;
    auto p = Motion::SCurveProfile::compute(D, 1.0f, 1.0f, 8.0f);

    TEST_ASSERT_TRUE(p.total_duration > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, 0.0f, p.position(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, D, p.position(p.total_duration));
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, D, p.position(p.total_duration + 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(kVelTol, 0.0f, p.velocity(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(kVelTol, 0.0f, p.velocity(p.total_duration));
}

void test_scurve_respects_vmax(void) {
    const float V = 1.5f;
    const float A = 4.0f;
    const float J = 20.0f;
    const float D = 5.0f;
    auto p = Motion::SCurveProfile::compute(D, V, A, J);

    float peak_v = 0.0f;
    float peak_a = 0.0f;
    const int n = sampleCount(p);
    for (int i = 0; i <= n; ++i) {
        const float t = p.total_duration * static_cast<float>(i) / static_cast<float>(n);
        peak_v = std::fmax(peak_v, std::fabs(p.velocity(t)));
        peak_a = std::fmax(peak_a, std::fabs(p.acceleration(t)));
        TEST_ASSERT_TRUE(p.position(t) >= -kPosTol);
        TEST_ASSERT_TRUE(p.position(t) <= D + kPosTol);
    }
    TEST_ASSERT_TRUE(peak_v <= V + 1e-3f);
    TEST_ASSERT_TRUE(peak_a <= A + 1e-2f);
    TEST_ASSERT_TRUE(p.peak_velocity <= V + 1e-3f);
}

void test_scurve_long_move_has_cruise(void) {
    // High jerk so the accel limit is reached; long distance → cruise.
    const float V = 1.0f;
    const float A = 1.0f;
    const float J = 1000.0f;
    const float D = 4.0f;
    auto p = Motion::SCurveProfile::compute(D, V, A, J);

    TEST_ASSERT_TRUE(p.seg[3].duration > 0.1f);  // cruise
    TEST_ASSERT_FLOAT_WITHIN(0.02f, V, p.peak_velocity);
}

void test_scurve_short_move_no_cruise(void) {
    auto p = Motion::SCurveProfile::compute(0.05f, 2.0f, 4.0f, 30.0f);
    TEST_ASSERT_TRUE(p.total_duration > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, p.seg[3].duration);
    TEST_ASSERT_TRUE(p.peak_velocity < 2.0f);
}

void test_scurve_position_matches_integrated_velocity(void) {
    auto p = Motion::SCurveProfile::compute(1.25f, 2.0f, 5.0f, 40.0f);
    const float dt = p.total_duration / 800.0f;
    float integ = 0.0f;
    for (int i = 0; i < 800; ++i) {
        const float t = (static_cast<float>(i) + 0.5f) * dt;
        integ += p.velocity(t) * dt;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.02f, p.total_distance, integ);
}

void test_scurve_unit_vs_scaled_distance_same_duration(void) {
    const float D = 0.7f;
    const float V = 1.2f;
    const float A = 3.0f;
    const float J = 25.0f;
    auto full = Motion::SCurveProfile::compute(D, V, A, J);
    auto unit = Motion::SCurveProfile::compute(1.0f, V / D, A / D, J / D);
    TEST_ASSERT_FLOAT_WITHIN(kTimeTol, full.total_duration, unit.total_duration);
}

// ====================================================================
//  TrajectoryPlanner
// ====================================================================

static Motion::JointVector pose(float t0, float t1, float t2, float t3,
                                float g = 0.0f) {
    return {t0, t1, t2, t3, g};
}

void test_plan_no_motion(void) {
    const auto q = pose(deg(10), deg(20), deg(30), deg(40), deg(0));
    auto p = Motion::TrajectoryPlanner::plan(q, q, Motion::PayloadProfile::Empty);

    TEST_ASSERT_FLOAT_WITHIN(kTimeTol, 0.0f, p.duration);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, 1.0f, p.progress(0.0f));
    auto a = p.anglesAt(0.0f);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, q.theta1, a.theta1);
    TEST_ASSERT_FLOAT_WITHIN(kVelTol, 0.0f, p.velocitiesAt(0.0f).theta1);
}

void test_plan_single_joint_matches_solo_scurve(void) {
    const auto start = pose(0, 0, 0, 0, 0);
    const auto end = pose(deg(40), 0, 0, 0, 0);
    auto p = Motion::TrajectoryPlanner::plan(start, end, Motion::PayloadProfile::Empty);

    auto solo = Motion::SCurveProfile::compute(
        p.joints[0].distance, p.joints[0].v_max, p.joints[0].a_max,
        p.joints[0].j_max);

    TEST_ASSERT_EQUAL(0, p.limiting_joint);
    TEST_ASSERT_FLOAT_WITHIN(kTimeTol, solo.total_duration, p.joints[0].time_optimal);
    TEST_ASSERT_FLOAT_WITHIN(kTimeTol, solo.total_duration, p.duration);
    TEST_ASSERT_TRUE(p.duration > 0.0f);
}

void test_plan_shared_duration_is_slowest_joint(void) {
    const auto start = pose(0, 0, 0, 0, 0);
    // Small base move vs large shoulder move — shoulder should dominate.
    const auto end = pose(deg(5), deg(90), 0, 0, 0);
    auto p = Motion::TrajectoryPlanner::plan(start, end, Motion::PayloadProfile::Empty);

    float max_opt = 0.0f;
    for (int i = 0; i < Motion::kJointCount; ++i) {
        max_opt = std::fmax(max_opt, p.joints[i].time_optimal);
    }
    TEST_ASSERT_TRUE(p.duration + 1e-5f >= max_opt);
    // Shared shape uses min of normalized limits, so duration equals max_opt
    // when one joint dominates v/a/j, and is only slightly longer otherwise.
    TEST_ASSERT_TRUE(p.duration <= max_opt * 1.05f + 1e-4f);
    TEST_ASSERT_TRUE(p.joints[1].time_optimal > p.joints[0].time_optimal);
    TEST_ASSERT_EQUAL(1, p.limiting_joint);
}

void test_plan_all_joints_start_and_stop_together(void) {
    const auto start = pose(0, deg(20), deg(30), deg(40), 0);
    const auto end = pose(deg(50), deg(80), deg(10), deg(70), deg(90));
    auto p = Motion::TrajectoryPlanner::plan(start, end, Motion::PayloadProfile::Empty);

    TEST_ASSERT_TRUE(p.duration > 0.0f);

    auto a0 = p.anglesAt(-1.0f);
    auto aT = p.anglesAt(p.duration + 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, start.theta0, a0.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, start.theta1, a0.theta1);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, start.theta2, a0.theta2);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, start.theta3, a0.theta3);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, start.gripper, a0.gripper);

    TEST_ASSERT_FLOAT_WITHIN(kPosTol, end.theta0, aT.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, end.theta1, aT.theta1);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, end.theta2, aT.theta2);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, end.theta3, aT.theta3);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, end.gripper, aT.gripper);

    auto v0 = p.velocitiesAt(0.0f);
    auto vT = p.velocitiesAt(p.duration);
    TEST_ASSERT_FLOAT_WITHIN(kVelTol, 0.0f, v0.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kVelTol, 0.0f, v0.theta1);
    TEST_ASSERT_FLOAT_WITHIN(kVelTol, 0.0f, vT.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kVelTol, 0.0f, vT.theta1);
}

void test_plan_progress_is_normalized_s(void) {
    const auto start = pose(0, 0, 0, 0, 0);
    const auto end = pose(0, deg(45), 0, 0, 0);
    auto p = Motion::TrajectoryPlanner::plan(start, end, Motion::PayloadProfile::Empty);

    TEST_ASSERT_FLOAT_WITHIN(kPosTol, 0.0f, p.progress(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, 1.0f, p.progress(p.duration));

    const float s_mid = p.progress(p.duration * 0.5f);
    TEST_ASSERT_TRUE(s_mid > 0.0f && s_mid < 1.0f);

    auto mid = p.anglesAt(p.duration * 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, start.theta1 + s_mid * deg(45), mid.theta1);
}

void test_plan_loaded_is_slower_than_empty(void) {
    const auto start = pose(0, 0, 0, 0, 0);
    const auto end = pose(deg(30), deg(60), deg(40), deg(20), 0);

    auto empty = Motion::TrajectoryPlanner::plan(start, end, Motion::PayloadProfile::Empty);
    auto loaded = Motion::TrajectoryPlanner::plan(start, end, Motion::PayloadProfile::Loaded);

    TEST_ASSERT_EQUAL(static_cast<int>(Motion::PayloadProfile::Loaded),
                      static_cast<int>(loaded.profile));
    TEST_ASSERT_TRUE(loaded.duration > empty.duration * 1.2f);
    TEST_ASSERT_FLOAT_WITHIN(
        1e-5f, empty.joints[1].v_max * MotionConfig::kLoadedDerate,
        loaded.joints[1].v_max);
}

void test_plan_respects_joint_velocity_limits(void) {
    const auto start = pose(0, 0, 0, 0, 0);
    const auto end = pose(deg(80), deg(70), deg(50), deg(40), deg(90));
    auto p = Motion::TrajectoryPlanner::plan(start, end, Motion::PayloadProfile::Empty);

    const int n = 500;
    for (int k = 0; k <= n; ++k) {
        const float t = p.duration * static_cast<float>(k) / static_cast<float>(n);
        auto vel = p.velocitiesAt(t);
        for (int i = 0; i < Motion::kJointCount; ++i) {
            const float v = std::fabs(Motion::jointAt(vel, i));
            TEST_ASSERT_TRUE(v <= p.joints[i].v_max * 1.02f);
        }
    }
}

void test_plan_stationary_joint_stays_put(void) {
    const auto start = pose(deg(12), deg(33), deg(44), deg(55), deg(10));
    auto end = start;
    end.theta2 = deg(100);
    auto p = Motion::TrajectoryPlanner::plan(start, end, Motion::PayloadProfile::Empty);

    auto mid = p.anglesAt(p.duration * 0.37f);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, start.theta0, mid.theta0);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, start.theta1, mid.theta1);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, start.theta3, mid.theta3);
    TEST_ASSERT_FLOAT_WITHIN(kPosTol, start.gripper, mid.gripper);
    TEST_ASSERT_TRUE(std::fabs(mid.theta2 - start.theta2) > deg(1.0f));
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(test_servo_conversion_pulse_and_duty);
    RUN_TEST(test_stepper_output_shaft_conversion);

    RUN_TEST(test_scurve_zero_distance);
    RUN_TEST(test_scurve_zero_limits);
    RUN_TEST(test_scurve_endpoints);
    RUN_TEST(test_scurve_respects_vmax);
    RUN_TEST(test_scurve_long_move_has_cruise);
    RUN_TEST(test_scurve_short_move_no_cruise);
    RUN_TEST(test_scurve_position_matches_integrated_velocity);
    RUN_TEST(test_scurve_unit_vs_scaled_distance_same_duration);

    RUN_TEST(test_plan_no_motion);
    RUN_TEST(test_plan_single_joint_matches_solo_scurve);
    RUN_TEST(test_plan_shared_duration_is_slowest_joint);
    RUN_TEST(test_plan_all_joints_start_and_stop_together);
    RUN_TEST(test_plan_progress_is_normalized_s);
    RUN_TEST(test_plan_loaded_is_slower_than_empty);
    RUN_TEST(test_plan_respects_joint_velocity_limits);
    RUN_TEST(test_plan_stationary_joint_stays_put);

    return UNITY_END();
}
