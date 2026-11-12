#include <unity.h>

#include "MotionConfig.h"
#include "Pins.h"
#include "ServoDriver.h"
#include "StepperDriver.h"

constexpr bool near(float actual, float expected, float tolerance = 0.001f) {
  return actual >= expected - tolerance && actual <= expected + tolerance;
}

static_assert(near(MotionConfig::STEPPER_STEPS_PER_OUTPUT_DEG, 44.444444f));
static_assert(near(MotionConfig::BASE_MAX_VEL_OUT_DPS, 90.0f));
static_assert(near(MotionConfig::kSafetyDerate, 0.55f));
static_assert(near(MotionConfig::kLoadedDerate, 0.50f));
static_assert(MotionConfig::STEPPER_ISR_HZ == 20000);
static_assert(Pins::SERVO_SHOULDER_PWM == 4);
static_assert(Pins::SERVO_ELBOW_PWM == 5);
static_assert(Pins::SERVO_WRIST_PWM == 6);
static_assert(Pins::SERVO_GRIPPER_PWM == 7);

void setUp(void) {}
void tearDown(void) {}

void test_stepper_degree_conversion_and_round_trip(void) {
  TEST_ASSERT_EQUAL_INT32(4000, Motion::StepperDriver::stepsFromDeg(90.0f));
  TEST_ASSERT_EQUAL_INT32(16000, Motion::StepperDriver::stepsFromDeg(360.0f));

  constexpr float oneMicrostepDeg =
      1.0f / MotionConfig::STEPPER_STEPS_PER_OUTPUT_DEG;
  constexpr float inputDeg = 123.456f;
  const float roundTrip =
      Motion::StepperDriver::degFromSteps(Motion::StepperDriver::stepsFromDeg(inputDeg));
  TEST_ASSERT_FLOAT_WITHIN(oneMicrostepDeg, inputDeg, roundTrip);
}

void test_servo_pulse_to_duty_conversion(void) {
  TEST_ASSERT_EQUAL_UINT16(1229, Motion::ServoDriver::pulseToDuty(1500));
  TEST_ASSERT_EQUAL_UINT16(410, Motion::ServoDriver::pulseToDuty(500));
  TEST_ASSERT_EQUAL_UINT16(2048, Motion::ServoDriver::pulseToDuty(2500));
}

void test_safety_derate_applies_to_each_actuator_limit(void) {
  TEST_ASSERT_TRUE(near(
      MotionConfig::kTheta0MaxAcc_radps2,
      JointLimits::degToRad(MotionConfig::accelerationLimitDegPerSec2(
          MotionConfig::BASE_MAX_VEL_OUT_DPS))));
  TEST_ASSERT_TRUE(near(
      MotionConfig::kTheta0MaxJerk_radps3,
      JointLimits::degToRad(MotionConfig::jerkLimitDegPerSec3(
          MotionConfig::accelerationLimitDegPerSec2(
              MotionConfig::BASE_MAX_VEL_OUT_DPS)))));

  TEST_ASSERT_TRUE(near(MotionConfig::kTheta1MaxVelDegPerSec,
                        (60.0f / 0.13f) * MotionConfig::kSafetyDerate));
  TEST_ASSERT_TRUE(near(MotionConfig::kTheta2MaxVelDegPerSec,
                        (60.0f / 0.14f) * MotionConfig::kSafetyDerate));
  TEST_ASSERT_TRUE(near(MotionConfig::kTheta3MaxVelDegPerSec,
                        (60.0f / 0.10f) * MotionConfig::kSafetyDerate));
  TEST_ASSERT_TRUE(near(MotionConfig::kGripperMaxVelDegPerSec,
                        (60.0f / 0.10f) * MotionConfig::kSafetyDerate));

  TEST_ASSERT_TRUE(near(
      MotionConfig::kTheta1MaxAccDegPerSec2,
      MotionConfig::accelerationLimitDegPerSec2(MotionConfig::kTheta1MaxVelDegPerSec)));
  TEST_ASSERT_TRUE(near(
      MotionConfig::kTheta2MaxAccDegPerSec2,
      MotionConfig::accelerationLimitDegPerSec2(MotionConfig::kTheta2MaxVelDegPerSec)));
  TEST_ASSERT_TRUE(near(
      MotionConfig::kTheta3MaxAccDegPerSec2,
      MotionConfig::accelerationLimitDegPerSec2(MotionConfig::kTheta3MaxVelDegPerSec)));
  TEST_ASSERT_TRUE(near(
      MotionConfig::kGripperMaxAccDegPerSec2,
      MotionConfig::accelerationLimitDegPerSec2(MotionConfig::kGripperMaxVelDegPerSec)));

  TEST_ASSERT_TRUE(near(
      MotionConfig::kTheta1MaxJerkDegPerSec3,
      MotionConfig::jerkLimitDegPerSec3(MotionConfig::kTheta1MaxAccDegPerSec2)));
  TEST_ASSERT_TRUE(near(
      MotionConfig::kTheta2MaxJerkDegPerSec3,
      MotionConfig::jerkLimitDegPerSec3(MotionConfig::kTheta2MaxAccDegPerSec2)));
  TEST_ASSERT_TRUE(near(
      MotionConfig::kTheta3MaxJerkDegPerSec3,
      MotionConfig::jerkLimitDegPerSec3(MotionConfig::kTheta3MaxAccDegPerSec2)));
  TEST_ASSERT_TRUE(near(
      MotionConfig::kGripperMaxJerkDegPerSec3,
      MotionConfig::jerkLimitDegPerSec3(MotionConfig::kGripperMaxAccDegPerSec2)));
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_stepper_degree_conversion_and_round_trip);
  RUN_TEST(test_servo_pulse_to_duty_conversion);
  RUN_TEST(test_safety_derate_applies_to_each_actuator_limit);
  return UNITY_END();
}
