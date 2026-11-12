#include "JointLimits.h"
#include "Logger.h"
#include "MotionConfig.h"
#include "Pins.h"

constexpr bool near(float actual, float expected) {
  return actual > expected - 0.0001f && actual < expected + 0.0001f;
}

static_assert(Pins::kStatusLed == 2);
static_assert(Pins::kNemaStep == 10);
static_assert(Pins::kNemaDir == 11);
static_assert(Pins::kNemaEn == 12);
static_assert(Pins::kNemaM0 == 13);
static_assert(Pins::kNemaM1 == 14);
static_assert(Pins::kNemaM2 == 15);
static_assert(Pins::kHomeSw == 16);
static_assert(Pins::kEstopSw == 17);
static_assert(Pins::kServoShoulderPwm == 4);
static_assert(Pins::kServoElbowPwm == 5);
static_assert(Pins::kServoWristPwm == 6);
static_assert(Pins::kServoGripperPwm == 7);

static_assert(near(JointLimits::kL1_cm, 15.0f));
static_assert(near(JointLimits::kL2_cm, 12.0f));
static_assert(near(JointLimits::kL3_cm, 8.0f));
static_assert(near(JointLimits::kD1_cm, 12.0f));
static_assert(near(JointLimits::kTheta0SoftAbsDeg, 170.0f));
static_assert(near(JointLimits::kTheta0SoftMin_rad, JointLimits::degToRad(-170.0f)));
static_assert(near(JointLimits::kTheta0SoftMax_rad, JointLimits::degToRad(170.0f)));
static_assert(near(JointLimits::kTheta0Home_rad, 0.0f));
static_assert(near(JointLimits::kTheta1Min_rad, JointLimits::degToRad(0.0f)));
static_assert(near(JointLimits::kTheta1Max_rad, JointLimits::degToRad(180.0f)));
static_assert(near(JointLimits::kTheta1Home_rad, JointLimits::degToRad(90.0f)));
static_assert(near(JointLimits::kTheta2Min_rad, JointLimits::degToRad(0.0f)));
static_assert(near(JointLimits::kTheta2Max_rad, JointLimits::degToRad(180.0f)));
static_assert(near(JointLimits::kTheta2Home_rad, JointLimits::degToRad(90.0f)));
static_assert(near(JointLimits::kTheta3Min_rad, JointLimits::degToRad(0.0f)));
static_assert(near(JointLimits::kTheta3Max_rad, JointLimits::degToRad(180.0f)));
static_assert(near(JointLimits::kTheta3Home_rad, JointLimits::degToRad(90.0f)));
static_assert(near(JointLimits::kGripperOpen_rad, JointLimits::degToRad(0.0f)));
static_assert(near(JointLimits::kGripperClosed_rad, JointLimits::degToRad(90.0f)));
static_assert(near(JointLimits::kGripperHome_rad, JointLimits::degToRad(0.0f)));
static_assert(near(MotionConfig::kSafetyDerate, 0.55f));
static_assert(near(MotionConfig::kLoadedDerate, 0.50f));
static_assert(near(MotionConfig::STEPPER_GEAR_RATIO, 5.0f));
static_assert(MotionConfig::STEPPER_MICROSTEP_DIV == 16);
static_assert(near(MotionConfig::STEPPER_STEPS_PER_OUTPUT_DEG, 8000.0f / 180.0f));

int sideEffectCount = 0;

int main() {
  LOG_DEBUG("debug %d", ++sideEffectCount);
  LOG_INFO("info %d", ++sideEffectCount);
  LOG_WARN("warn %d", ++sideEffectCount);
  LOG_ERROR("error %d", ++sideEffectCount);
  return sideEffectCount;
}
