#pragma once

// Link lengths, power-on/neutral poses, and firmware-enforced soft limits.
// Internal angle math is radians. Degrees exist only as named sources for
// those radians (PCA9685 pulse mapping and JSON happen at the driver/schema
// boundary, not here).

namespace JointLimits {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;

constexpr float degToRad(float deg) { return deg * kDegToRad; }
constexpr float radToDeg(float rad) { return rad * kRadToDeg; }

// IK solutions within this margin are accepted and clamped to the soft limit.
constexpr float kSoftLimitTolerance_deg = 0.5f;

// Revision 3 §3.6: map elbow and wrist IK angles to joint angles.
constexpr float kElbowSign = -1.0f;
constexpr float kWristSign = -1.0f;

// Mechanical links (cm). Targets (x, y, z) are fingertip position, not wrist center.
constexpr float kL1_cm = 15.0f;  // shoulder → elbow
constexpr float kL2_cm = 12.0f;  // elbow → wrist
constexpr float kL3_cm = 8.0f;   // wrist → fingertip (3.5 + 4.5 cm treated as one rigid segment)
constexpr float kD1_cm = 12.0f;  // ground → shoulder axis

// θ0 base: mechanical travel is a full turn, but wiring has no confirmed slip
// ring — soft-limit ±170° from the power-on reference so cables cannot wind up.
// Widen kTheta0SoftAbsDeg in this file only if a slip ring is added.
constexpr float kTheta0SoftAbsDeg = 170.0f;
constexpr float kTheta0SoftMin_rad = degToRad(-kTheta0SoftAbsDeg);
constexpr float kTheta0SoftMax_rad = degToRad(kTheta0SoftAbsDeg);
constexpr float kTheta0Home_rad = degToRad(0.0f);  // 0° = power-on reference

// θ1 shoulder, θ2 elbow, θ3 wrist: 180° servos, power-on/neutral = 90°.
constexpr float kTheta1Min_rad = degToRad(0.0f);
constexpr float kTheta1Max_rad = degToRad(180.0f);
constexpr float kTheta1Home_rad = degToRad(90.0f);  // power-on neutral: vertical / upright

constexpr float kTheta2Min_rad = degToRad(0.0f);
constexpr float kTheta2Max_rad = degToRad(180.0f);
constexpr float kTheta2Home_rad = degToRad(90.0f);  // power-on neutral: folded, compact/safe

constexpr float kTheta3Min_rad = degToRad(0.0f);
constexpr float kTheta3Max_rad = degToRad(180.0f);
constexpr float kTheta3Home_rad = degToRad(90.0f);  // power-on neutral: level

// Gripper: 0° open, 90° closed. Power-on pose = open. Not a positioning DoF.
constexpr float kGripperOpen_rad = degToRad(0.0f);
constexpr float kGripperClosed_rad = degToRad(90.0f);
constexpr float kGripperHome_rad = kGripperOpen_rad;  // power-on neutral: open

}  // namespace JointLimits
