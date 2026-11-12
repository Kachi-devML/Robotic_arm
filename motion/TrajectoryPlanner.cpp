#include "TrajectoryPlanner.h"

#include "MotionConfig.h"

#include <cmath>

namespace Motion {

namespace {

constexpr float kEps = 1e-12f;

float payloadScale(PayloadProfile profile) {
    return (profile == PayloadProfile::Loaded) ? MotionConfig::kLoadedDerate
                                               : 1.0f;
}

void jointLimits(int i, float scale, float& v, float& a, float& j) {
    switch (i) {
        case 0:
            v = MotionConfig::kTheta0MaxVel_radps * scale;
            a = MotionConfig::kTheta0MaxAcc_radps2 * scale;
            j = MotionConfig::kTheta0MaxJerk_radps3 * scale;
            break;
        case 1:
            v = MotionConfig::kTheta1MaxVel_radps * scale;
            a = MotionConfig::kTheta1MaxAcc_radps2 * scale;
            j = MotionConfig::kTheta1MaxJerk_radps3 * scale;
            break;
        case 2:
            v = MotionConfig::kTheta2MaxVel_radps * scale;
            a = MotionConfig::kTheta2MaxAcc_radps2 * scale;
            j = MotionConfig::kTheta2MaxJerk_radps3 * scale;
            break;
        case 3:
            v = MotionConfig::kTheta3MaxVel_radps * scale;
            a = MotionConfig::kTheta3MaxAcc_radps2 * scale;
            j = MotionConfig::kTheta3MaxJerk_radps3 * scale;
            break;
        default:
            v = MotionConfig::kGripperMaxVel_radps * scale;
            a = MotionConfig::kGripperMaxAcc_radps2 * scale;
            j = MotionConfig::kGripperMaxJerk_radps3 * scale;
            break;
    }
}

}  // namespace

TrajectoryPlan TrajectoryPlanner::plan(const JointVector& start,
                                       const JointVector& end,
                                       PayloadProfile profile) {
    TrajectoryPlan plan{};
    plan.profile = profile;
    plan.duration = 0.0f;
    plan.limiting_joint = 0;

    const float scale = payloadScale(profile);

    float s_vmax = 0.0f;
    float s_amax = 0.0f;
    float s_jmax = 0.0f;
    bool any_motion = false;
    float max_T = 0.0f;
    int limiting = 0;

    for (int i = 0; i < kJointCount; ++i) {
        JointPlan& jp = plan.joints[i];
        jp.start = jointAt(start, i);
        jp.end = jointAt(end, i);
        jp.delta = jp.end - jp.start;
        jp.distance = std::fabs(jp.delta);
        jointLimits(i, scale, jp.v_max, jp.a_max, jp.j_max);

        const SCurveProfile solo =
            SCurveProfile::compute(jp.distance, jp.v_max, jp.a_max, jp.j_max);
        jp.time_optimal = solo.total_duration;

        if (jp.time_optimal > max_T) {
            max_T = jp.time_optimal;
            limiting = i;
        }

        if (jp.distance > kEps) {
            const float nv = jp.v_max / jp.distance;
            const float na = jp.a_max / jp.distance;
            const float nj = jp.j_max / jp.distance;
            if (!any_motion) {
                s_vmax = nv;
                s_amax = na;
                s_jmax = nj;
                any_motion = true;
            } else {
                s_vmax = std::fmin(s_vmax, nv);
                s_amax = std::fmin(s_amax, na);
                s_jmax = std::fmin(s_jmax, nj);
            }
        }
    }

    plan.limiting_joint = limiting;

    if (!any_motion) {
        plan.shape = SCurveProfile::compute(0.0f, 1.0f, 1.0f, 1.0f);
        plan.duration = 0.0f;
        return plan;
    }

    // Unit-distance profile: position(t) is s(t) ∈ [0, 1].
    plan.shape = SCurveProfile::compute(1.0f, s_vmax, s_amax, s_jmax);
    plan.duration = plan.shape.total_duration;
    return plan;
}

float TrajectoryPlan::progress(float t) const {
    if (duration <= kEps) {
        return 1.0f;
    }
    return shape.position(t);
}

JointVector TrajectoryPlan::anglesAt(float t) const {
    const float s = progress(t);
    JointVector out{};
    for (int i = 0; i < kJointCount; ++i) {
        setJoint(out, i, joints[i].start + s * joints[i].delta);
    }
    return out;
}

JointVector TrajectoryPlan::velocitiesAt(float t) const {
    const float s_dot = (duration <= kEps) ? 0.0f : shape.velocity(t);
    JointVector out{};
    for (int i = 0; i < kJointCount; ++i) {
        setJoint(out, i, s_dot * joints[i].delta);
    }
    return out;
}

JointVector TrajectoryPlan::accelerationsAt(float t) const {
    const float s_ddot = (duration <= kEps) ? 0.0f : shape.acceleration(t);
    JointVector out{};
    for (int i = 0; i < kJointCount; ++i) {
        setJoint(out, i, s_ddot * joints[i].delta);
    }
    return out;
}

}  // namespace Motion
