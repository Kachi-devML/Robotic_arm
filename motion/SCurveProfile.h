#pragma once

// 7-segment jerk-limited (S-curve) motion profile generator.
//
// Pure math — NO hardware dependencies (no GPIO, I2C, FreeRTOS, Arduino).
// Unit-testable on desktop/native.
//
// A symmetric S-curve profile has seven time segments:
//
//   Seg  Jerk   Phase
//   ───  ────   ─────────────────────────────
//    1   +J     acceleration ramp-up
//    2    0     constant max acceleration
//    3   −J     acceleration ramp-down (velocity reaches V)
//    4    0     constant velocity cruise
//    5   −J     deceleration ramp-up
//    6    0     constant max deceleration
//    7   +J     deceleration ramp-down (velocity reaches 0)
//
// Any segment may have zero duration when the corresponding limit
// is not reached (e.g. no cruise when distance is short, no
// constant-acceleration phase when jerk/velocity alone suffice).
//
// Reference: AGENT_BUILD_SPEC §6 (Motion Profiling).

namespace Motion {

struct SCurveProfile {
    static constexpr int kSegments = 7;

    /// State at the boundary between two segments (or profile start/end).
    struct Segment {
        float duration;  // seconds — length of this segment
        float jerk;      // units/s³ — constant jerk during this segment
        float t0;        // seconds — cumulative start time
        float p0;        // units   — position at segment start
        float v0;        // units/s — velocity at segment start
        float a0;        // units/s² — acceleration at segment start
    };

    Segment seg[kSegments];

    float total_duration;  // seconds — sum of all segment durations
    float total_distance;  // units   — requested move distance (always ≥ 0)

    // Peak values actually achieved (may be less than the input limits
    // when distance is short).
    float peak_velocity;      // units/s
    float peak_acceleration;  // units/s²

    // ---- Construction ----

    /// Compute a time-optimal profile for a move of `distance` units,
    /// subject to the given velocity / acceleration / jerk limits.
    ///
    /// All arguments must be ≥ 0.  Returns a zero-duration profile
    /// when distance ≈ 0.
    static SCurveProfile compute(float distance,
                                 float v_max,
                                 float a_max,
                                 float j_max);

    // ---- Evaluation (clamped to [0, total_duration]) ----

    /// Position at absolute time t.
    /// t < 0 → 0 ;  t > total_duration → total_distance.
    float position(float t) const;

    /// Velocity at absolute time t.
    /// Returns 0 outside (0, total_duration).
    float velocity(float t) const;

    /// Acceleration at absolute time t.
    /// Returns 0 outside (0, total_duration).
    float acceleration(float t) const;
};

}  // namespace Motion
