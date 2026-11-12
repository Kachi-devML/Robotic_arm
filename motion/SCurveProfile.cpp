#include "SCurveProfile.h"

#include <cmath>

namespace Motion {

namespace {

/// Find the index of the segment containing time t.
/// Assumes t is within [0, total_duration].
int findSegment(const SCurveProfile& p, float t) {
    for (int i = 0; i < SCurveProfile::kSegments - 1; ++i) {
        if (t < p.seg[i].t0 + p.seg[i].duration) return i;
    }
    return SCurveProfile::kSegments - 1;
}

}  // namespace

// ============================================================
//  Profile computation
// ============================================================
//
// Given distance D, max velocity V, max acceleration A, max jerk J,
// the algorithm determines the seven segment durations:
//
//   Tj = jerk-phase duration  (segments 1,3,5,7 — symmetric)
//   Ta = constant-accel duration (segments 2,6 — symmetric)
//   Tv = cruise duration (segment 4)
//
// Three cases arise:
//   Case 1 — All limits reached: cruise phase exists.
//   Case 2 — V reduced (no cruise), A still reached.
//   Case 3 — Both V and A reduced (no cruise, short distance).
// ============================================================

SCurveProfile SCurveProfile::compute(float distance,
                                     float v_max,
                                     float a_max,
                                     float j_max) {
    SCurveProfile prof{};
    prof.total_distance = distance;

    constexpr float kEps = 1e-12f;

    if (distance < kEps || v_max < kEps || a_max < kEps || j_max < kEps) {
        // Zero-move profile — all segments have zero duration.
        prof.total_duration   = 0.0f;
        prof.peak_velocity    = 0.0f;
        prof.peak_acceleration = 0.0f;
        return prof;
    }

    float Tj, Ta, Tv;
    float V;  // actual peak velocity achieved
    const float J = j_max;

    // ── Phase 1: compute Tj, Ta assuming full Vmax is achievable ───

    if (v_max * j_max >= a_max * a_max) {
        // Acceleration limit IS reached during the jerk ramp
        Tj = a_max / j_max;
        Ta = v_max / a_max - Tj;
    } else {
        // Velocity is reached before acceleration hits Amax
        Tj = std::sqrt(v_max / j_max);
        Ta = 0.0f;
    }

    // ── Phase 2: check whether a cruise phase exists ──────────────

    float T_accel = 2.0f * Tj + Ta;
    V = v_max;

    // Distance covered during accel = decel phase:
    //   Da = ½ · V · T_accel   (exact for symmetric S-curve)
    float Da = 0.5f * V * T_accel;

    if (2.0f * Da <= distance + kEps) {
        // ── Case 1: cruise phase exists ──
        Tv = (distance - 2.0f * Da) / V;
        if (Tv < 0.0f) Tv = 0.0f;  // numerical guard
    } else {
        // ── No cruise — must reduce velocity ──
        Tv = 0.0f;

        // Sub-case A: try keeping Amax reached.
        //   D = V' · (Tj + V'/A)   →  V'²/A + V'·Tj − D = 0
        const float Tj_full = a_max / j_max;
        const float disc    = a_max * a_max * Tj_full * Tj_full
                            + 4.0f * a_max * distance;
        const float V_try   = (-a_max * Tj_full + std::sqrt(disc)) * 0.5f;

        if (V_try * j_max >= a_max * a_max) {
            // Amax is still reached with the reduced velocity
            V  = V_try;
            Tj = Tj_full;
            Ta = V / a_max - Tj;
            if (Ta < 0.0f) Ta = 0.0f;
        } else {
            // Sub-case B: Amax also not reached.
            //   D = 2 · V'^(3/2) / √J   →  V' = (D·√J / 2)^(2/3)
            Ta = 0.0f;
            V  = std::pow(distance * std::sqrt(j_max) * 0.5f,
                          2.0f / 3.0f);
            Tj = std::sqrt(V / j_max);
        }

        T_accel = 2.0f * Tj + Ta;
    }

    // Actual peak acceleration = J · Tj
    const float A_peak = J * Tj;

    prof.peak_velocity     = V;
    prof.peak_acceleration = A_peak;

    // ── Build the 7 segments ──────────────────────────────────────
    //
    // Symmetric layout:
    //   seg 0: accel ramp-up   (+J,  Tj)
    //   seg 1: const accel     ( 0,  Ta)
    //   seg 2: accel ramp-down (−J,  Tj)
    //   seg 3: cruise          ( 0,  Tv)
    //   seg 4: decel ramp-up   (−J,  Tj)
    //   seg 5: const decel     ( 0,  Ta)
    //   seg 6: decel ramp-down (+J,  Tj)

    const float durations[kSegments] = {Tj, Ta, Tj,  Tv,  Tj, Ta, Tj};
    const float jerks[kSegments]     = { J,  0, -J, 0.0f, -J,  0,  J};

    float t_cum = 0.0f;
    float p = 0.0f, v = 0.0f, a = 0.0f;

    for (int i = 0; i < kSegments; ++i) {
        prof.seg[i].duration = durations[i];
        prof.seg[i].jerk     = jerks[i];
        prof.seg[i].t0       = t_cum;
        prof.seg[i].p0       = p;
        prof.seg[i].v0       = v;
        prof.seg[i].a0       = a;

        const float dt = durations[i];
        const float ji = jerks[i];

        // Advance state to the END of this segment using the
        // constant-jerk kinematic equations:
        //   p += v·dt + ½·a·dt² + ⅙·j·dt³
        //   v += a·dt + ½·j·dt²
        //   a += j·dt
        p += v * dt + 0.5f * a * dt * dt
           + (1.0f / 6.0f) * ji * dt * dt * dt;
        v += a * dt + 0.5f * ji * dt * dt;
        a += ji * dt;

        t_cum += dt;
    }

    prof.total_duration = t_cum;
    return prof;
}

// ============================================================
//  Evaluation helpers
// ============================================================

float SCurveProfile::position(float t) const {
    if (t <= 0.0f)           return 0.0f;
    if (t >= total_duration) return total_distance;

    const int   s   = findSegment(*this, t);
    const float tau = t - seg[s].t0;

    return seg[s].p0
         + seg[s].v0 * tau
         + 0.5f * seg[s].a0 * tau * tau
         + (1.0f / 6.0f) * seg[s].jerk * tau * tau * tau;
}

float SCurveProfile::velocity(float t) const {
    if (t <= 0.0f || t >= total_duration) return 0.0f;

    const int   s   = findSegment(*this, t);
    const float tau = t - seg[s].t0;

    return seg[s].v0
         + seg[s].a0 * tau
         + 0.5f * seg[s].jerk * tau * tau;
}

float SCurveProfile::acceleration(float t) const {
    if (t <= 0.0f || t >= total_duration) return 0.0f;

    const int   s   = findSegment(*this, t);
    const float tau = t - seg[s].t0;

    return seg[s].a0 + seg[s].jerk * tau;
}

}  // namespace Motion
