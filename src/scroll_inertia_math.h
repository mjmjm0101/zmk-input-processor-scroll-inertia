/*
 * Pure math helpers for the scroll-inertia input processor.
 *
 * Split out from the main driver so the numeric invariants (EMA,
 * peak update, decay, fixed-point accumulate) can be unit-tested on
 * the host without pulling in Zephyr or ZMK.  The state machine and
 * anything that touches Zephyr APIs lives in the driver .c file.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZMK_INPUT_PROCESSOR_SCROLL_INERTIA_MATH_H
#define ZMK_INPUT_PROCESSOR_SCROLL_INERTIA_MATH_H

#include <stdbool.h>
#include <stdint.h>

/* Fixed-point: 8-bit fractional part (×256) */
#define FP_SHIFT 8
#define FP_SCALE (1 << FP_SHIFT)

/* Configuration (read from Devicetree at compile time in the driver,
 * populated directly in host tests).  POD-only so it can be brace-
 * initialised and compared in tests.
 *
 * Advanced-tuning constants (decel-samples, gesture-timeout,
 * decel-ratio, suppress-limit, peak-decay, min-events) are exposed
 * as Devicetree properties; see dts/bindings/...yaml for defaults
 * and recommended ranges. */
struct scroll_inertia_config {
    int32_t gain;
    int32_t blend;

    int32_t start_fp;
    int32_t move;
    int32_t release_ms;

    int32_t fast_fp;
    int32_t decay_fast;
    int32_t decay_slow;
    int32_t slow_fp;
    int32_t decay_tail;
    int32_t friction_fp;
    int32_t stop_fp;

    int32_t scale;
    int32_t scale_div;
    int32_t limit_fp;
    int32_t span_ms;
    int32_t tick_ms;

    int32_t axis;
    int32_t layer;
    int32_t swap_mod;
    int32_t unlock_mod;

    int32_t min_events;
    int32_t decel_samples;
    int32_t decel_ratio;
    int32_t peak_decay;
    int32_t gesture_timeout_ms;
    int32_t suppress_limit;
    int32_t handoff_ms;
};

static inline int32_t abs32(int32_t v) { return v < 0 ? -v : v; }

/* Integer approximation of sqrt(a² + b²).
 * Uses max(|a|,|b|) + min(|a|,|b|)/2  (max error ~12%). */
static inline int32_t fast_magnitude(int32_t a, int32_t b) {
    int32_t aa = abs32(a);
    int32_t bb = abs32(b);
    int32_t hi = aa > bb ? aa : bb;
    int32_t lo = aa > bb ? bb : aa;
    return hi + (lo >> 1);
}

static inline int32_t clamp_velocity(int32_t vel, int32_t limit_fp) {
    if (vel > limit_fp)  return limit_fp;
    if (vel < -limit_fp) return -limit_fp;
    return vel;
}

/* Defensive guards against pathological DT values.  We don't try to
 * "auto-correct" every odd setting (that would silently mask user
 * mistakes); we only protect against the two values that would crash
 * or busy-loop: a zero divisor and a zero tick interval. */
static inline int32_t safe_scale_div(int32_t v) { return v > 0 ? v : 1; }
static inline int32_t safe_tick_ms(int32_t v)   { return v > 0 ? v : 1; }

/* ------------------------------------------------------------------ */
/* Per-axis math operations                                            */
/* ------------------------------------------------------------------
 * Shared math used by the Y and X paths in TRACKING, cross-axis, and
 * COASTING.  All fixed-point conversions use `* FP_SCALE` rather than
 * `<< FP_SHIFT` so negative deltas don't invoke undefined behaviour on
 * strict toolchains.
 * ------------------------------------------------------------------ */

/* EMA velocity update: vel = (delta * gain + vel * blend) / 1000, clamped. */
static inline int32_t update_velocity_ema(int32_t raw_value, int32_t vel,
                                          const struct scroll_inertia_config *cfg) {
    int32_t delta_fp = raw_value * FP_SCALE;
    int32_t updated = ((int64_t)delta_fp * cfg->gain +
                       (int64_t)vel * cfg->blend) / 1000;
    return clamp_velocity(updated, cfg->limit_fp);
}

/* Did the velocity flip direction relative to the peak?  Guarded
 * against zero on either side so a truncation-to-zero isn't misread as
 * a reversal. */
static inline bool peak_reversed(int32_t vel, int32_t peak) {
    return peak != 0 && vel != 0 && (vel > 0) != (peak > 0);
}

/* Peak update: on direction reversal, snap to the new velocity.  Same
 * direction, hold the peak while the EMA is below it, but let the
 * peak drift toward the EMA so a transient spike doesn't leave an
 * inflated peak forever. */
static inline int32_t update_peak(int32_t vel, int32_t peak,
                                  const struct scroll_inertia_config *cfg) {
    if (peak_reversed(vel, peak)) return vel;
    if (abs32(vel) > abs32(peak)) return vel;
    int32_t decayed = (int64_t)peak * cfg->peak_decay / 1000;
    return abs32(decayed) > abs32(vel) ? decayed : vel;
}

/* One tick of velocity decay: three-stage multiplicative (decay_fast /
 * decay_slow / decay_tail selected by the fast_fp and slow_fp
 * boundaries), then additive Coulomb friction.
 *
 * `commit_permille` (0..1000) and `init_vel` together shape an extra,
 * velocity-tapered loss on top of the baseline multiplicative decay.
 *
 *     taper_lin = max(0, |vel| - stop_fp) / max(init_vel - stop_fp, 1)
 *     taper     = taper_lin²   (weights the effect toward the top)
 *     full_extra = base_loss × (1000 - commit_permille) / commit_permille
 *     scaled_loss = base_loss + full_extra × taper
 *
 * The squared taper concentrates the commit-based acceleration in the
 * upper speed band: at the top of the coast (|vel| ≈ init_vel) taper
 * ≈ 1 and weakly-committed flicks burn through the initial phase
 * quickly; in the mid speed band the effect drops off steeply so the
 * coast retains the velocity it needs for smooth accumulate_and_emit
 * output; as vel approaches stop_fp, taper → 0 and the loss returns
 * to the baseline rate.  Stopping behaviour stays identical
 * regardless of commitment — only the upper band shrinks.
 *
 * Pass commit_permille = 1000 (or init_vel = 0) for the unscaled
 * baseline decay (host tests and AXIS_BOTH fallback). */
static inline void apply_decay(int32_t *vel,
                               const struct scroll_inertia_config *cfg,
                               int32_t commit_permille,
                               int32_t init_vel) {
    int32_t av = abs32(*vel);
    int32_t d = av > cfg->fast_fp ? cfg->decay_fast
              : av > cfg->slow_fp ? cfg->decay_slow
              :                     cfg->decay_tail;
    if (commit_permille > 0 && commit_permille < 1000 && init_vel > cfg->stop_fp) {
        int32_t vel_above = av > cfg->stop_fp ? av - cfg->stop_fp : 0;
        int32_t init_above = init_vel - cfg->stop_fp;
        int32_t taper = (int64_t)vel_above * 1000 / init_above;
        if (taper > 1000) taper = 1000;
        taper = (int64_t)taper * taper / 1000;
        int32_t base_loss = 1000 - d;
        int32_t full_extra = (int64_t)base_loss * (1000 - commit_permille)
                             / commit_permille;
        int32_t extra = (int64_t)full_extra * taper / 1000;
        int32_t scaled_loss = base_loss + extra;
        if (scaled_loss > 1000) scaled_loss = 1000;
        d = 1000 - scaled_loss;
    }
    *vel = (int64_t)(*vel) * d / 1000;
    if (cfg->friction_fp > 0) {
        if (*vel > cfg->friction_fp)       *vel -= cfg->friction_fp;
        else if (*vel < -cfg->friction_fp) *vel += cfg->friction_fp;
        else                                *vel = 0;
    }
}

/* Accumulate sub-unit velocity into *accum and emit the whole-unit
 * delta, keeping the fractional remainder in *accum so slow tails
 * still tick out discrete scroll events. */
static inline int16_t accumulate_and_emit(int32_t vel, int32_t *accum,
                                          const struct scroll_inertia_config *cfg,
                                          int32_t sdiv) {
    *accum += (int64_t)vel * cfg->scale / sdiv;
    int16_t emit = (int16_t)(*accum >> FP_SHIFT);
    *accum -= (int32_t)emit * FP_SCALE;
    return emit;
}

#endif /* ZMK_INPUT_PROCESSOR_SCROLL_INERTIA_MATH_H */
