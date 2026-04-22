/*
 * ZMK Input Processor: Scroll Inertia
 *
 * Adds iOS-like inertial scrolling to trackball scroll mode.
 *
 * The processor monitors scroll velocity, and once a flick is detected,
 * takes over at the moment deceleration begins — suppressing the
 * trackball's natural slowdown and replacing it with a smooth,
 * configurable exponential decay curve.
 *
 * Designed to be instantiated per-axis: one instance for vertical scroll
 * (axis=1) and another for horizontal (axis=2), each in its own ZMK
 * input_listener scroller block bound to its own layer.  See README.md
 * for the rationale and recommended placement.
 *
 * Architecture
 * ------------
 * Each instance is modelled as an explicit 3-state machine:
 *
 *     IDLE  ──▶  TRACKING  ──▶  COASTING  ──▶  IDLE
 *
 * Transitions:
 *   IDLE → TRACKING      first tracked-axis event
 *   TRACKING → COASTING  peak magnitude ≥ start, movement ≥ move,
 *                        ≥ min-events, then deceleration
 *                        confirmed (or stop_detect fallback)
 *   COASTING → TRACKING  reverse direction, cross-axis break, or
 *                        suppress-limit same-dir absorbs
 *   COASTING → IDLE      vel < stop, span exceeded, layer off
 *   any → IDLE           gesture timeout, or stale inertia
 *                        (see should_reset_on_timeout)
 *
 * Only one state is active at a time.  Every event is classified
 * (untracked / cross-axis / tracked-axis), then dispatched to the
 * current state's handler.  State transitions are explicit function
 * calls (to_idle / to_tracking_from_* / to_coasting) so the code can
 * be audited by grepping for them.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_scroll_inertia

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>

#include <drivers/input_processor.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>

#include "scroll_inertia_math.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* Axis mode constants */
#define AXIS_BOTH 0
#define AXIS_Y    1
#define AXIS_X    2

/* ------------------------------------------------------------------
 * Concurrency model
 * ------------------------------------------------------------------
 * All three handlers below
 *   - scroll_inertia_handle_event() (input subsystem callback)
 *   - inertia_tick_handler()        (k_work_delayable)
 *   - stop_detect_handler()         (k_work_delayable)
 * are normally serialised on the Zephyr system workqueue under the
 * standard ZMK input-listener path, so they cannot preempt each other
 * and the per-instance `scroll_inertia_data` is accessed from a single
 * thread of control at a time.
 *
 * Should a downstream integration ever route input events from a
 * different context (ISR or driver thread), most fields are int32_t
 * which are single-instruction loads/stores on Cortex-M and therefore
 * naturally atomic; the worst observable effect would be one tick
 * working from a value that's one update old.  We deliberately keep
 * the implementation lock-free for simplicity and predictability.
 * ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* State machine                                                       */
/* ------------------------------------------------------------------ */

enum scroll_state {
    SS_IDLE,        /* no gesture active                         */
    SS_TRACKING,    /* building velocity / peak for arming       */
    SS_COASTING,    /* inertia tick is emitting decaying scroll  */
};

enum event_class {
    EC_UNTRACKED,   /* not a REL scroll event — pass straight through */
    EC_CROSS_AXIS,  /* single-axis mode, event is on the other axis */
    EC_TRACKED,     /* scroll event on a tracked axis (zero values
                     * are still classified as EC_TRACKED and filtered
                     * later as idle keep-alives) */
};

/* struct scroll_inertia_config lives in scroll_inertia_math.h so
 * host-side tests can populate it directly without Zephyr headers. */

/* ------------------------------------------------------------------ */
/* Runtime state                                                       */
/* ------------------------------------------------------------------ */

struct scroll_inertia_data {
    const struct device *dev;

    enum scroll_state state;

    /* --- TRACKING fields --- */

    /* Velocity EMA in fixed-point (×256).  Updated each event from
     * the raw delta in TRACKING.  Inherited into COASTING (where the
     * tick handler decays it) and, in single-axis mode, reseeded to
     * the current vector-magnitude EMA on the TRACKING → COASTING
     * transition so diagonal flicks coast at the same strength as
     * axis-aligned flicks of the same physical vector magnitude.
     * Using the current EMA (not the peak) keeps the coast
     * continuous with the tracking output — no acceleration jump
     * at handoff.  Cleared when TRACKING starts. */
    int32_t vel_x;
    int32_t vel_y;

    /* Peak velocity observed in the current gesture (fixed-point) */
    int32_t peak_vel_x;
    int32_t peak_vel_y;

    /* Cumulative |delta| within the current gesture */
    int32_t total_movement;

    /* Consecutive sub-peak samples seen so far */
    int32_t decel_count;

    /* Events since the last reset (warmup counter) */
    int32_t tracking_count;

    /* Buffered non-tracked-axis value, awaiting merge into the
     * tracked axis via fast_magnitude().  Single slot by design:
     * input devices report X and Y in pairs, so the buffered value is
     * normally consumed by the very next event of the tracked axis.
     * If the same non-tracked axis fires twice in a row (rare), the
     * older value is overwritten — accepted as a small magnitude loss
     * rather than carrying a queue. */
    int32_t pending_other;

    /* --- COASTING fields --- */

    /* Sub-unit scroll accumulators */
    int32_t accum_x;
    int32_t accum_y;

    int64_t inertia_start_time;

    /* Consecutive same-direction events absorbed in COASTING.  Reset
     * on any cross-axis or state change.  Hitting suppress-limit
     * transitions back to TRACKING on the assumption that the user
     * has been actively rolling past a stale coast. */
    int32_t suppress_count;

    /* Cumulative |delta| of cross-axis input seen while in COASTING.
     * In single-axis modes, cross-axis events early-return with their
     * value zeroed; if the user is vigorously rolling the non-tracked
     * axis, this counter grows.  When it exceeds `move`, inertia is
     * cancelled so the user regains direct control.  Cleared whenever
     * a tracked-axis event is absorbed, so diagonal post-flick coasts
     * (interleaved tracked/cross events) don't accidentally trigger
     * the break. */
    int32_t cross_axis_accumulated;

    /* --- Global timing --- */

    /* Timestamp of the last event (for gesture-timeout detection) */
    int64_t last_event_time;

    /* Set true when leaving COASTING (reverse-flick, cross-axis break,
     * suppress-limit, stale-inertia, or natural decay).  While true,
     * the next arm skips the min-events gate — we've already witnessed
     * a committed gesture, so the user's follow-up flick doesn't need
     * to re-prove itself against the noise filter.  Cleared when the
     * next arm consumes it (to_coasting) or when we fully reset via
     * to_idle from any non-COASTING state. */
    bool post_coast;

    /* Gesture commitment captured at to_coasting, in permille (0..1000).
     * Used to scale the per-tick decay rate: weaker commitment makes
     * the coast decay faster so a barely-armed flick doesn't sustain
     * the high-velocity phase as long as a long, committed flick. */
    int32_t commit_permille;

    /* Per-axis initial coast velocity magnitude captured at to_coasting
     * (absolute value, after any limit clamping).  Used as the taper
     * reference in apply_decay so the commit-based extra loss is at
     * full strength near the top of the coast (taper ≈ 1.0) and tapers
     * to zero as that axis approaches stop_fp.
     *
     * Per-axis (not a single magnitude) so each axis's taper starts at
     * 1.0 in AXIS_BOTH too: a diagonal coast where vel_y ≈ vel_x ≈ V
     * and fast_magnitude ≈ 1.5V would otherwise leave each axis's
     * taper at V / 1.5V ≈ 0.67, under-applying the commit-based loss
     * on weakly-committed diagonal flicks. */
    int32_t init_vel_y;
    int32_t init_vel_x;

    /* Delayed work items */
    struct k_work_delayable stop_detect_work;
    struct k_work_delayable inertia_tick_work;
};

/* ------------------------------------------------------------------ */
/* Helpers (Zephyr/ZMK-dependent — pure math lives in the header)      */
/* ------------------------------------------------------------------ */

/* Returns true if any of the configured swap-modifier bits are held. */
static inline bool swap_active(const struct scroll_inertia_config *cfg) {
    if (cfg->swap_mod == 0) return false;
    return (zmk_hid_get_explicit_mods() & cfg->swap_mod) != 0;
}

/* Returns true if any of the configured unlock-modifier bits are held. */
static inline bool unlock_active(const struct scroll_inertia_config *cfg) {
    if (cfg->unlock_mod == 0) return false;
    return (zmk_hid_get_explicit_mods() & cfg->unlock_mod) != 0;
}

/* The axis to use right now.  When unlock-mod is held, the configured
 * axis is overridden to AXIS_BOTH so both axes flow freely. */
static inline int32_t effective_axis(const struct scroll_inertia_config *cfg) {
    return unlock_active(cfg) ? AXIS_BOTH : cfg->axis;
}

/* Full per-tracked-axis update: EMA + total_movement + peak, and on a
 * direction reversal reset the gesture-wide counters (decel_count,
 * total_movement, tracking_count).  Lives here (not in the math
 * header) because it mutates scroll_inertia_data, which depends on
 * Zephyr types.  Cross-axis updates don't reset gesture counters, so
 * they call update_velocity_ema / update_peak from the header
 * directly. */
static inline void update_tracked_axis(struct scroll_inertia_data *data,
                                       const struct scroll_inertia_config *cfg,
                                       int32_t raw_value,
                                       int32_t *vel, int32_t *peak) {
    int32_t prev_peak = *peak;
    *vel = update_velocity_ema(raw_value, *vel, cfg);
    data->total_movement += abs32(raw_value);
    bool reversed = peak_reversed(*vel, prev_peak);
    *peak = update_peak(*vel, prev_peak, cfg);
    if (reversed) {
        data->decel_count = 0;
        data->total_movement = abs32(raw_value);
        data->tracking_count = 1;
    }
}

/* ------------------------------------------------------------------ */
/* State transitions                                                   */
/* ------------------------------------------------------------------
 * These are the only functions that assign to data->state.  All other
 * code goes through them.  Each transition knows exactly which fields
 * to clear for the destination state; callers don't need to remember.
 * ------------------------------------------------------------------ */

static void clear_tracking_fields(struct scroll_inertia_data *data) {
    data->vel_x = 0;
    data->vel_y = 0;
    data->peak_vel_x = 0;
    data->peak_vel_y = 0;
    data->total_movement = 0;
    data->decel_count = 0;
    data->tracking_count = 0;
    data->pending_other = 0;
}

static void clear_coasting_fields(struct scroll_inertia_data *data) {
    data->accum_x = 0;
    data->accum_y = 0;
    data->suppress_count = 0;
    data->cross_axis_accumulated = 0;
}

static void to_idle(struct scroll_inertia_data *data) {
    bool was_coasting = (data->state == SS_COASTING);
    if (was_coasting) {
        k_work_cancel_delayable(&data->inertia_tick_work);
    }
    k_work_cancel_delayable(&data->stop_detect_work);
    clear_tracking_fields(data);
    clear_coasting_fields(data);
    /* Preserve the post-coast signal only when we're actually coming
     * from a real coast (e.g., stale-inertia reset from a live coast).
     * A to_idle from TRACKING or from IDLE itself means any prior
     * post-coast credit is spent without arming — clear it. */
    data->post_coast = was_coasting;
    data->state = SS_IDLE;
}

static void to_tracking_from_idle(struct scroll_inertia_data *data) {
    /* IDLE already has tracking fields cleared, but clear again to be
     * safe — cheap, and defends against a stray IDLE entry where only
     * part of the state was reset. */
    clear_tracking_fields(data);
    data->state = SS_TRACKING;
}

static void to_tracking_from_coasting(struct scroll_inertia_data *data) {
    k_work_cancel_delayable(&data->inertia_tick_work);
    clear_coasting_fields(data);
    clear_tracking_fields(data);
    /* We just witnessed a committed gesture; let the next arm skip
     * the min-events gate so rapid reversal flicks can coast without
     * accumulating 10 events per direction. */
    data->post_coast = true;
    data->state = SS_TRACKING;
}

static void to_coasting(struct scroll_inertia_data *data,
                       const struct scroll_inertia_config *cfg) {
    /* Velocity is inherited from TRACKING — that's the whole point.
     * Only clear coasting-specific fields and schedule the tick. */
    clear_coasting_fields(data);
    data->inertia_start_time = k_uptime_get();
    /* post-coast credit consumed by this arm. */
    data->post_coast = false;

    /* Angle-invariant initial inertia strength: in a single-axis mode,
     * fold the vector magnitude of the gesture's *current* EMA into
     * the tracked axis.  Using the current vel (not peak) means the
     * inertia starts at the speed the user was actually seeing the
     * moment release was detected, so there's no observable jump
     * between the tracking output and the coast.  Angle invariance
     * still holds because fast_magnitude of (vel_x, vel_y) is
     * symmetric — a 45° flick and an axis-aligned flick of the same
     * physical strength both land on the same cur_mag here.  Guarded
     * by `peak_* != 0` on the tracked axis so a pure cross-axis
     * gesture doesn't get unsolicited Y/X inertia. */
    int32_t ax = effective_axis(cfg);
    int32_t cur_mag = fast_magnitude(data->vel_x, data->vel_y);
    if (ax == AXIS_Y && data->peak_vel_y != 0) {
        int32_t sign = (data->peak_vel_y > 0) ? 1 : -1;
        data->vel_y = clamp_velocity(cur_mag * sign, cfg->limit_fp);
    } else if (ax == AXIS_X && data->peak_vel_x != 0) {
        int32_t sign = (data->peak_vel_x > 0) ? 1 : -1;
        data->vel_x = clamp_velocity(cur_mag * sign, cfg->limit_fp);
    } else if (ax == AXIS_BOTH && cur_mag > cfg->limit_fp) {
        /* Per-axis clamp in update_velocity_ema bounds each of vel_y
         * and vel_x to limit_fp individually, which lets a diagonal
         * coast reach a vector magnitude of up to limit_fp × √2.
         * `limit` is meant as the runaway cap on the *physical* coast
         * speed, so in AXIS_BOTH we re-project onto a limit_fp circle
         * by scaling vel_y and vel_x proportionally — preserves the
         * gesture's direction while matching the single-axis cap. */
        data->vel_y = (int64_t)data->vel_y * cfg->limit_fp / cur_mag;
        data->vel_x = (int64_t)data->vel_x * cfg->limit_fp / cur_mag;
    }

    /* Commitment factor (saturation curve): 500 permille right at the
     * arming threshold (total_movement == move), asymptotically
     * approaching 1000 as the gesture grows.  Used by the tick handler
     * to scale the decay rate; the taper below confines the effect to
     * the upper speed band so the tail still converges at the unscaled
     * rate. */
    data->commit_permille = (int64_t)data->total_movement * 1000
                            / (data->total_movement + cfg->move);
    /* Per-axis taper reference, captured post-clamp so taper starts at
     * 1.0 on each live axis regardless of AXIS_BOTH vs single-axis. */
    data->init_vel_y = abs32(data->vel_y);
    data->init_vel_x = abs32(data->vel_x);

    k_work_cancel_delayable(&data->stop_detect_work);
    data->state = SS_COASTING;

    LOG_DBG("Inertia start  vel_y=%d vel_x=%d  mov=%d  commit=%d",
            data->vel_y, data->vel_x, data->total_movement,
            data->commit_permille);

    k_work_schedule(&data->inertia_tick_work,
                    K_MSEC(safe_tick_ms(cfg->tick_ms)));
}

/* ------------------------------------------------------------------ */
/* Event classification                                                */
/* ------------------------------------------------------------------ */

static enum event_class classify_event(const struct input_event *event,
                                       int32_t ax) {
    if (event->type != INPUT_EV_REL) return EC_UNTRACKED;

    bool is_y = (event->code == INPUT_REL_WHEEL);
    bool is_x = (event->code == INPUT_REL_HWHEEL);
    if (!is_y && !is_x) return EC_UNTRACKED;

    if (ax == AXIS_Y && is_x) return EC_CROSS_AXIS;
    if (ax == AXIS_X && is_y) return EC_CROSS_AXIS;

    /* AXIS_BOTH accepts both; AXIS_Y accepts Y; AXIS_X accepts X. */
    return EC_TRACKED;
}

/* ------------------------------------------------------------------ */
/* Per-state event handlers                                            */
/* ------------------------------------------------------------------ */

/* Returns true if the event should still be passed through downstream
 * with its current value.  False means the handler consumed it (or
 * zeroed it) and nothing further needs to happen.
 *
 * `raw_value` is the event's delta before the cross-axis merge was
 * applied to event->value.  We feed the EMA / peak / total_movement
 * from the raw value so the per-axis trackers don't double-count the
 * cross-axis contribution (which is already captured independently
 * through vel_x/vel_y on its own events).  event->value keeps the
 * merged magnitude because it's what flows downstream (pre-inertia
 * scroll output should still be boosted for diagonal rolls). */
static bool handle_tracked_in_tracking(struct scroll_inertia_data *data,
                                      const struct scroll_inertia_config *cfg,
                                      struct input_event *event,
                                      int32_t raw_value) {
    int32_t ax = effective_axis(cfg);
    bool is_y = (event->code == INPUT_REL_WHEEL  && ax != AXIS_X);
    bool is_x = (event->code == INPUT_REL_HWHEEL && ax != AXIS_Y);

    data->tracking_count++;

    /* Direction reversal on the tracked axis resets gesture-wide state
     * (decel_count, total_movement, tracking_count) so the old
     * direction's peak can't trigger false deceleration or premature
     * arming.  See update_tracked_axis for the reset logic. */
    if (is_y) {
        update_tracked_axis(data, cfg, raw_value,
                            &data->vel_y, &data->peak_vel_y);
    }
    if (is_x) {
        update_tracked_axis(data, cfg, raw_value,
                            &data->vel_x, &data->peak_vel_x);
    }

    /* Arming check — uses the vector magnitude of the per-axis peaks
     * so that a sloppy-angle flick arms at the same overall strength
     * as an axis-aligned one.  fast_magnitude ≈ sqrt(x²+y²) with a
     * max ~12% error, plenty accurate for a threshold. */
    int32_t peak_magnitude = fast_magnitude(data->peak_vel_x,
                                            data->peak_vel_y);
    bool vel_armed = (peak_magnitude >= cfg->start_fp);
    /* post_coast bypasses min-events: after a real coast, the user's
     * next flick doesn't need to re-prove itself against the noise
     * filter.  start/move still apply, so sub-threshold jitter can't
     * arm. */
    bool events_ok = data->post_coast ||
                     data->tracking_count >= cfg->min_events;
    bool armed = vel_armed && data->total_movement >= cfg->move
                 && events_ok;

    /* Deceleration detection — magnitude-based for angle invariance.
     * A flick slowing down in X but steady in Y (or vice versa) is
     * still slowing down overall; single-axis decel checks used to
     * miss those cases. */
    if (armed) {
        int32_t cur_magnitude = fast_magnitude(data->vel_x, data->vel_y);
        bool decelerating = false;
        if (cur_magnitude <
                (int64_t)peak_magnitude * cfg->decel_ratio / 1000) {
            /* Direction guard: the event's own axis peak must share
             * sign with the event.  During a reversal the EMA on
             * that axis lags and the overall magnitude can dip for a
             * reason other than real deceleration. */
            int32_t axis_peak = is_y ? data->peak_vel_y : data->peak_vel_x;
            if (axis_peak == 0 ||
                (raw_value > 0) == (axis_peak > 0)) {
                decelerating = true;
            }
        }

        if (decelerating) {
            data->decel_count++;
            if (data->decel_count >= cfg->decel_samples) {
                to_coasting(data, cfg);
                event->value = 0;
                return false;
            }
        } else {
            data->decel_count = 0;
        }
    }

    /* Event is passing through — schedule stop_detect so that a sudden
     * release (no more events) can still trigger inertia. */
    k_work_reschedule(&data->stop_detect_work, K_MSEC(cfg->release_ms));
    return true;
}

/* Returns true to pass the event through, false if consumed/zeroed.
 * `raw_value` is forwarded to handle_tracked_in_tracking if this
 * event triggers a reverse-cancel fall-through (so the new tracking
 * gesture starts from the same unmerged delta as any other event). */
static bool handle_tracked_in_coasting(struct scroll_inertia_data *data,
                                      const struct scroll_inertia_config *cfg,
                                      struct input_event *event,
                                      int32_t raw_value) {
    /* classify_event routes cross-axis events to handle_cross_axis in
     * single-axis modes, so any event reaching this handler is on a
     * tracked axis — the raw event code is therefore sufficient to
     * pick the inertia velocity. */
    bool is_y = (event->code == INPUT_REL_WHEEL);
    int32_t inertia_vel = is_y ? data->vel_y : data->vel_x;
    int32_t event_vel_fp = event->value * FP_SCALE;
    bool same_dir = inertia_vel != 0 &&
                    (event->value > 0) == (inertia_vel > 0);

    if (same_dir) {
        /* Tracked-axis activity means the user isn't just cross-
         * rolling; clear the cross accumulator so diagonal coasts
         * (interleaved tracked/cross events) don't reach the break
         * threshold through accumulated X alone.  Shared across all
         * three same-direction branches below. */
        data->cross_axis_accumulated = 0;

        /* Bump.  The ball's natural deceleration may briefly fluctuate
         * above the (faster-decaying) inertia velocity — bump the
         * inertia velocity up to track the ball instead of cancelling,
         * keeping the handoff seamless.  A new deliberate flick above
         * the current inertia velocity takes the same path, which is
         * fine: it refreshes the coast rather than fighting it. */
        if (abs32(event_vel_fp) > abs32(inertia_vel)) {
            if (is_y) data->vel_y = event_vel_fp;
            else      data->vel_x = event_vel_fp;
            event->value = 0;
            data->suppress_count++;
            if (data->suppress_count >= cfg->suppress_limit) {
                /* Long same-direction absorption — user has been
                 * actively rolling past a stale coast.  Drop back to
                 * TRACKING.  The event's value was zeroed above; do
                 * not revive it. */
                to_tracking_from_coasting(data);
            }
            return false;
        }

        /* Below-inertia same-direction event.  Two sub-cases:
         *
         * (1) Within the handoff window, OR event velocity sits close
         *     to the current inertia — this is almost certainly the
         *     tail of the ball's natural post-flick deceleration (which
         *     decays more slowly than the configured inertia, so its
         *     events stay within ~50% of inertia_vel as the bump path
         *     tracks it).  Absorb to avoid double-emitting the same
         *     physical motion that inertia is already representing.
         *
         * (2) Post-handoff AND event velocity is clearly below half
         *     of inertia — a new weak user gesture, not ball tail.
         *     Pass through so the user's own input drives visible
         *     scroll alongside the decaying tick; silently absorbing
         *     it would stall the screen at the late-coast sparse-emit
         *     regime.  We do not cancel the coast: the event is much
         *     weaker than the current inertia, so letting inertia
         *     finish out is the natural behaviour. */
        int64_t coast_age = k_uptime_get() - data->inertia_start_time;
        bool clearly_weaker = abs32(event_vel_fp) < (abs32(inertia_vel) >> 1);
        if (coast_age < cfg->handoff_ms || !clearly_weaker) {
            event->value = 0;
            data->suppress_count++;
            if (data->suppress_count >= cfg->suppress_limit) {
                to_tracking_from_coasting(data);
            }
            return false;
        }

        return true;
    }

    /* Reverse direction — user wants to scroll the other way.  Cancel
     * inertia and re-process this event in TRACKING. */
    to_tracking_from_coasting(data);
    return handle_tracked_in_tracking(data, cfg, event, raw_value);
}

/* AXIS_BOTH COASTING handler.  2D-pan semantics: X and Y are equal
 * first-class axes and each is absorbed / bumped / reset on its own,
 * so a pure Y coast isn't cancelled by an unrelated X input and a
 * reverse on one axis doesn't kill the other axis's coast.
 *
 * Differences from the single-axis handler:
 *   - Event axis with axis_vel == 0 is a *fresh* axis (not a reversal);
 *     pass through and leave the other axis coasting.
 *   - Reverse on an axis is a *partial reset*: zero only that axis's
 *     vel / peak / accum, keep coasting on the other axis.  Only when
 *     both axes are now dead do we fall back to the shared
 *     to_tracking_from_coasting path.
 *
 * Same-direction bump / absorb / weak-passthrough semantics mirror the
 * single-axis handler one-to-one (intentional duplication to keep each
 * handler's logic readable in isolation). */
static bool handle_tracked_in_coasting_both(struct scroll_inertia_data *data,
                                           const struct scroll_inertia_config *cfg,
                                           struct input_event *event,
                                           int32_t raw_value) {
    bool is_y = (event->code == INPUT_REL_WHEEL);
    int32_t *axis_vel   = is_y ? &data->vel_y      : &data->vel_x;
    int32_t *axis_peak  = is_y ? &data->peak_vel_y : &data->peak_vel_x;
    int32_t *axis_accum = is_y ? &data->accum_y    : &data->accum_x;
    int32_t event_vel_fp = event->value * FP_SCALE;

    /* Fresh axis.  The user is adding a new pan direction while the
     * other axis still coasts; don't treat vel==0 as a reversal.  We
     * intentionally don't build a new coast on this axis here — arming
     * only happens via TRACKING, which keeps the state machine simple. */
    if (*axis_vel == 0) {
        return true;
    }

    bool same_dir = (event->value > 0) == (*axis_vel > 0);
    if (same_dir) {
        if (abs32(event_vel_fp) > abs32(*axis_vel)) {
            *axis_vel = event_vel_fp;
            event->value = 0;
            data->suppress_count++;
            if (data->suppress_count >= cfg->suppress_limit) {
                to_tracking_from_coasting(data);
            }
            return false;
        }

        int64_t coast_age = k_uptime_get() - data->inertia_start_time;
        bool clearly_weaker = abs32(event_vel_fp) < (abs32(*axis_vel) >> 1);
        if (coast_age < cfg->handoff_ms || !clearly_weaker) {
            event->value = 0;
            data->suppress_count++;
            if (data->suppress_count >= cfg->suppress_limit) {
                to_tracking_from_coasting(data);
            }
            return false;
        }

        return true;
    }

    /* Reverse on this axis.  Partial reset so the user regains direct
     * control on this axis; the other axis keeps coasting.  Only fall
     * back to TRACKING when both axes are dead — otherwise we'd kill a
     * legitimate orthogonal coast. */
    *axis_vel   = 0;
    *axis_peak  = 0;
    *axis_accum = 0;

    int32_t other_vel = is_y ? data->vel_x : data->vel_y;
    if (other_vel == 0) {
        to_tracking_from_coasting(data);
        return handle_tracked_in_tracking(data, cfg, event, raw_value);
    }
    return true;
}

/* Single-axis cross-axis events never pass through downstream.  They
 * - stash the value in pending_other so the next tracked-axis event
 *   can magnitude-merge it (preserving the "diagonal = full scroll"
 *   behaviour the merge was designed for),
 * - during COASTING, count toward the freeze-break threshold,
 * - during TRACKING, also update the cross-axis velocity / peak and
 *   advance tracking_count.  That lets arming use the *vector
 *   magnitude* of both peaks — so a sloppy-angle flick crosses the
 *   `start` threshold the same as an axis-aligned flick of the same
 *   physical strength. */
static void handle_cross_axis(struct scroll_inertia_data *data,
                              const struct scroll_inertia_config *cfg,
                              struct input_event *event) {
    int32_t raw_value = event->value;
    int32_t cross_mag = abs32(raw_value);
    data->pending_other = raw_value;
    event->value = 0;

    if (data->state == SS_COASTING) {
        data->cross_axis_accumulated += cross_mag;
        if (data->cross_axis_accumulated >= cfg->move) {
            /* Vigorous cross-axis rolling — user is driving the
             * untracked axis.  Drop the silent coast so they regain
             * direct control.  to_tracking_from_coasting() clears
             * pending_other, so re-stash this event's value so the
             * new TRACKING gesture gets the magnitude contribution
             * on the next tracked-axis event. */
            to_tracking_from_coasting(data);
            data->pending_other = raw_value;
        }
        return;
    }

    if (data->state == SS_TRACKING) {
        /* Update cross-axis EMA/peak in parallel with the tracked
         * axis.  This is identical to the tracked-axis EMA math; the
         * only difference is that a cross-axis direction reversal
         * doesn't reset gesture-wide fields (decel_count,
         * total_movement, tracking_count) — those are tracked-axis
         * concerns. */
        int32_t ax = effective_axis(cfg);
        int32_t *vel_cross  = (ax == AXIS_Y) ? &data->vel_x : &data->vel_y;
        int32_t *peak_cross = (ax == AXIS_Y) ? &data->peak_vel_x
                                             : &data->peak_vel_y;

        *vel_cross  = update_velocity_ema(raw_value, *vel_cross, cfg);
        *peak_cross = update_peak(*vel_cross, *peak_cross, cfg);

        data->tracking_count++;
        /* total_movement receives the raw cross-axis magnitude.  The
         * tracked-axis handler now feeds EMA/peak/total from raw
         * (unmerged) values too, so no double-counting: each event
         * contributes its own |delta| exactly once. */
        data->total_movement += cross_mag;

        k_work_reschedule(&data->stop_detect_work,
                          K_MSEC(cfg->release_ms));
        return;
    }

    /* SS_IDLE: keep the stop_detect reschedule so a following tracked
     * event still sees a live timer.  No tracking yet. */
    k_work_reschedule(&data->stop_detect_work, K_MSEC(cfg->release_ms));
}

/* ------------------------------------------------------------------ */
/* Implicit state resets                                               */
/* ------------------------------------------------------------------ */

static bool should_reset_on_timeout(struct scroll_inertia_data *data,
                                   const struct scroll_inertia_config *cfg,
                                   int64_t now) {
    /* Gesture timeout: long silence means the next event is a brand
     * new gesture.  Applies in any state. */
    if (data->last_event_time > 0 &&
        now - data->last_event_time > cfg->gesture_timeout_ms) {
        return true;
    }

    /* Stale-inertia: inertia is scheduled but the event stream has
     * paused long enough that the coast is no longer "live".  Only
     * meaningful in COASTING. */
    int32_t tick_ms = safe_tick_ms(cfg->tick_ms);
    if (data->state == SS_COASTING && data->last_event_time > 0 &&
        now - data->last_event_time > tick_ms * 2) {
        return true;
    }

    return false;
}

/* ------------------------------------------------------------------ */
/* Delayed-work handlers                                               */
/* ------------------------------------------------------------------ */

static void inertia_tick_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct scroll_inertia_data *data =
        CONTAINER_OF(dwork, struct scroll_inertia_data, inertia_tick_work);
    const struct scroll_inertia_config *cfg = data->dev->config;

    if (data->state != SS_COASTING) {
        return;
    }

    /* Layer gate */
    if (cfg->layer >= 0 && !zmk_keymap_layer_active(cfg->layer)) {
        to_idle(data);
        return;
    }

    /* Duration gate (safety cap only — the natural exponential decay
     * already gives lighter flicks shorter inertia because lower
     * initial velocity reaches the stop threshold sooner). */
    if (k_uptime_get() - data->inertia_start_time > cfg->span_ms) {
        to_idle(data);
        return;
    }

    int32_t ax = effective_axis(cfg);

    /* Set fast=0 and slow=0 with all three rates equal for a
     * single-curve (iOS-style) decay; see apply_decay.  commit_permille
     * and per-axis init_vel_* (all captured at to_coasting) shape a
     * velocity-tapered extra loss so weakly-committed gestures shed
     * their upper-speed phase quickly while the tail converges at the
     * unscaled baseline rate. */
    if (ax != AXIS_X) apply_decay(&data->vel_y, cfg,
                                  data->commit_permille,
                                  data->init_vel_y);
    if (ax != AXIS_Y) apply_decay(&data->vel_x, cfg,
                                  data->commit_permille,
                                  data->init_vel_x);

    /* Stop gate */
    bool below_y = (ax == AXIS_X) || abs32(data->vel_y) < cfg->stop_fp;
    bool below_x = (ax == AXIS_Y) || abs32(data->vel_x) < cfg->stop_fp;
    if (below_y && below_x) {
        to_idle(data);
        return;
    }

    /* Accumulate sub-unit values and emit integer scroll deltas */
    int16_t emit_x = 0, emit_y = 0;
    int32_t sdiv = safe_scale_div(cfg->scale_div);
    if (ax != AXIS_X) emit_y = accumulate_and_emit(data->vel_y, &data->accum_y, cfg, sdiv);
    if (ax != AXIS_Y) emit_x = accumulate_and_emit(data->vel_x, &data->accum_x, cfg, sdiv);

    if (emit_x != 0 || emit_y != 0) {
        int16_t out_x = emit_x;
        int16_t out_y = emit_y;
        /* swap-mod has no effect when unlock-mod is freeing both axes */
        if (ax != AXIS_BOTH && swap_active(cfg)) {
            out_x = emit_y;
            out_y = emit_x;
        }
        zmk_hid_mouse_movement_set(0, 0);
        zmk_hid_mouse_scroll_set(out_x, out_y);
        zmk_endpoints_send_mouse_report();
        zmk_hid_mouse_scroll_set(0, 0);
    }

    k_work_schedule(&data->inertia_tick_work,
                    K_MSEC(safe_tick_ms(cfg->tick_ms)));
}

static void stop_detect_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct scroll_inertia_data *data =
        CONTAINER_OF(dwork, struct scroll_inertia_data, stop_detect_work);
    const struct scroll_inertia_config *cfg = data->dev->config;

    /* stop_detect only runs as a TRACKING → {COASTING | IDLE} bridge;
     * if we somehow fire while coasting or idle, noop. */
    if (data->state != SS_TRACKING) {
        return;
    }

    if (cfg->layer >= 0 && !zmk_keymap_layer_active(cfg->layer)) {
        to_idle(data);
        return;
    }

    /* Same vector-magnitude basis as the in-event arming check — see
     * handle_tracked_in_tracking for the rationale. */
    int32_t vel_magnitude = fast_magnitude(data->vel_x, data->vel_y);
    bool vel_ok = (vel_magnitude >= cfg->start_fp);
    bool mov_ok = data->total_movement >= cfg->move;

    bool events_ok = data->post_coast ||
                     data->tracking_count >= cfg->min_events;
    if (vel_ok && mov_ok && events_ok) {
        to_coasting(data, cfg);
    } else {
        to_idle(data);
    }
}

/* ------------------------------------------------------------------ */
/* Input processor callback                                            */
/* ------------------------------------------------------------------ */

static int scroll_inertia_handle_event(const struct device *dev,
                                       struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    struct scroll_inertia_data *data = dev->data;
    const struct scroll_inertia_config *cfg = dev->config;

    int32_t ax = effective_axis(cfg);
    enum event_class ec = classify_event(event, ax);

    if (ec == EC_UNTRACKED) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* Implicit resets from timing (applies regardless of event class
     * so that a long silence followed by any event lands us in IDLE). */
    int64_t now = k_uptime_get();
    if (should_reset_on_timeout(data, cfg, now)) {
        to_idle(data);
    }
    data->last_event_time = now;

    /* Cross-axis events are self-contained: they don't enter the state
     * machine dispatch (they're buffered/counted and early-returned). */
    if (ec == EC_CROSS_AXIS) {
        handle_cross_axis(data, cfg, event);
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* EC_TRACKED from here on.
     *
     * Capture the raw delta before the cross-axis merge is applied.
     * The handlers use `raw_value` for all tracking state updates
     * (EMA, peak, total_movement) so the vector-magnitude view of
     * the gesture isn't double-counted through merged Y and
     * independent vel_x.  event->value gets the merged magnitude
     * because that's what flows downstream as the direct scroll
     * output and what the COASTING absorb/bump logic compares
     * against the inertia velocity. */
    int32_t raw_value = event->value;

    /* Apply pending cross-axis magnitude merge if we're in a
     * single-axis mode and the previous event stashed a value.
     * AXIS_BOTH never buffers, so pending_other stays zero there. */
    if (data->pending_other != 0 && event->value != 0) {
        int32_t sign = event->value >= 0 ? 1 : -1;
        event->value = sign * fast_magnitude(event->value,
                                              data->pending_other);
        data->pending_other = 0;
    }

    /* Zero-value tracked events are treated as idle keep-alives: they
     * extend the stop_detect timer but don't drive state changes. */
    if (raw_value == 0) {
        if (data->state == SS_TRACKING) {
            k_work_reschedule(&data->stop_detect_work,
                              K_MSEC(cfg->release_ms));
        }
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* Dispatch to the state handler. */
    bool pass_through = true;
    switch (data->state) {
    case SS_IDLE:
        to_tracking_from_idle(data);
        pass_through = handle_tracked_in_tracking(data, cfg, event, raw_value);
        break;
    case SS_TRACKING:
        pass_through = handle_tracked_in_tracking(data, cfg, event, raw_value);
        break;
    case SS_COASTING:
        pass_through = (ax == AXIS_BOTH)
            ? handle_tracked_in_coasting_both(data, cfg, event, raw_value)
            : handle_tracked_in_coasting(data, cfg, event, raw_value);
        break;
    }

    if (pass_through && ax != AXIS_BOTH && swap_active(cfg)) {
        /* Only swap the output channel when we're actually letting
         * the event through; inertia-absorbed events have value=0
         * and swapping them is a no-op anyway. */
        if (event->code == INPUT_REL_WHEEL)       event->code = INPUT_REL_HWHEEL;
        else if (event->code == INPUT_REL_HWHEEL) event->code = INPUT_REL_WHEEL;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

/* ------------------------------------------------------------------ */
/* Device boilerplate                                                  */
/* ------------------------------------------------------------------ */

static int scroll_inertia_init(const struct device *dev) {
    struct scroll_inertia_data *data = dev->data;
    data->dev = dev;
    data->state = SS_IDLE;
    k_work_init_delayable(&data->stop_detect_work, stop_detect_handler);
    k_work_init_delayable(&data->inertia_tick_work, inertia_tick_handler);
    return 0;
}

static struct zmk_input_processor_driver_api scroll_inertia_driver_api = {
    .handle_event = scroll_inertia_handle_event,
};

#define SCROLL_INERTIA_INST(n)                                                \
    static struct scroll_inertia_data scroll_inertia_data_##n = {0};          \
    static const struct scroll_inertia_config scroll_inertia_config_##n = {   \
        .gain       = DT_INST_PROP(n, gain),                                  \
        .blend      = DT_INST_PROP(n, blend),                                 \
        .start_fp   = DT_INST_PROP(n, start) * FP_SCALE,                      \
        .move       = DT_INST_PROP(n, move),                                  \
        .release_ms = DT_INST_PROP(n, release),                               \
        .fast_fp    = DT_INST_PROP(n, fast) * FP_SCALE,                       \
        .decay_fast = DT_INST_PROP(n, decay_fast),                            \
        .decay_slow = DT_INST_PROP(n, decay_slow),                            \
        .slow_fp    = DT_INST_PROP(n, slow) * FP_SCALE,                       \
        .decay_tail = DT_INST_PROP(n, decay_tail),                            \
        .friction_fp = DT_INST_PROP(n, friction) * FP_SCALE / 1000,           \
        .stop_fp    = DT_INST_PROP(n, stop) * FP_SCALE,                       \
        .scale      = DT_INST_PROP(n, scale),                                 \
        .scale_div  = DT_INST_PROP(n, scale_div),                             \
        .limit_fp   = DT_INST_PROP(n, limit) * FP_SCALE,                      \
        .span_ms    = DT_INST_PROP(n, span),                                  \
        .tick_ms    = DT_INST_PROP(n, tick),                                   \
        .axis       = DT_INST_PROP(n, axis),                                   \
        .layer      = DT_INST_PROP(n, layer),                                  \
        .swap_mod   = DT_INST_PROP(n, swap_mod),                              \
        .unlock_mod = DT_INST_PROP(n, unlock_mod),                            \
        .min_events         = DT_INST_PROP(n, min_events),                    \
        .decel_samples      = DT_INST_PROP(n, decel_samples),                 \
        .decel_ratio        = DT_INST_PROP(n, decel_ratio),                   \
        .peak_decay         = DT_INST_PROP(n, peak_decay),                    \
        .gesture_timeout_ms = DT_INST_PROP(n, gesture_timeout),               \
        .suppress_limit     = DT_INST_PROP(n, suppress_limit),                \
        .handoff_ms         = DT_INST_PROP(n, handoff_ms),                    \
    };                                                                        \
    DEVICE_DT_INST_DEFINE(n, scroll_inertia_init, NULL,                       \
                          &scroll_inertia_data_##n,                           \
                          &scroll_inertia_config_##n,                         \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,   \
                          &scroll_inertia_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_INERTIA_INST)
