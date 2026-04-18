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

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* Fixed-point: 8-bit fractional part (×256) */
#define FP_SHIFT 8
#define FP_SCALE (1 << FP_SHIFT)

/* Axis mode constants */
#define AXIS_BOTH 0
#define AXIS_Y    1
#define AXIS_X    2

/* Consecutive sub-peak samples required to confirm deceleration */
#define DECEL_CONFIRM_COUNT  3

/* If no event arrives for this many ms, the next event is treated as
 * the start of a brand-new gesture and all tracking state is reset.
 * This prevents stale velocity / peak from a previous scroll session
 * (or layer) from contaminating a new one. */
#define GESTURE_TIMEOUT_MS   100

/* EMA must drop below this fraction of peak to count as deceleration.
 * 850 / 1000 = 85%.  Prevents minor EMA fluctuations during steady
 * scrolling from falsely triggering inertia. */
#define DECEL_PEAK_RATIO     850

/* Hard safety limit: if this many consecutive events are suppressed
 * (by inertia absorption etc.), force a full state reset.
 * 50 events ≈ 400 ms at 125 Hz. */
#define SUPPRESS_SAFETY_LIMIT 50

/* Peak velocity decay rate (permille per event).  When the current
 * velocity is below the peak, the peak slowly decays toward it.  This
 * prevents a brief initial acceleration transient from leaving a
 * permanently inflated peak that makes steady-state scrolling look
 * like deceleration. */
#define PEAK_DECAY           990

/* Minimum events after a reset before arming is allowed.  Gives the
 * EMA time to converge and filters out transient spikes from shared
 * upstream modules (e.g. a pointer-acceleration processor switching
 * from cursor mode to scroll mode). */
#define MIN_TRACKING_EVENTS  10

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
/* Configuration (read from Devicetree at compile time)                */
/* ------------------------------------------------------------------ */

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
    int32_t stop_fp;

    int32_t scale;
    int32_t scale_div;
    int32_t limit_fp;
    int32_t span_ms;
    int32_t tick_ms;

    int32_t axis;
    int32_t layer;
    int32_t swap_mod;
    int32_t unlock_mod;     /* modifier bitmask that frees both axes */
};

/* ------------------------------------------------------------------ */
/* Runtime state                                                       */
/* ------------------------------------------------------------------ */

struct scroll_inertia_data {
    const struct device *dev;

    /* Velocity EMA in fixed-point (×256) */
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

    /* Sub-unit scroll accumulators */
    int32_t accum_x;
    int32_t accum_y;

    /* Inertia state */
    bool inertia_active;
    int64_t inertia_start_time;
    int32_t start_movement;

    /* Timestamp of the last event (for gesture-timeout detection) */
    int64_t last_event_time;

    /* Consecutive suppressed events (safety net) */
    int32_t suppress_count;

    /* Delayed work items */
    struct k_work_delayable stop_detect_work;
    struct k_work_delayable inertia_tick_work;
};

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static inline int32_t abs32(int32_t v) { return v < 0 ? -v : v; }

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

static void reset_state(struct scroll_inertia_data *data) {
    data->vel_x = 0;
    data->vel_y = 0;
    data->peak_vel_x = 0;
    data->peak_vel_y = 0;
    data->total_movement = 0;
    data->decel_count = 0;
    data->tracking_count = 0;
    data->suppress_count = 0;
    data->pending_other = 0;
}

static void cancel_inertia(struct scroll_inertia_data *data) {
    data->inertia_active = false;
    k_work_cancel_delayable(&data->inertia_tick_work);
    data->accum_x = 0;
    data->accum_y = 0;
    reset_state(data);
}

static void start_inertia(struct scroll_inertia_data *data,
                          const struct scroll_inertia_config *cfg) {
    data->inertia_active = true;
    data->inertia_start_time = k_uptime_get();
    data->start_movement = data->total_movement;
    data->accum_x = 0;
    data->accum_y = 0;
    k_work_cancel_delayable(&data->stop_detect_work);

    LOG_DBG("Inertia start  vel_y=%d vel_x=%d  mov=%d",
            data->vel_y, data->vel_x, data->total_movement);

    k_work_schedule(&data->inertia_tick_work, K_MSEC(safe_tick_ms(cfg->tick_ms)));
}

/* ------------------------------------------------------------------ */
/* Inertia tick                                                        */
/* ------------------------------------------------------------------ */

static void inertia_tick_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct scroll_inertia_data *data =
        CONTAINER_OF(dwork, struct scroll_inertia_data, inertia_tick_work);
    const struct scroll_inertia_config *cfg = data->dev->config;

    if (!data->inertia_active) {
        return;
    }

    /* Layer gate */
    if (cfg->layer >= 0 && !zmk_keymap_layer_active(cfg->layer)) {
        cancel_inertia(data);
        return;
    }

    /* Duration gate (safety cap only — the natural exponential decay
     * already gives lighter flicks shorter inertia because lower
     * initial velocity reaches the stop threshold sooner). */
    if (k_uptime_get() - data->inertia_start_time > cfg->span_ms) {
        cancel_inertia(data);
        return;
    }

    int32_t ax = effective_axis(cfg);

    /* Three-stage decay.  Set fast=0 and slow=0 with all three rates
     * equal for a single-curve (iOS-style) decay. */
    if (ax != AXIS_X) {
        int32_t av = abs32(data->vel_y);
        int32_t d = av > cfg->fast_fp ? cfg->decay_fast
                  : av > cfg->slow_fp ? cfg->decay_slow
                  :                     cfg->decay_tail;
        data->vel_y = (int64_t)data->vel_y * d / 1000;
    }
    if (ax != AXIS_Y) {
        int32_t av = abs32(data->vel_x);
        int32_t d = av > cfg->fast_fp ? cfg->decay_fast
                  : av > cfg->slow_fp ? cfg->decay_slow
                  :                     cfg->decay_tail;
        data->vel_x = (int64_t)data->vel_x * d / 1000;
    }

    /* Stop gate */
    bool below_y = (ax == AXIS_X) || abs32(data->vel_y) < cfg->stop_fp;
    bool below_x = (ax == AXIS_Y) || abs32(data->vel_x) < cfg->stop_fp;
    if (below_y && below_x) {
        cancel_inertia(data);
        return;
    }

    /* Accumulate sub-unit values and emit integer scroll deltas */
    int16_t emit_x = 0, emit_y = 0;

    int32_t sdiv = safe_scale_div(cfg->scale_div);
    if (ax != AXIS_X) {
        data->accum_y += (int64_t)data->vel_y * cfg->scale / sdiv;
        emit_y = (int16_t)(data->accum_y >> FP_SHIFT);
        data->accum_y -= (int32_t)emit_y << FP_SHIFT;
    }
    if (ax != AXIS_Y) {
        data->accum_x += (int64_t)data->vel_x * cfg->scale / sdiv;
        emit_x = (int16_t)(data->accum_x >> FP_SHIFT);
        data->accum_x -= (int32_t)emit_x << FP_SHIFT;
    }

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

    k_work_schedule(&data->inertia_tick_work, K_MSEC(safe_tick_ms(cfg->tick_ms)));
}

/* ------------------------------------------------------------------ */
/* Stop detection (fallback for abrupt stops)                          */
/* ------------------------------------------------------------------ */

static void stop_detect_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct scroll_inertia_data *data =
        CONTAINER_OF(dwork, struct scroll_inertia_data, stop_detect_work);
    const struct scroll_inertia_config *cfg = data->dev->config;

    if (data->inertia_active) {
        return;
    }

    /* If the layer became inactive between the last event and this
     * timer firing, do not start inertia. */
    if (cfg->layer >= 0 && !zmk_keymap_layer_active(cfg->layer)) {
        reset_state(data);
        return;
    }

    int32_t ax = effective_axis(cfg);
    bool vel_ok = false;
    if (ax != AXIS_X) vel_ok |= (abs32(data->vel_y) >= cfg->start_fp);
    if (ax != AXIS_Y) vel_ok |= (abs32(data->vel_x) >= cfg->start_fp);

    bool mov_ok = data->total_movement >= cfg->move;

    if (vel_ok && mov_ok && data->tracking_count >= MIN_TRACKING_EVENTS) {
        start_inertia(data, cfg);
    } else {
        reset_state(data);
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

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    bool is_y = (event->code == INPUT_REL_WHEEL);
    bool is_x = (event->code == INPUT_REL_HWHEEL);

    if (!is_y && !is_x) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int32_t ax = effective_axis(cfg);

    /* ---- Magnitude merge for single-axis modes ----
     * In axis=1 (Y only) or axis=2 (X only) mode, fold the non-tracked
     * axis component into the tracked axis so that diagonal rotation
     * produces the same scroll amount as straight rotation of equal
     * physical distance.  Skipped when unlock-mod is held (ax becomes
     * AXIS_BOTH and both axes flow through independently). */
    if (ax == AXIS_Y && is_x) {
        data->pending_other = event->value;
        event->value = 0;
        data->last_event_time = k_uptime_get();
        if (!data->inertia_active) {
            k_work_reschedule(&data->stop_detect_work,
                              K_MSEC(cfg->release_ms));
        }
        return ZMK_INPUT_PROC_CONTINUE;
    }
    if (ax == AXIS_X && is_y) {
        data->pending_other = event->value;
        event->value = 0;
        data->last_event_time = k_uptime_get();
        if (!data->inertia_active) {
            k_work_reschedule(&data->stop_detect_work,
                              K_MSEC(cfg->release_ms));
        }
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (data->pending_other != 0 && event->value != 0) {
        int32_t sign = event->value >= 0 ? 1 : -1;
        event->value = sign * fast_magnitude(event->value,
                                              data->pending_other);
        data->pending_other = 0;
    }

    /* Re-evaluate which axis this event targets after the merge. */
    is_y = (event->code == INPUT_REL_WHEEL  && ax != AXIS_X);
    is_x = (event->code == INPUT_REL_HWHEEL && ax != AXIS_Y);

    if (!is_y && !is_x) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* ---- State reset triggers ---- */
    int64_t now = k_uptime_get();
    bool need_reset = false;

    if (data->last_event_time > 0 &&
        now - data->last_event_time > GESTURE_TIMEOUT_MS) {
        need_reset = true;
    }
    if (data->suppress_count >= SUPPRESS_SAFETY_LIMIT) {
        need_reset = true;
    }
    /* Stale-inertia detection: if inertia is active but no events
     * arrived for more than a couple of tick intervals, the ball has
     * stopped or the layer was briefly toggled.  Either way, the
     * inertia belongs to a previous interaction and must end. */
    if (data->inertia_active && data->last_event_time > 0 &&
        now - data->last_event_time > safe_tick_ms(cfg->tick_ms) * 2) {
        need_reset = true;
    }

    if (need_reset) {
        if (data->inertia_active) {
            cancel_inertia(data);
        } else {
            reset_state(data);
        }
        k_work_cancel_delayable(&data->stop_detect_work);
    }
    data->last_event_time = now;

    /* ---- Inertia active: absorb same-direction, cancel on reverse ---- */
    if (data->inertia_active) {
        if (event->value == 0) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

        int32_t inertia_vel = is_y ? data->vel_y : data->vel_x;
        int32_t event_vel_fp = (int32_t)event->value << FP_SHIFT;
        bool same_dir = inertia_vel != 0 &&
                        (event->value > 0) == (inertia_vel > 0);

        if (same_dir) {
            /* The ball's natural deceleration may briefly fluctuate
             * above the (faster-decaying) inertia velocity.  Bump
             * the inertia velocity up to track the ball instead of
             * cancelling — this keeps the transition seamless. */
            if (abs32(event_vel_fp) > abs32(inertia_vel)) {
                if (is_y) data->vel_y = event_vel_fp;
                if (is_x) data->vel_x = event_vel_fp;
            }
            event->value = 0;
            data->suppress_count++;
            return ZMK_INPUT_PROC_CONTINUE;
        }

        /* Reverse direction — user wants to scroll the other way. */
        cancel_inertia(data);
        /* Fall through to tracking */
    }

    /* ---- Tracking ---- */
    if (event->value == 0) {
        k_work_reschedule(&data->stop_detect_work, K_MSEC(cfg->release_ms));
        return ZMK_INPUT_PROC_CONTINUE;
    }

    data->tracking_count++;

    if (is_y) {
        int32_t delta_fp = (int32_t)event->value << FP_SHIFT;
        data->vel_y = ((int64_t)delta_fp * cfg->gain +
                       (int64_t)data->vel_y * cfg->blend) / 1000;
        data->vel_y = clamp_velocity(data->vel_y, cfg->limit_fp);
        data->total_movement += abs32(event->value);

        /* Direction reversal — reset peak and movement for the new
         * direction so the old direction's state doesn't trigger
         * false deceleration or premature arming. */
        if (data->peak_vel_y != 0 &&
            (data->vel_y > 0) != (data->peak_vel_y > 0)) {
            data->peak_vel_y = data->vel_y;
            data->decel_count = 0;
            data->total_movement = abs32(event->value);
        } else if (abs32(data->vel_y) > abs32(data->peak_vel_y)) {
            data->peak_vel_y = data->vel_y;
        } else {
            int32_t decayed = (int64_t)data->peak_vel_y * PEAK_DECAY / 1000;
            data->peak_vel_y = abs32(decayed) > abs32(data->vel_y)
                                   ? decayed : data->vel_y;
        }
    }
    if (is_x) {
        int32_t delta_fp = (int32_t)event->value << FP_SHIFT;
        data->vel_x = ((int64_t)delta_fp * cfg->gain +
                       (int64_t)data->vel_x * cfg->blend) / 1000;
        data->vel_x = clamp_velocity(data->vel_x, cfg->limit_fp);
        data->total_movement += abs32(event->value);

        if (data->peak_vel_x != 0 &&
            (data->vel_x > 0) != (data->peak_vel_x > 0)) {
            data->peak_vel_x = data->vel_x;
            data->decel_count = 0;
            data->total_movement = abs32(event->value);
        } else if (abs32(data->vel_x) > abs32(data->peak_vel_x)) {
            data->peak_vel_x = data->vel_x;
        } else {
            int32_t decayed = (int64_t)data->peak_vel_x * PEAK_DECAY / 1000;
            data->peak_vel_x = abs32(decayed) > abs32(data->vel_x)
                                   ? decayed : data->vel_x;
        }
    }

    /* Arming check */
    bool vel_armed = false;
    if (ax != AXIS_X) vel_armed |= (abs32(data->peak_vel_y) >= cfg->start_fp);
    if (ax != AXIS_Y) vel_armed |= (abs32(data->peak_vel_x) >= cfg->start_fp);
    bool armed = vel_armed && data->total_movement >= cfg->move
                 && data->tracking_count >= MIN_TRACKING_EVENTS;

    /* Deceleration detection */
    if (armed) {
        bool decelerating = false;
        if (is_y && ax != AXIS_X &&
            abs32(data->vel_y) <
                abs32(data->peak_vel_y) * DECEL_PEAK_RATIO / 1000) {
            /* Only count when the raw event direction matches the peak.
             * During a reversal the EMA lags behind the actual direction
             * and would otherwise look like deceleration of the old peak. */
            if (data->peak_vel_y == 0 ||
                (event->value > 0) == (data->peak_vel_y > 0)) {
                decelerating = true;
            }
        }
        if (is_x && ax != AXIS_Y &&
            abs32(data->vel_x) <
                abs32(data->peak_vel_x) * DECEL_PEAK_RATIO / 1000) {
            if (data->peak_vel_x == 0 ||
                (event->value > 0) == (data->peak_vel_x > 0)) {
                decelerating = true;
            }
        }

        if (decelerating) {
            data->decel_count++;
            if (data->decel_count >= DECEL_CONFIRM_COUNT) {
                start_inertia(data, cfg);
                event->value = 0;
                return ZMK_INPUT_PROC_CONTINUE;
            }
        } else {
            data->decel_count = 0;
        }
    }

    /* Event passed through */
    data->suppress_count = 0;
    k_work_reschedule(&data->stop_detect_work, K_MSEC(cfg->release_ms));

    /* Swap output axis if the configured modifier is held.
     * unlock-mod takes precedence: when both axes are free, swapping
     * has no meaning. */
    if (ax != AXIS_BOTH && swap_active(cfg)) {
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
        .start_fp   = DT_INST_PROP(n, start) << FP_SHIFT,                    \
        .move       = DT_INST_PROP(n, move),                                  \
        .release_ms = DT_INST_PROP(n, release),                               \
        .fast_fp    = DT_INST_PROP(n, fast) << FP_SHIFT,                      \
        .decay_fast = DT_INST_PROP(n, decay_fast),                            \
        .decay_slow = DT_INST_PROP(n, decay_slow),                            \
        .slow_fp    = DT_INST_PROP(n, slow) << FP_SHIFT,                      \
        .decay_tail = DT_INST_PROP(n, decay_tail),                            \
        .stop_fp    = DT_INST_PROP(n, stop) << FP_SHIFT,                      \
        .scale      = DT_INST_PROP(n, scale),                                 \
        .scale_div  = DT_INST_PROP(n, scale_div),                             \
        .limit_fp   = DT_INST_PROP(n, limit) << FP_SHIFT,                    \
        .span_ms    = DT_INST_PROP(n, span),                                  \
        .tick_ms    = DT_INST_PROP(n, tick),                                   \
        .axis       = DT_INST_PROP(n, axis),                                   \
        .layer      = DT_INST_PROP(n, layer),                                  \
        .swap_mod   = DT_INST_PROP(n, swap_mod),                              \
        .unlock_mod = DT_INST_PROP(n, unlock_mod),                            \
    };                                                                        \
    DEVICE_DT_INST_DEFINE(n, scroll_inertia_init, NULL,                       \
                          &scroll_inertia_data_##n,                           \
                          &scroll_inertia_config_##n,                         \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,   \
                          &scroll_inertia_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_INERTIA_INST)
