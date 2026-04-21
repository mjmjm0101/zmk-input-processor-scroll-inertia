/*
 * Host-side unit tests for scroll_inertia_math.h.
 *
 * These cover the numeric invariants of the pure math helpers used
 * by the input processor: the EMA filter, peak update rule, decay
 * curve, and fixed-point accumulator.  The state machine and any
 * Zephyr/ZMK-dependent code is NOT exercised here — see README.md
 * and CONTRIBUTING.md for why.
 *
 * Build & run:
 *   make -C tests test
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>

#include "../src/scroll_inertia_math.h"

static int total_tests = 0;
static int failed_tests = 0;

#define ASSERT_EQ(label, actual, expected) do {                         \
    long long _a = (long long)(actual);                                 \
    long long _e = (long long)(expected);                               \
    total_tests++;                                                      \
    if (_a != _e) {                                                     \
        fprintf(stderr, "FAIL %s: got %lld, expected %lld (%s:%d)\n",   \
                (label), _a, _e, __FILE__, __LINE__);                   \
        failed_tests++;                                                 \
    }                                                                   \
} while (0)

#define ASSERT_TRUE(label, cond) do {                                   \
    total_tests++;                                                      \
    if (!(cond)) {                                                      \
        fprintf(stderr, "FAIL %s: expected true (%s:%d)\n",             \
                (label), __FILE__, __LINE__);                           \
        failed_tests++;                                                 \
    }                                                                   \
} while (0)

#define ASSERT_FALSE(label, cond) do {                                  \
    total_tests++;                                                      \
    if ((cond)) {                                                       \
        fprintf(stderr, "FAIL %s: expected false (%s:%d)\n",            \
                (label), __FILE__, __LINE__);                           \
        failed_tests++;                                                 \
    }                                                                   \
} while (0)

/* Mirror of the YAML binding defaults, with friction_fp pre-scaled the
 * same way the SCROLL_INERTIA_INST macro does. */
static struct scroll_inertia_config default_cfg(void) {
    struct scroll_inertia_config cfg = {0};
    cfg.gain         = 300;
    cfg.blend        = 700;
    cfg.start_fp     = 40 * FP_SCALE;
    cfg.move         = 80;
    cfg.release_ms   = 24;
    cfg.fast_fp      = 0;
    cfg.decay_fast   = 990;
    cfg.decay_slow   = 990;
    cfg.slow_fp      = 0;
    cfg.decay_tail   = 990;
    cfg.friction_fp  = 35 * FP_SCALE / 1000;   /* = 8 */
    cfg.stop_fp      = 7 * FP_SCALE;
    cfg.scale        = 1000;
    cfg.scale_div    = 1000;
    cfg.limit_fp     = 600 * FP_SCALE;
    cfg.peak_decay   = 990;
    return cfg;
}

/* ------------------------------------------------------------------ */
static void test_fast_magnitude(void) {
    printf("-- fast_magnitude\n");
    ASSERT_EQ("(0,0)",       fast_magnitude(0, 0), 0);
    ASSERT_EQ("(5,0)",       fast_magnitude(5, 0), 5);
    ASSERT_EQ("(0,5)",       fast_magnitude(0, 5), 5);
    ASSERT_EQ("(-5,0)",      fast_magnitude(-5, 0), 5);
    ASSERT_EQ("(0,-5)",      fast_magnitude(0, -5), 5);
    /* Approximation: max + min/2.  (3,4) gives 4+1=5, matching real sqrt. */
    ASSERT_EQ("(3,4)",       fast_magnitude(3, 4), 5);
    ASSERT_EQ("(4,3)",       fast_magnitude(4, 3), 5);
    /* (100,100): 100+50=150 (real ~141, ~6% over). */
    ASSERT_EQ("(100,100)",   fast_magnitude(100, 100), 150);
    /* Negative handling via abs32. */
    ASSERT_EQ("(-100,-100)", fast_magnitude(-100, -100), 150);
    ASSERT_EQ("(-1000,500)", fast_magnitude(-1000, 500), 1250);
}

/* ------------------------------------------------------------------ */
static void test_clamp_velocity(void) {
    printf("-- clamp_velocity\n");
    ASSERT_EQ("in range",     clamp_velocity(100, 200), 100);
    ASSERT_EQ("zero",         clamp_velocity(0, 200), 0);
    ASSERT_EQ("upper clip",   clamp_velocity(300, 200), 200);
    ASSERT_EQ("lower clip",   clamp_velocity(-300, 200), -200);
    ASSERT_EQ("at +limit",    clamp_velocity(200, 200), 200);
    ASSERT_EQ("at -limit",    clamp_velocity(-200, 200), -200);
}

/* ------------------------------------------------------------------ */
static void test_safe_guards(void) {
    printf("-- safe_scale_div / safe_tick_ms\n");
    ASSERT_EQ("pos scale_div", safe_scale_div(1000), 1000);
    ASSERT_EQ("zero scale_div", safe_scale_div(0), 1);
    ASSERT_EQ("neg scale_div", safe_scale_div(-5), 1);
    ASSERT_EQ("pos tick_ms",   safe_tick_ms(8), 8);
    ASSERT_EQ("zero tick_ms",  safe_tick_ms(0), 1);
}

/* ------------------------------------------------------------------ */
static void test_update_velocity_ema(void) {
    printf("-- update_velocity_ema\n");
    struct scroll_inertia_config cfg = default_cfg();

    /* Zero delta, zero vel: stays zero. */
    ASSERT_EQ("0 delta 0 vel", update_velocity_ema(0, 0, &cfg), 0);

    /* delta=100, vel=0, gain=300:
     * (100*256*300 + 0) / 1000 = 7680000/1000 = 7680 */
    ASSERT_EQ("pos delta 0 vel", update_velocity_ema(100, 0, &cfg), 7680);

    /* Negative-delta path: proves `* FP_SCALE` replacement handles
     * negatives without UB (the original `<< FP_SHIFT` was UB per C11). */
    ASSERT_EQ("neg delta 0 vel", update_velocity_ema(-100, 0, &cfg), -7680);

    /* 0 delta, vel=1000: (0 + 1000*700)/1000 = 700 */
    ASSERT_EQ("0 delta + vel", update_velocity_ema(0, 1000, &cfg), 700);

    /* delta=100, vel=1000: (7680000 + 700000)/1000 = 8380 */
    ASSERT_EQ("pos delta + pos vel", update_velocity_ema(100, 1000, &cfg), 8380);

    /* Clamp upper bound. */
    struct scroll_inertia_config tight = cfg;
    tight.gain  = 1000;
    tight.blend = 0;
    tight.limit_fp = 100;
    ASSERT_EQ("upper clamp",  update_velocity_ema(10000, 0, &tight), 100);
    ASSERT_EQ("lower clamp",  update_velocity_ema(-10000, 0, &tight), -100);
}

/* ------------------------------------------------------------------ */
static void test_peak_reversed(void) {
    printf("-- peak_reversed\n");
    ASSERT_FALSE("peak=0",           peak_reversed(100, 0));
    ASSERT_FALSE("vel=0",            peak_reversed(0, 100));
    ASSERT_FALSE("both 0",           peak_reversed(0, 0));
    ASSERT_FALSE("same pos",         peak_reversed(100, 50));
    ASSERT_FALSE("same neg",         peak_reversed(-100, -50));
    ASSERT_TRUE("pos vel neg peak",  peak_reversed(100, -100));
    ASSERT_TRUE("neg vel pos peak",  peak_reversed(-100, 100));
}

/* ------------------------------------------------------------------ */
static void test_update_peak(void) {
    printf("-- update_peak\n");
    struct scroll_inertia_config cfg = default_cfg();
    cfg.peak_decay = 990;

    /* Reversal: snap to new vel. */
    ASSERT_EQ("reverse",      update_peak(-100, 100, &cfg), -100);
    /* Grow: vel beats peak. */
    ASSERT_EQ("grow",         update_peak(200, 100, &cfg), 200);
    /* Hold with decay: decayed=99 beats vel=50. */
    ASSERT_EQ("hold+decay",   update_peak(50, 100, &cfg), 99);
    /* Decay drops below vel: vel wins. */
    cfg.peak_decay = 900;
    ASSERT_EQ("decay<vel",    update_peak(95, 100, &cfg), 95);
    /* Zero peak, nonzero vel: vel wins (grow branch). */
    cfg.peak_decay = 990;
    ASSERT_EQ("peak=0 grow",  update_peak(50, 0, &cfg), 50);
    /* Both zero. */
    ASSERT_EQ("both 0",       update_peak(0, 0, &cfg), 0);
    /* Negative same-dir decay. */
    ASSERT_EQ("neg decay",    update_peak(-50, -100, &cfg), -99);
}

/* ------------------------------------------------------------------ */
static void test_apply_decay(void) {
    printf("-- apply_decay\n");
    struct scroll_inertia_config cfg = default_cfg();
    cfg.friction_fp = 0;   /* isolate multiplicative decay first */

    /* With fast=slow=0, any nonzero av uses decay_fast=990. */
    int32_t vel = 25600;   /* 100 scroll units */
    apply_decay(&vel, &cfg);
    ASSERT_EQ("decay pos", vel, 25344);

    vel = -25600;
    apply_decay(&vel, &cfg);
    ASSERT_EQ("decay neg", vel, -25344);

    /* Friction bites below threshold. */
    cfg.decay_fast = 1000;
    cfg.decay_slow = 1000;
    cfg.decay_tail = 1000;
    cfg.friction_fp = 8;
    vel = 5;
    apply_decay(&vel, &cfg);
    ASSERT_EQ("friction clamps small +", vel, 0);
    vel = -5;
    apply_decay(&vel, &cfg);
    ASSERT_EQ("friction clamps small -", vel, 0);
    vel = 100;
    apply_decay(&vel, &cfg);
    ASSERT_EQ("friction subtracts +", vel, 92);
    vel = -100;
    apply_decay(&vel, &cfg);
    ASSERT_EQ("friction subtracts -", vel, -92);

    /* Three-stage boundaries. */
    cfg.friction_fp = 0;
    cfg.fast_fp     = 1000;
    cfg.slow_fp     = 100;
    cfg.decay_fast  = 900;
    cfg.decay_slow  = 950;
    cfg.decay_tail  = 990;
    vel = 2000;
    apply_decay(&vel, &cfg);
    ASSERT_EQ("fast zone", vel, 1800);   /* 2000*900/1000 */
    vel = 500;
    apply_decay(&vel, &cfg);
    ASSERT_EQ("slow zone", vel, 475);    /* 500*950/1000 */
    vel = 50;
    apply_decay(&vel, &cfg);
    ASSERT_EQ("tail zone", vel, 49);     /* 50*990/1000 = 49 */
}

/* ------------------------------------------------------------------ */
static void test_accumulate_and_emit(void) {
    printf("-- accumulate_and_emit\n");
    struct scroll_inertia_config cfg = default_cfg();
    cfg.scale = 1000;
    cfg.scale_div = 1000;

    int32_t accum = 0;

    /* One whole unit in fixed-point → emits 1, remainder 0. */
    int16_t e = accumulate_and_emit(FP_SCALE, &accum, &cfg, 1000);
    ASSERT_EQ("emit 1",            e, 1);
    ASSERT_EQ("remainder 0",       accum, 0);

    /* Half a unit → emit 0, remainder = FP_SCALE/2. */
    accum = 0;
    e = accumulate_and_emit(FP_SCALE / 2, &accum, &cfg, 1000);
    ASSERT_EQ("emit 0 half",       e, 0);
    ASSERT_EQ("remainder half",    accum, FP_SCALE / 2);

    /* Add another half → emit 1, remainder 0. */
    e = accumulate_and_emit(FP_SCALE / 2, &accum, &cfg, 1000);
    ASSERT_EQ("emit 1 cumulated",  e, 1);
    ASSERT_EQ("remainder 0 again", accum, 0);

    /* Negative path: proves `* FP_SCALE` replacement for the
     * remainder subtraction handles negative emit without UB. */
    accum = 0;
    e = accumulate_and_emit(-FP_SCALE, &accum, &cfg, 1000);
    ASSERT_EQ("emit -1",           e, -1);
    ASSERT_EQ("remainder 0 neg",   accum, 0);

    /* Scaled ratio 4:675 (the recommended pairing). */
    cfg.scale = 4;
    cfg.scale_div = 675;
    accum = 0;
    /* vel=FP_SCALE*200 → accum += FP_SCALE*200*4/675 = 51200*4/675 = 303.
     * emit = 303>>8 = 1; remainder = 303 - 256 = 47. */
    e = accumulate_and_emit(FP_SCALE * 200, &accum, &cfg, 675);
    ASSERT_EQ("4:675 emit",        e, 1);
    ASSERT_EQ("4:675 remainder",   accum, 47);
}

/* ------------------------------------------------------------------ */
int main(void) {
    printf("scroll_inertia math unit tests\n");
    test_fast_magnitude();
    test_clamp_velocity();
    test_safe_guards();
    test_update_velocity_ema();
    test_peak_reversed();
    test_update_peak();
    test_apply_decay();
    test_accumulate_and_emit();

    printf("\n%d/%d passed\n", total_tests - failed_tests, total_tests);
    return failed_tests == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
