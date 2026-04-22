# zmk-input-processor-scroll-inertia

[![test](https://github.com/mjmjm0101/zmk-input-processor-scroll-inertia/actions/workflows/test.yml/badge.svg)](https://github.com/mjmjm0101/zmk-input-processor-scroll-inertia/actions/workflows/test.yml)

iOS-style inertial scrolling for ZMK trackballs.

*[日本語版 README はこちら / Japanese version available here](README_ja.md)*

> **Stop accelerating. Start coasting.**

![scroll demo](https://github.com/user-attachments/assets/fc8f89e9-8212-4a2c-80cf-3898f67d8bdf)

>
> Most ZMK trackball setups try to feel responsive by adding scroll
> *acceleration* (move the ball faster → scroll more aggressively).
> This module takes a different path: it adds true *inertia*
> (release the ball → scrolling continues, then gradually fades).
> Combine the two if you like — but if you've never tried inertia
> alone, do that first.  It's not just a feature — it's the missing
> piece of the trackball experience.

*Note: the inertia feel has been smoothed out compared to v0.5.x and earlier.*

---

## What this module is

A ZMK input processor that watches scroll events on a single axis, detects when you've flicked the ball and let go, and then keeps emitting smoothly fading scroll output until it naturally stops.

The fade is a straightforward two-part model: each tick, multiply the velocity by a constant slightly less than 1 (iOS-style exponential decay), then subtract a tiny constant (a touch of friction so small flicks don't glide forever).
Between the two, three numbers give you the whole curve.
Optional multi-stage settings are there if you want different behaviour at high vs low speeds.

## What this module isn't

- Not a *scroll acceleration* processor.
  It does not amplify input.
- Not an "axis-detection" or "smart axis lock" implementation.
  See below.

---

## Why multiple layers, not one

### The truth about trackballs

A trackball gives you continuous 2D rotation.
There's no built-in "axis" — every roll has both X and Y in it.
Whether you *meant* to scroll vertically or horizontally is something only you know; it's not in the rotation data at all.

### Why auto-detection always loses

Single-layer implementations try to guess your intent from the motion.
That guessing works most of the time but breaks down at:

- The ~45° angle (no clear winner)
- Direction reversals (the previous direction's peak misleads the guess)
- Slow drift (noise looks like signal)
- The handoff between active scroll and inertia (state has to carry over correctly)

These aren't bugs you can fix — they happen because the firmware is trying to read information that simply isn't in the input.

### The solution: tell, don't guess

This module skips the guessing entirely and asks **you** to tell it what you mean, by holding a layer key:

- One layer for vertical scroll → `axis = <1>`
- Another for horizontal scroll → `axis = <2>`
- Optionally a third for free 2D / canvas pan → `axis = <0>`

The whole class of axis-guessing problems disappears — not because they're handled cleverly, but because they can't happen.

### *"But I don't have that many spare layers."*

That's fine.
While the cleanest experience comes from dedicated layers, you can get the same 100% axis-lock reliability on a single scroll layer by using two helper modifiers:

- `swap-mod` (e.g. `MOD_LSFT`) — while held, the output axis is inverted.
  Vertical scroll layer + Shift = horizontal scroll.
- `unlock-mod` (e.g. `MOD_RGUI`) — while held, the axis lock is released entirely and both axes flow through (free 2D / canvas pan).

You're still declaring your intent to the firmware — just through a modifier instead of a layer change.
The core principle is unchanged: no guessing, no stuck axes, deterministic behaviour at every angle.

> **Pick the modifiers carefully.**  They are sent to the host while
> held, so they can collide with host shortcuts (e.g. Cmd/Ctrl + scroll
> = zoom in most apps).  Two ways to stay safe:
>
> 1. Choose modifiers your host rarely pairs with scroll — right-side
>    modifiers like `MOD_RGUI` are a good default.
> 2. Even safer: bind the modifier key **only inside the scroll layer**.
>    The key falls through to whatever's underneath in every other
>    layer, so the modifier is only ever sent during an active scroll
>    session — never during normal typing.

---

## Installation

### `west.yml`

Add to the `projects:` list in your `config/west.yml`:

```yaml
manifest:
  remotes:
    - name: mjmjm0101
      url-base: https://github.com/mjmjm0101
  projects:
    - name: zmk-input-processor-scroll-inertia
      remote: mjmjm0101
      revision: main
```

### `*.conf`

Nothing to add.
The module turns itself on when it sees a matching node in your overlay.

### `*.overlay`

Recommended placement: **before** `&zip_scroll_scaler` in the chain, with `scale` and `scale-div` set to the scaler's two arguments.

```dts
#include <dt-bindings/zmk/modifiers.h>

/ {
    scroll_inertia_v: scroll_inertia_v {
        compatible = "zmk,input-processor-scroll-inertia";
        #input-processor-cells = <0>;

        /* axis: 0 = both (free 2D),
                  1 = Y only (vertical scroll),
                  2 = X only (horizontal scroll) */
        axis      = <1>;
        layer     = <4>;          /* your vertical-scroll layer */

        /* Match the downstream zip_scroll_scaler arguments */
        scale     = <4>;
        scale-div = <675>;

        /* Optional: hold Shift to swap the output axis */
        swap-mod  = <(MOD_LSFT|MOD_RSFT)>;

        /* Optional: hold Right-GUI to free both axes (2D scroll) */
        unlock-mod = <MOD_RGUI>;
    };

    /* This dedicated free-2D layer is OPTIONAL — the unlock-mod above
     * already gives you 2D scrolling from inside the vertical layer.
     * Pick whichever style you prefer; you don't need both. */
    scroll_inertia_free: scroll_inertia_free {
        compatible = "zmk,input-processor-scroll-inertia";
        #input-processor-cells = <0>;
        axis      = <0>;          /* both axes — diagonal scroll allowed */
        layer     = <5>;          /* free 2D scroll layer (e.g. canvas pan) */
        scale     = <4>;
        scale-div = <675>;
    };

    trackball_listener: trackball_listener {
        compatible = "zmk,input-listener";
        device = <&trackball>;
        input-processors = <&pointer_accel /* etc */>;

        scroller_v {
            layers = <4>;
            input-processors = <
                &zip_y_scaler (-1) 1
                &zip_xy_to_scroll_mapper
                &scroll_inertia_v
                &zip_scroll_scaler 4 675
            >;
        };

        scroller_free {
            layers = <5>;
            input-processors = <
                &zip_y_scaler (-1) 1
                &zip_xy_to_scroll_mapper
                &scroll_inertia_free
                &zip_scroll_scaler 4 675
            >;
        };
    };
};
```

> **A note about `pointer_accel`:** iOS-style scrolling demands 1:1
> physical fidelity while you're rolling.  If you have an acceleration
> processor in your scroll chain, slow physical movement gets boosted
> into values that look like flicks, falsely triggering inertia.  Keep
> `pointer_accel` in your default (cursor) chain if you want, but
> **leave it out of the scroll chains**.

---

## Properties reference

A note on units: several properties take values "out of 1000" (sometimes called *permille*).
For example, a decay rate of `990` means *multiply velocity by 0.990 each tick*.
This avoids floating-point math on the keyboard.

**Enforced safety rules** (the module rewrites these silently to keep itself running):

- `scale-div = 0` is treated as `1` (would otherwise be division by zero).
- `tick = 0` is treated as `1` ms (would otherwise busy-loop).

**Recommended ranges** (your responsibility — the module does *not* auto-correct these, out-of-range values just produce out-of-range behaviour):

- `gain` + `blend` should add to `1000`.
  Otherwise the EMA is unstable: it grows unbounded if the sum exceeds 1000, or under-tracks the input if it is below.
- `decay-fast`, `decay-slow`, `decay-tail` in `[800, 999]`.
  Values ≥ 1000 prevent decay entirely (velocity grows or holds forever).
- `slow` ≤ `fast`.
  The multi-stage check assumes ascending boundaries; inverting them silently skips the mid stage.

| Property | Default | Description |
|---|---|---|
| `gain` | `300` | How strongly each new event pulls the smoothed velocity (out of 1000).  Together with `blend`, this controls a moving average: `velocity = (event * gain + velocity * blend) / 1000`. |
| `blend` | `700` | How much of the previous velocity is kept (out of 1000).  `gain + blend` should equal 1000. |
| `start` | `40` | The peak **vector** velocity (in raw scroll units, `√(peak_x² + peak_y²)`) that a flick must reach before inertia is allowed to kick in.  Angle-invariant: a diagonal flick arms at the same overall strength as an axis-aligned one.  Raise it if casual scrolls keep triggering inertia. |
| `move` | `80` | The total raw distance the ball must roll within a single flick before inertia is allowed to kick in.  Sums raw per-event magnitudes from both axes.  Stops tiny stray motions from triggering it.  Raise it if casual scrolls trigger inertia, lower it if real flicks fail to. |
| `release` | `24` | If no events arrive for this many milliseconds, the ball is treated as released (~3 frames at 125 Hz). |
| `fast` | `0` | Velocity boundary between the fast and mid decay zones.  `0` disables zoning (single curve). |
| `decay-fast` | `990` | High-speed decay (out of 1000 per tick).  990 means `velocity *= 0.990` each tick. |
| `decay-slow` | `990` | Mid-speed decay. |
| `slow` | `0` | Velocity boundary between mid and tail zones.  `0` disables zoning. |
| `decay-tail` | `990` | Low-speed (tail) decay. |
| `friction` | `35` | Constant absolute deceleration subtracted from velocity each tick, **in permille of a scroll unit** (`friction=1000` = 1 unit/tick).  Additive (Coulomb) friction on top of the multiplicative `decay-*`.  Its relative bite grows as velocity shrinks, so small flicks stop noticeably sooner while large flicks in their high-velocity phase are less affected.  `0` disables (pure exponential).  See note below. |
| `stop` | `7` | When velocity drops below this, inertia stops.  See note below. |
| `scale` | `1000` | Output scale numerator. |
| `scale-div` | `1000` | Output scale denominator.  Each tick adds `velocity × scale / scale-div` to an internal accumulator and emits whole scroll units when it overflows. |
| `limit` | `600` | Velocity safety cap.  Keeps a wild flick from producing absurd inertia. |
| `span` | `6000` | Maximum inertia duration in milliseconds.  This is a runaway safety cap; the natural fade usually finishes much earlier. |
| `tick` | `8` | How often the inertia step runs, in milliseconds.  Match your sensor's polling rate (8 ms ≈ 125 Hz). |
| `axis` | `0` | `0` = both axes (diagonal), `1` = Y only (vertical), `2` = X only (horizontal). |
| `layer` | `-1` | If `>= 0`, inertia stops the moment this layer turns off.  `-1` disables the check. |
| `swap-mod` | `0` | Bitmask of modifier keys (`MOD_LSFT`, `MOD_RCTL`, etc.) that, while held, swap the output axis (V↔H).  `0` disables. |
| `unlock-mod` | `0` | Bitmask of modifier keys that, while held, release the axis lock so both axes flow through (free 2D scroll, equivalent to `axis = <0>`).  Takes precedence over `swap-mod`.  `0` disables. |

### Advanced tuning

These knobs expose the internal state-machine timing.
The defaults are tuned for a 1000 CPI PMW3610 at 125 Hz and cover typical setups; only adjust if you've characterised a specific problem on your own hardware.
Out-of-range values will *not* be auto-corrected.

| Property | Default | Recommended range | Description |
|---|---|---|---|
| `min-events` | `10` | `[3, 30]` | Minimum tracked-axis events required before arming is allowed.  Lets the EMA converge and filters transient spikes.  Too low arms on noise; too high prevents arming on short but deliberate flicks.  Automatically skipped when TRACKING is re-entered directly from COASTING, so rapid reversal flicks aren't blocked by this gate. |
| `decel-samples` | `3` | `[1, 10]` | Consecutive sub-peak samples required to confirm deceleration before TRACKING → COASTING.  `1` triggers on any single dip; high values delay the transition. |
| `decel-ratio` | `850` | `[700, 950]` | Deceleration threshold as permille of peak magnitude.  `850` = current must drop below 85% of peak.  Higher triggers on minor fluctuations; lower delays activation on gentle releases. |
| `peak-decay` | `990` | `[950, 999]` | Peak velocity decay rate (permille per event).  Lets the peak drift down when velocity is below it, avoiding an inflated peak from a brief initial acceleration transient.  **Values ≥ 1000 are unsafe** — the peak can grow without bound. |
| `gesture-timeout` | `100` | `[50, 500]` | Milliseconds of silence after which the next event starts a brand-new gesture (all tracking state reset).  Prevents stale velocity/peak from a previous session contaminating a new one. |
| `suppress-limit` | `50` | `[20, 200]` | Hard safety limit: consecutive same-direction events absorbed in COASTING before forcing a transition back to TRACKING.  `50` ≈ 400 ms at 125 Hz.  Too low causes drop-outs on valid long coasts; too high lets stuck states persist. |
| `handoff-ms` | `100` | `[50, 300]` | Duration (ms) after inertia starts during which any below-inertia same-direction event is absorbed unconditionally (covers the brief window where the ball's natural post-flick deceleration is still producing events).  After the window, a same-direction event only passes through when it is *clearly weaker* than inertia (below half of inertia_vel): ball decay events stay close to inertia_vel so they remain absorbed and normal flick behaviour is unchanged, while a genuinely weak new user gesture falls well below the half-threshold and flows through to avoid the visible stall at the late-coast sparse-emit regime.  Above-inertia events still bump regardless of window. |

### About `stop`

How smooth the *very end* of the inertia feels depends not on `stop` alone, but on `stop` combined with `scale` and `scale-div`:

```
output frequency at cutoff (Hz) ≈ stop × scale × 125 / scale-div
```

Below about 5 Hz the discrete scroll units start to feel like tiny clicks instead of smooth motion.
Tune so the cutoff stays above that.

| `scale/scale-div` | recommended `stop` (≈ 5 Hz cutoff) |
|---|---|
| `1:1` (1000/1000) | `1` |
| `1:10` (100/1000) | `10` |
| `4:675` (recommended placement) | `7` |
| `1:100` (10/1000) | `100` |

Counter-intuitively, lowering `stop` makes the tail feel **worse**, not better — it lets the velocity fade further, into a region where the system has to wait longer and longer between scroll units.
Raise `stop` instead and you get a clean cutoff while motion is still smooth.

### About `friction`

`decay-*` is multiplicative (velocity × rate), so every flick has the *same relative tail length* — a small flick and a large flick both take a similar number of ticks to fade out, just from different starting points.
That's why small flicks can feel like they glide a bit too long even when big flicks feel right.

`friction` adds a constant absolute deceleration on top, configured in permille of a scroll unit per tick (`friction=1000` subtracts 1 unit/tick).
Because it subtracts the same amount regardless of current velocity, its relative bite grows as velocity shrinks:

| Phase | Exp decel (at `decay-fast=990`) | Friction `friction=35` (default) | Who dominates |
|---|---|---|---|
| Large flick peak (vel=600) | −6/tick (1.0%) | −0.035/tick (0.006%) | Exponential |
| Mid (vel=100) | −1/tick (1.0%) | −0.035/tick (0.035%) | Exponential |
| Tail (vel=20) | −0.2/tick (1.0%) | −0.035/tick (0.175%) | Exponential |
| Near-stop (vel=5) | −0.05/tick (1.0%) | −0.035/tick (0.7%) | Roughly even |

Small flicks spend more of their run at low velocity, where friction bites harder relative to the EMA decay, so they stop sooner than a pure-exponential coast would leave them.

**Caveat:** friction is additive and applies every tick for the full coast.
Its total effect over a long coast is noticeable, so very high `friction` values clip big flicks too.
The default `35` is tuned to shorten small flicks without being felt on big ones.

**Precision:** values below ~4 truncate to zero internally (8-bit fixed point).
If you want to disable friction entirely, set it to `0` (not a small positive number).

To tune: raise toward ~100 for a firmer stop on small flicks, drop toward `0` for a longer, purely-exponential tail.
Values above `~500` visibly shorten large flicks too.

### About `scale` / `scale-div`

The processor's inertia output is sent straight to the host (it bypasses any scaler that comes after it in the chain).
To keep the inertia speed equal to the active scroll speed, set `scale` and `scale-div` to the same numbers as the scaler downstream:

```dts
&scroll_inertia_v
&zip_scroll_scaler 4 675     /* ← these two arguments  */

scroll_inertia_v {
    scale     = <4>;          /*    must match these   */
    scale-div = <675>;
};
```

The default `1000` / `1000` (1:1) is only correct if you place the processor at the *end* of the chain (after any scaler).
That's not the recommended placement — putting it before the scaler gives the velocity tracker bigger numbers to work with (more precision) and avoids zero-valued events that the scaler emits while it's accumulating its internal remainder.

---

## Tuning tips

A few non-obvious lessons from real-world tuning.

### Reduce your overall scroll amount

Without inertia, you probably made up for slow scrolling by setting your `zip_scroll_scaler` aggressively (or by using macros that emit `&msc` 3-5 times per detent).
With inertia, each flick already covers a lot of ground — keeping the old aggressive scale on top of that makes scrolling feel uncontrollably fast.

Rule of thumb: roughly halve your previous scroll amount when adopting inertia.
Update both the `zip_scroll_scaler` arguments **and** this processor's `scale` / `scale-div` (they should always match).

### Keep `pointer_accel` out of the scroll chain

iOS-style scrolling demands 1:1 physical fidelity while you're actively rolling.
If `pointer_accel` (or any other acceleration processor) is in the scroll chain, slow physical movement gets boosted into values that look like flicks, triggering inertia when you didn't mean to flick.
Leave it in the cursor chain if you want; just remove it from the scroll chain.

### Tune `start` and `move` to your trackball

These two numbers decide *when* inertia activates.
The right values depend on your trackball CPI, your acceleration settings, and how hard you typically flick.

- Start at `start = 40`, `move = 80` (the bundled defaults — calibrated on a 1000 CPI PMW3610 against vector-magnitude arming).
- If casual scrolls keep triggering inertia: raise both 1.5–2× until only deliberate flicks trigger it.
- If real flicks aren't triggering: lower them by ~30%.

The two thresholds work independently — `start` filters by peak **vector** speed (combining both axes), `move` filters by cumulative distance.
Both must be cleared for inertia to start.
Because `start` operates on the magnitude, sloppy-angle flicks and precise flicks hit the same threshold at the same physical strength.

### Pick a decay rate that matches your style

The default `decay-* = 990` gives an iOS-like long, smooth tail.

| Goal | Setting |
|---|---|
| iOS-like long glide | `990–995` (single curve, all three rates equal) |
| Snappier, shorter inertia | `980–985` |
| Two-stage: fast initial, gentle tail | `decay-fast = 985`, `decay-slow = decay-tail = 993`, `fast = 60` |

Try a single curve first.
It usually gets you 90% of the way there with one parameter, before you reach for multi-stage.

### Shorten small flicks with `friction`

The default `friction = 35` already adds a gentle additive stop so small flicks don't glide forever.
If the feel is still off, adjust in either direction.
Representative coast times at different friction values (pure exponential vs friction-augmented):

| Value | Large flick (v=600) | Small flick (v=75) | Notes |
|---|---|---|---|
| `0` | 3.5 s | 1.9 s | Pure exponential, no asymmetry. |
| `35` (default) | ≈3.3 s (−6%) | ≈1.7 s (−12%) | Small-flick tail noticeably trimmed, big flicks barely touched. |
| `100` | 2.8 s (−20%) | 1.3 s (−30%) | Firmer — useful if default still feels long on small flicks. |
| `200` | 2.5 s (−30%) | 1.0 s (−47%) | Aggressive; big flicks start to feel snappier too. |
| `500`+ | 1.9 s or less | sub-second | Too much for an iOS feel; big flicks get clipped. |

If small flicks still coast too long: raise in `25`–`50` steps.
If big flicks feel dampened: lower, eventually to `0` for purely exponential decay.
Values below `~4` truncate to zero internally, so don't bother with `1`–`3`.

### Don't extend the tail too far

Scroll units are whole numbers — you can't scroll "half a notch."
Below about 5 scroll units per second the gaps between scroll units become visible as little jolts, no matter how smooth the underlying velocity curve is.
See **About `stop`** above for the formula and the table of recommended `stop` values.

The fix is to **raise** `stop`, not to lower it.
Letting velocity fade all the way to zero gives you the worst feel — long pauses between single scroll clicks, and then silence.

### Match `tick` to your input device polling rate

The default `tick = 8` (≈ 125 Hz) matches the PMW3610.
For a 60 Hz sensor use `tick = 16`; for higher rates, lower it.
A mismatched `tick` doesn't break anything, but matching it makes the handoff between active scroll and inertia feel more in sync.

---

## How it works

The processor runs a small three-state machine per instance:

```
    IDLE  ──▶  TRACKING  ──▶  COASTING  ──▶  IDLE
```

Transitions in detail:

| From → To | Trigger |
|---|---|
| `IDLE → TRACKING` | First tracked-axis event. |
| `TRACKING → COASTING` | Peak magnitude ≥ `start`, total movement ≥ `move`, ≥ `min-events` (skipped when TRACKING was entered directly from a prior COASTING), then deceleration confirmed (`decel-samples` consecutive sub-peak samples).  Or the `stop_detect` fallback fires after `release` ms of silence with the same conditions. |
| `COASTING → TRACKING` | Reverse-direction event, cross-axis freeze break (cumulative cross-axis motion ≥ `move` while coasting, in single-axis modes), or `suppress-limit` consecutive same-direction absorbed events (suppress safety). |
| `COASTING → IDLE` | Velocity < `stop`, `span` exceeded, or the bound layer turns off. |
| *any* `→ IDLE` | Gesture timeout (no events for `gesture-timeout` ms), or stale-inertia detection (no events for two ticks while COASTING). |

### States

1. **IDLE.**
   No gesture in progress.
   The first tracked-axis event transitions to TRACKING.
2. **TRACKING.**
   Every event updates a per-axis smoothed velocity (moving average) and a per-axis "peak so far" that slowly drifts down toward the current value when no new high is reached.
   The tracking state is fully angle-invariant: EMA, peak, and total movement are all fed from the event's **raw** delta, so a cross-axis event contributes exactly once and a tracked-axis event exactly once — no double counting through the merge.
   Both arming and deceleration detection compare *vector magnitudes* (`√(peak_x² + peak_y²)`, approximated) rather than per-axis values, so a sloppy-angle flick crosses thresholds at the same overall strength as an axis-aligned one.
   Once magnitude ≥ `start`, total movement ≥ `move`, and at least `min-events` events have been seen, the gesture is *armed* — if the magnitude then drops below `decel-ratio`/1000 of the peak magnitude for `decel-samples` events in a row, the processor transitions to COASTING.
   If no events arrive for `release` ms while armed, a fallback "stop detect" timer performs the same check for abrupt releases.
   **Post-coast shortcut:** if TRACKING was entered directly from a prior COASTING (reverse flick, cross-axis break, suppress safety, or stale-inertia reset), the `min-events` gate is skipped on the next arm — the previous coast already proved the gesture was committed, so short rapid reversal flicks don't need to re-accumulate 10 events per direction.
   `start` and `move` still apply, so sub-threshold jitter can't arm.
3. **COASTING.**
   On entry the tracked axis's initial velocity is reseeded to the current vector-magnitude EMA — so a diagonal flick coasts at the same strength as an axis-aligned flick of the same physical vector strength, not just the tracked axis's component.
   Using the *current* EMA rather than the peak means the coast picks up from the speed the user was actually seeing at release, with no observable acceleration jump.
   A timer fires every `tick` ms.
   Each tick multiplies velocity by the configured decay (slightly accelerated in the upper-speed band for gestures that only just cleared `move`, so weakly-committed flicks don't sustain the high-speed phase as long as committed ones), subtracts friction, accumulates, and emits whole scroll units to the host.
   Tracking-axis events that arrive in the same direction are absorbed (possibly bumping velocity to match the ball's actual motion); opposite-direction events cancel inertia back to TRACKING.
   Coasting ends when velocity drops below `stop`, when the layer turns off, or when the `span` safety cap is reached.

### Implicit resets

Three timing-based checks can force a transition back to IDLE from any state:

- **Gesture timeout** (`gesture-timeout` ms with no events).
  The next event restarts from a clean slate.
- **Stale inertia** (coasting but no events for more than two ticks).
  The coast has detached from the physical gesture; end it.
- **Cross-axis break** (cumulative cross-axis motion exceeding `move` during coasting, in single-axis mode).
  The user is actively rolling the untracked axis — give them direct control.

### Safety

A `suppress_count` counts consecutive same-direction events absorbed during COASTING; after `suppress-limit` (default `50` ≈ 400 ms at 125 Hz) the processor drops back to TRACKING on the assumption that the user has been rolling through a stale coast.
The cross-axis accumulator handles the more common freeze vector separately, so this is a genuinely rare fallback.

---

## Project status

This project is feature-complete for its intended purpose.

I may update it for critical bugs or major ZMK breaking changes, but I do not plan frequent feature additions.

---

## License

MIT
