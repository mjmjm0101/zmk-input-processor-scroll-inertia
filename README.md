# zmk-input-processor-scroll-inertia

iOS-style inertial scrolling for ZMK trackballs.

*[日本語版 README はこちら / Japanese version available here](README_ja.md)*

> **Stop accelerating. Start coasting.**
>
> Most ZMK trackball setups try to feel responsive by adding scroll
> *acceleration* (move the ball faster → scroll more aggressively).
> This module takes a different path: it adds true *inertia*
> (release the ball → scrolling continues, then gradually fades).
> Combine the two if you like — but if you've never tried inertia
> alone, do that first.  It's not just a feature — it's the missing
> piece of the trackball experience.

---

## What this module is

A ZMK input processor that watches scroll events on a single axis,
detects when you've flicked the ball and let go, and then keeps
emitting smoothly fading scroll output until it naturally stops.

The fade follows the same simple rule iOS uses: each tick, multiply the
velocity by a constant slightly less than 1.  That's it — one number
gives you the whole curve.  (Optional multi-stage settings are there if
you want different behaviour at high vs low speeds.)

## What this module isn't

- Not a *scroll acceleration* processor.  It does not amplify input.
- Not an "axis-detection" or "smart axis lock" implementation.  See below.

---

## Why multiple layers, not one

### The truth about trackballs

A trackball gives you continuous 2D rotation.  There's no built-in
"axis" — every roll has both X and Y in it.  Whether you *meant* to
scroll vertically or horizontally is something only you know; it's not
in the rotation data at all.

### Why auto-detection always loses

Single-layer implementations try to guess your intent from the motion.
That guessing works most of the time but breaks down at:

- The ~45° angle (no clear winner)
- Direction reversals (the previous direction's peak misleads the guess)
- Slow drift (noise looks like signal)
- The handoff between active scroll and inertia (state has to carry over correctly)

These aren't bugs you can fix — they happen because the firmware is
trying to read information that simply isn't in the input.

### The solution: tell, don't guess

This module skips the guessing entirely and asks **you** to tell it
what you mean, by holding a layer key:

- One layer for vertical scroll → `axis = <1>`
- Another for horizontal scroll → `axis = <2>`
- Optionally a third for free 2D / canvas pan → `axis = <0>`

The whole class of axis-guessing problems disappears — not because
they're handled cleverly, but because they can't happen.

### *"But I don't have that many spare layers."*

That's fine.  While the cleanest experience comes from dedicated
layers, you can get the same 100% axis-lock reliability on a single
scroll layer by using two helper modifiers:

- `swap-mod` (e.g. `MOD_LSFT`) — while held, the output axis is
  inverted.  Vertical scroll layer + Shift = horizontal scroll.
- `unlock-mod` (e.g. `MOD_RGUI`) — while held, the axis lock is
  released entirely and both axes flow through (free 2D / canvas pan).

You're still declaring your intent to the firmware — just through a
modifier instead of a layer change.  The core principle is unchanged:
no guessing, no stuck axes, deterministic behaviour at every angle.

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

Nothing to add.  The module turns itself on when it sees a matching
node in your overlay.

### `*.overlay`

Recommended placement: **before** `&zip_scroll_scaler` in the chain,
with `scale` and `scale-div` set to the scaler's two arguments.

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

A note on units: several properties take values "out of 1000"
(sometimes called *permille*).  For example, a decay rate of `990`
means *multiply velocity by 0.990 each tick*.  This avoids floating-point
math on the keyboard.

**Enforced safety rules** (the module rewrites these silently to keep
itself running):

- `scale-div = 0` is treated as `1` (would otherwise be division by zero).
- `tick = 0` is treated as `1` ms (would otherwise busy-loop).

**Recommended ranges** (your responsibility — the module does *not*
auto-correct these, out-of-range values just produce out-of-range
behaviour):

- `gain` + `blend` should add to `1000`.  Otherwise the EMA is
  unstable: it grows unbounded if the sum exceeds 1000, or
  under-tracks the input if it is below.
- `decay-fast`, `decay-slow`, `decay-tail` in `[800, 999]`.  Values
  ≥ 1000 prevent decay entirely (velocity grows or holds forever).
- `slow` ≤ `fast`.  The multi-stage check assumes ascending
  boundaries; inverting them silently skips the mid stage.

| Property | Default | Description |
|---|---|---|
| `gain` | `300` | How strongly each new event pulls the smoothed velocity (out of 1000).  Together with `blend`, this controls a moving average: `velocity = (event * gain + velocity * blend) / 1000`. |
| `blend` | `700` | How much of the previous velocity is kept (out of 1000).  `gain + blend` should equal 1000. |
| `start` | `75` | The peak velocity (in raw scroll units) that a flick must reach before inertia is allowed to kick in.  The default is intentionally on the conservative side so that small, deliberate scrolls don't trigger inertia — tune to taste. |
| `move` | `135` | The total distance the ball must roll within a single flick before inertia is allowed to kick in.  Stops tiny stray motions from triggering it.  Same as `start`, the default is conservative; raise it if even casual scrolls trigger inertia, lower it if real flicks fail to. |
| `release` | `24` | If no events arrive for this many milliseconds, the ball is treated as released (~3 frames at 125 Hz). |
| `fast` | `0` | Velocity boundary between the fast and mid decay zones.  `0` disables zoning (single curve). |
| `decay-fast` | `990` | High-speed decay (out of 1000 per tick).  990 means `velocity *= 0.990` each tick. |
| `decay-slow` | `990` | Mid-speed decay. |
| `slow` | `0` | Velocity boundary between mid and tail zones.  `0` disables zoning. |
| `decay-tail` | `990` | Low-speed (tail) decay. |
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

### About `stop`

How smooth the *very end* of the inertia feels depends not on `stop`
alone, but on `stop` combined with `scale` and `scale-div`:

```
output frequency at cutoff (Hz) ≈ stop × scale × 125 / scale-div
```

Below about 5 Hz the discrete scroll units start to feel like tiny
clicks instead of smooth motion.  Tune so the cutoff stays above that.

| `scale/scale-div` | recommended `stop` (≈ 5 Hz cutoff) |
|---|---|
| `1:1` (1000/1000) | `1` |
| `1:10` (100/1000) | `10` |
| `4:675` (recommended placement) | `7` |
| `1:100` (10/1000) | `100` |

Counter-intuitively, lowering `stop` makes the tail feel **worse**, not
better — it lets the velocity fade further, into a region where the
system has to wait longer and longer between scroll units.  Raise
`stop` instead and you get a clean cutoff while motion is still smooth.

### About `scale` / `scale-div`

The processor's inertia output is sent straight to the host (it
bypasses any scaler that comes after it in the chain).  To keep the
inertia speed equal to the active scroll speed, set `scale` and
`scale-div` to the same numbers as the scaler downstream:

```dts
&scroll_inertia_v
&zip_scroll_scaler 4 675     /* ← these two arguments  */

scroll_inertia_v {
    scale     = <4>;          /*    must match these   */
    scale-div = <675>;
};
```

The default `1000` / `1000` (1:1) is only correct if you place the
processor at the *end* of the chain (after any scaler).  That's not the
recommended placement — putting it before the scaler gives the velocity
tracker bigger numbers to work with (more precision) and avoids
zero-valued events that the scaler emits while it's accumulating its
internal remainder.

---

## Tuning tips

A few non-obvious lessons from real-world tuning.

### Reduce your overall scroll amount

Without inertia, you probably made up for slow scrolling by setting
your `zip_scroll_scaler` aggressively (or by using macros that emit
`&msc` 3-5 times per detent).  With inertia, each flick already covers
a lot of ground — keeping the old aggressive scale on top of that makes
scrolling feel uncontrollably fast.

Rule of thumb: roughly halve your previous scroll amount when adopting
inertia.  Update both the `zip_scroll_scaler` arguments **and** this
processor's `scale` / `scale-div` (they should always match).

### Keep `pointer_accel` out of the scroll chain

iOS-style scrolling demands 1:1 physical fidelity while you're actively
rolling.  If `pointer_accel` (or any other acceleration processor) is
in the scroll chain, slow physical movement gets boosted into values
that look like flicks, triggering inertia when you didn't mean to
flick.  Leave it in the cursor chain if you want; just remove it from
the scroll chain.

### Tune `start` and `move` to your trackball

These two numbers decide *when* inertia activates.  The right values
depend on your trackball CPI, your acceleration settings, and how hard
you typically flick.

- Start at `start = 75`, `move = 135` (the bundled defaults — calibrated
  on a 1000 CPI PMW3610).
- If casual scrolls keep triggering inertia: raise both 1.5–2× until
  only deliberate flicks trigger it.
- If real flicks aren't triggering: lower them by ~30%.

The two thresholds work independently — `start` filters by speed, `move`
filters by distance.  Both must be cleared for inertia to start.

### Pick a decay rate that matches your style

The default `decay-* = 990` gives an iOS-like long, smooth tail.

| Goal | Setting |
|---|---|
| iOS-like long glide | `990–995` (single curve, all three rates equal) |
| Snappier, shorter inertia | `980–985` |
| Two-stage: fast initial, gentle tail | `decay-fast = 985`, `decay-slow = decay-tail = 993`, `fast = 60` |

Try a single curve first.  It usually gets you 90% of the way there
with one parameter, before you reach for multi-stage.

### Don't extend the tail too far

Scroll units are whole numbers — you can't scroll "half a notch."
Below about 5 scroll units per second the gaps between scroll units
become visible as little jolts, no matter how smooth the underlying
velocity curve is.  See **About `stop`** above for the formula and the
table of recommended `stop` values.

The fix is to **raise** `stop`, not to lower it.  Letting velocity fade
all the way to zero gives you the worst feel — long pauses between
single scroll clicks, and then silence.

### Match `tick` to your input device polling rate

The default `tick = 8` (≈ 125 Hz) matches the PMW3610.  For a 60 Hz
sensor use `tick = 16`; for higher rates, lower it.  A mismatched
`tick` doesn't break anything, but matching it makes the handoff
between active scroll and inertia feel more in sync.

---

## How it works

1. **Tracking.**  Every event updates a smoothed velocity (a moving
   average) and a "peak so far" that slowly drifts down toward the
   current value when no new high is reached.
2. **Arming.**  Once peak ≥ `start`, total movement ≥ `move`, and at
   least 10 events have been seen, the gesture is "armed" — meaning
   inertia is allowed to take over.
3. **Deceleration detection.**  While armed, if the smoothed velocity
   drops below 85% of the peak for 3 events in a row (and the event
   direction matches the peak direction), inertia takes over.
4. **Inertia.**  A timer fires every `tick` ms.  Each tick multiplies
   velocity by the configured decay, accumulates the result, and sends
   any whole scroll units to the host.  Real events that arrive in
   the same direction get absorbed; events in the opposite direction
   cancel inertia.
5. **Stopping.**  Inertia ends when velocity drops below `stop`, when
   the layer turns off, or when the `span` safety cap is reached.

There's also a fallback "stop detect" timer for the case where the ball
goes from full speed to zero with no gradual slowdown to detect.

---

## Project status

This project is feature-complete for its intended purpose.

I may update it for critical bugs or major ZMK breaking changes, but
I do not plan frequent feature additions.

---

## License

MIT
