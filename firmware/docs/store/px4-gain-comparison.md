# PX4 default-gain comparison → rate/attitude retune (2026-07-14)

Reference comparison of the vayu default PID gains against PX4-Autopilot's
multicopter defaults, done to explain (and fix) the ~8 Hz saturation/relay limit
cycle seen in the `~/vayu-logs/export-20260713-015023.bin` hand-test log (roll and
yaw outputs slamming ±1 while the craft was held roughly steady).

## Unit convention (the whole point)

- **PX4** rate loop works in **rad/s** (`MC_ROLLRATE_P` = "output per 1 rad/s error"),
  attitude P in **rad** (`MC_ROLL_P` = "rad/s rate-setpoint per 1 rad error").
- **vayu** rate loop works in **deg/s** (gyro is `bmx160_raw_gyr_to_dps`, PID output
  ±1); the angle-loop P maps angle-error → rate-setpoint in the same deg units.

To compare rate gains, multiply vayu's per-deg/s gains by **57.2958** (deg/rad).
Attitude-P is a dimensionless ratio (1/s) and compares directly.

## Rate loop (inner)

| gain | vayu native (/dps) | vayu → /rad | PX4 | vayu vs PX4 (old) |
|---|---|---|---|---|
| Roll P  | 0.012  | 0.69  | 0.15  | 4.6× |
| Roll D  | 0.00025| 0.014 | 0.003 | 4.8× |
| Pitch P | 0.008  | 0.46  | 0.15  | 3.1× |
| Pitch D | 0.0008 | 0.046 | 0.003 | **15×** |
| Yaw P   | 0.018  | 1.03  | 0.2   | 5.2× |
| (I gains) | — | — | — | 0.9–4.6× |

PX4 roll/pitch rate: P 0.15, I 0.2, D 0.003, K 1.0. Yaw: P 0.2, I 0.1, D 0.0.

## Attitude loop (outer, unit 1/s, direct)

| gain | vayu (old) | PX4 | ratio |
|---|---|---|---|
| Roll/Pitch P | 1.6898 | 4.0 | 0.42× (sluggish) |
| Yaw P | 1.2 (unused; yaw is rate-controlled) | 2.8 | — |

## Diagnosis

Mismatched cascade: **inner (rate) loop 3–15× hotter than PX4** under an **outer
(angle) loop ~0.4× PX4**. The hot rate P saturates the ±1 output at ~83 dps error
(relay limit cycle); the oversized rate D (5–15× PX4) became fully active once the
D-LPF was opened 0.3 s → 0.004 s (~40 Hz, see `cmd-set-d-lpf`), feeding the 8 Hz.

## Retune applied (variables.h, defaults only — no pid.bin magic bump)

| gain | old | new | ≈ PX4-equiv |
|---|---|---|---|
| Roll rate P  | 0.012  | **0.003**   | 1.15× |
| Roll rate D  | 0.00025| **0.00005** | 0.96× |
| Pitch rate P | 0.008  | **0.0025**  | 0.95× |
| Pitch rate D | 0.0008 | **0.00005** | 0.96× |
| Yaw rate P   | 0.018  | **0.004**   | 1.15× |
| Roll angle P | 1.6898 | **3.0**     | 0.75× |
| Pitch angle P| 1.6898 | **3.0**     | 0.75× |

**Left as-is:** all rate I-gains (they don't drive an 8 Hz limit cycle; slower to
act — trim in flight test if overshoot appears), yaw angle P (unused), the 0.004 s
D-LPF, and output/i_max clamps.

PX4 defaults are for a generic 250–450 mm quad, not this rig — treat as sanity
bounds, then flight-test / autotune from here.
