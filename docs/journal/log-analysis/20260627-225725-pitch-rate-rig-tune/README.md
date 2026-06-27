# Pitch rate-loop rig tune — 2026-06-27

Single-axis **rig** tuning session for the inner (rate) **pitch** PID, run live over the
NavLink/UDP bridge with `CmdSetPid` (no reflash). Inner loop was switched to PID this
session (`RATE_CTRL_ALGO_USED = RATE_CTRL_PID`). Result is persisted in `pid.bin`.

## Final tune (validated)

Firmware **RATE pitch** slot (axis 1):

| | Kp | Ki | Kd | Kff | D-LPF RC |
|---|--:|--:|--:|--:|--:|
| **final** | **0.008** | **0.003** | **0.0008** | 0.0 | 0.004 s |

`pid.bin` in this folder is the exact 156-byte PID3 file (magic `0x50494433`): only the
RATE-pitch slot is `valid`; every other slot is invalid → firmware uses compiled defaults.
Restore by copying to the SD root as `0:pid.bin`, or re-push with `CmdSetPid`.

## How we got there (methodology)

The default pitch loop (`Kp 0.012`) drove a **saturation / relay limit cycle**: at the
oscillation amplitude the P term alone (`0.012 × ~158 ≈ 1.9`) exceeded the ±1 output
clamp, so the actuator slammed rail-to-rail and self-sustained a ~3 Hz cycle. Fix arc:

1. **Kp down** → stop the rails slamming (kills the relay cycle).
2. **Add Kd** → actively damp the residual rate oscillation (we had headroom).
3. **Kp back up** → tighten the angle hold, with Kd keeping it off the rails.

## Four-way comparison (engaged window, throttle > 0.3)

| metric | `0.012 / 1e-4` (default) | `0.005 / 0` | `0.005 / 8e-4` | **`0.008 / 8e-4` (final)** |
|---|--:|--:|--:|--:|
| pitch_out **saturation** | 37 % | 0 % | 0 % | **1 %** |
| pitch_rate **RMS** (deg/s) | 74 | 83 | 40 | **22** |
| pitch_rate max (deg/s) | 158 | 203 | 107 | **108** |
| **angle hold RMS** (deg) | — | 25 | 27 | **13** |
| angle range (deg) | — | −19..84 | −60..43 | **−42..37** |
| pitch_out RMS (effort) | 0.73 | 0.47 | 0.24 | **0.38** |
| engaged window | — | ~8 s | ~18 s | **~30 s** |

Final = lowest oscillation, tightest angle hold, ~no saturation. Kd was the key: it cut
rate RMS while *lowering* control effort (signature of real damping). Roll/yaw stayed calm.

## Captures (raw VREC, `~/vayu-logs/`)

| file | gains | note |
|---|---|---|
| `export-20260627-230732.bin` | 0.012 / 1e-4 | baseline — the 37 %-saturation 3 Hz relay cycle |
| `pitch-detune-20260627.bin` | 0.005 / 0 | light throttle (~0.31), 0 % sat but loose |
| `pitch-detune-hithr-20260627.bin` | 0.005 / 0 | full throttle, 18 s — 0 % sat confirmed |
| `pitch-kd-20260627.bin` | 0.005 / 8e-4 | Kd added, rate RMS halved |
| `pitch-kp8-20260627.bin` | 0.008 / 8e-4 | **final**, 30 s — copied here as `capture-final-kp8.bin` |

Decode: `python3 tools/telemetry/analyze_pitch_osc.py <file>`.

## Caveats / scope

- **Rig = single free axis.** The runaway-on-arm at thr 0 was confirmed **open-loop**:
  all four motors commanded an identical 0.150 (idle floor), controller gated below 0.30
  throttle → a constant motor/rig-balance torque spun the free axis up unopposed. Not a
  control/sign/mixer fault. Balance the thrust/props/rig-pivot separately.
- **Physical-axis naming:** the rig's free axis read on the firmware **pitch** channel
  (what the operator called "roll"). Gains here are for firmware RATE **pitch** (axis 1).
- **Only pitch was tuned.** RATE roll/yaw and all ANGLE slots remain compiled defaults.
  Roll stayed calm here but its default `Kp 0.012` is the same value that saturated on
  pitch — give roll the same rig treatment before relying on it in free flight.
- Tuned on a constrained rig, not in hover — treat as a strong starting point, re-verify
  in flight.
