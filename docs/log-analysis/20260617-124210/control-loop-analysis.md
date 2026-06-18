# Control-loop analysis — `export-20260617-124210.bin`

Cascade controller behaviour from `ControlTrace` (msgid 1030, 7,450 frames),
restricted to the four **ARMED** windows. **Context: this was a bench-rig run —
the airframe never flew (`IN_AIR` never reached) and was hand-spun/clamped in
yaw.** Read accordingly. Firmware refs ground every interpretation. Companion to
[`session-analysis.md`](session-analysis.md) and
[`motor-analysis.md`](motor-analysis.md).

Units (firmware `control_telemetry_t`, `include/variables.h:243`): **angles in
degrees, rates in deg/s, `*_out` ∈ [−1, +1]** feeding the mixer.

## Architecture (from firmware)

Cascade, two rates:

```
RC/setpoint ─► ANGLE loop (outer, 250 Hz, Kp only) ─► rate_sp (clamped ±100 °/s)
                                                          │
              gyro ─► RATE loop (inner, 1000 Hz, PI) ────► *_out ∈ [−1,1] ─► mixer
```

- Outer `OUTER_LOOP` = 250 Hz (`outer_dt` = 4.000 ms, **zero jitter** in log).
- Inner `INNER_LOOP` = 1000 Hz (`inner_dt` = 1.000 ms, **zero jitter**).
- Default gains (`include/variables.h`):
  - **Angle (outer):** roll/pitch Kp = 4.0, out clamp ±100 °/s. Yaw is
    rate-controlled (no outer angle loop in flight).
  - **Rate (inner):** roll/pitch **Kp = 0.0005**, Ki = 0.01, Kd = 0, i_max 0.2.
    **Yaw Kp = 0.018**, Ki = 0.008 — i.e. **yaw P-gain is 36× roll/pitch**.
  - **PID-authority ramp:** outputs scale from 0 at `MIN_ARMED_THROTTLE` (0.10)
    to full at `PID_FULL_AUTHORITY_THROTTLE` (0.30) to stop ground feedback.

## Loop timing — healthy

`outer_dt` = 4.000 ms and `inner_dt` = 1.000 ms with **zero variance** across all
7,450 frames. The 250 Hz / 1 kHz cascade is keeping schedule exactly (corroborated
by the kernel `EstPerf` 250 Hz and `PerfTask` timing — see
[`kernel-analysis.md`](kernel-analysis.md)). No timing pathology.

## The "level command but tilted" finding

With a **level command** (|roll_sp|, |pitch_sp| < 2°) and throttle up, the
airframe sat at:

| | mean | std |
|---|---:|---:|
| roll_angle_curr | **−9.7°** | 22.6° |
| pitch_angle_curr | **+25.1°** | 16.3° |

So pitch held ~**+25° nose-up** against a 0° command. Tracing the cascade in
these frames shows the controller is doing the *right thing in the right
direction* — it just has almost no authority:

```
pitch angle error  -25°  ──Kp=4──►  pitch_rate_sp  -66 °/s   (outer loop OK, commands nose-down)
pitch rate error   -66 °/s ──Kp=5e-4──► pitch_out  -0.023    (inner loop output ~zero)
```

The inner-loop output is tiny **by design**: empirical output/error slope is
**0.00033** for pitch and **0.00034** for roll — matching the firmware
`Kp = 0.0005` (the rest leans on the slow Ki = 0.01 integral, clamped at 0.2).
With Kd = 0 there is no rate damping either. Net: roll/pitch have very low
proportional authority.

This is compounded by the **authority ramp**: armed throttle averaged **0.25 and
was below the 0.30 full-authority threshold 100 % of the time**, so roll/pitch
output was *additionally* attenuated (≈0.5×) the entire session.

**Conclusion.** The +25° pitch is a *real* tilt (confirmed independently — fused
attitude matches the gravity vector, see
[`sensor-analysis.md`](sensor-analysis.md) §Fusion), and the attitude controller
could not pull it level because (a) it ran on the rig where the frame is
mechanically constrained, (b) roll/pitch rate authority is intentionally tiny
(Kp 5e-4, Kd 0) and integral-dominated, and (c) throttle never exceeded the 0.30
full-authority point. **On a rig this is expected; it does not by itself prove a
tuning fault — but it does mean the bench cannot demonstrate level-hold, and the
integral-only roll/pitch authority is worth validating in a real hover.**

## Yaw — disturbance rejection, saturation, and the hand-clamp episodes

Yaw was **rate-controlled with `yaw_rate_sp` = 0 for the entire session** (pilot
never commanded yaw). Every bit of yaw motion is therefore a *disturbance* — the
rig being spun by hand, which the user confirms ("yaw suddenly starts to rotate,
which I sometimes hand-clamped"). `yaw_rate_curr` ranged **−80…+68 °/s**.

Detected yaw episodes (|rate| > 40 °/s) fall in two regimes:

- **t = 0–5 s, ±55–59 °/s, `yaw_out` = 0.00.** Free swings with *no* control
  response — because throttle was near idle, the authority ramp gated the output
  to zero. The frame spins freely on the rig.
- **t = 112–123 s, 216–233 s, 330–351 s, up to ±80 °/s, `yaw_out` ±0.6…0.76.**
  Here the controller fights hard. `yaw_out` saturates (|·| > 0.9) in 0.5 % of
  armed frames, and the **343–351 s** stretch is a run of alternating ±65–80 °/s
  spins on a ~2 s period — consistent with repeated hand-spin → clamp cycles
  (and/or a yaw limit-cycle as the strong yaw loop, Kp 0.018, fights the rig).

Because yaw's gain is 36× roll/pitch, yaw is the only axis with meaningful
authority in this log — which is exactly why yaw shows large `*_out` while
roll/pitch sit near zero. The empirical yaw slope (0.0117) matches `Kp = 0.018`
(reduced by integral/clamp/ramp).

## Takeaways

1. **Timing is perfect** (4 ms / 1 ms, zero jitter) — no scheduler/loop problem.
2. **The bench tilt is real and unforced by the controller**; fusion is accurate.
   Roll/pitch leveling can't be judged from a rig run, especially one held below
   30 % throttle the whole time.
3. **Roll/pitch rate authority is very low** (Kp 5e-4, Kd 0, integral-dominated).
   Validate level-hold and disturbance recovery in an actual hover before trusting
   it; consider whether Kd = 0 is intended.
4. **Yaw loop is ~36× stronger** and saturates against rig disturbances; the
   343–351 s alternating spins deserve a look for a possible yaw limit-cycle once
   off the rig.
5. Next capture: fly it (`IN_AIR`), command non-zero roll/pitch/yaw, and keep
   throttle above 0.30 so the controller runs at full authority.
