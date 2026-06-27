# Recommendations (ranked)

The campaign's failure is **control-authority starvation at the actuator**, not
tuning. The preceding [reference-autopilot comparison](reference-autopilots-comparison.md)
confirms that diagnosis against two shipping autopilots and locates, in their
source, the exact mechanism that fixes each problem; this section is the ranked
action plan drawn from it. **Fix authority first** — tuning or filtering on top of a
clip nonlinearity will keep chasing its tail.

> These supersede the original campaign recommendations: they now account for the
> PX4/ArduPilot comparison, the fact that **these ESCs support neither DShot nor RPM
> telemetry**, and the [resource budget](reference-autopilots-comparison.md#10-resource-analysis--cpu--memory-impact-of-the-proposed-changes)
> on the F401.

## P0 — restore thrust & control-authority margin

1. **Fix the power/weight margin (root cause, hardware).** Re-weigh the craft and
   bench-test static thrust per motor — *especially the previously burned arm*.
   Target **hover throttle ≤ 0.4** with four healthy, matched motors. Until hover
   sits well below mid-stick, no controller has symmetric authority and the limit
   cycle is unavoidable ([motor-analysis.md](motor-analysis.md), comparison §9.7).
   Software cushion *afterwards*, not instead: an ArduPilot-style `thrust_boost`
   re-weight of the healthy motors on a degraded airframe.

2. **Raise `MOTOR_IDLE_FLOOR`** from `0.005` to a real idle (~0.05–0.08)
   (`firmware/include/variables.h:122`). This removes the **transport-delay source**:
   a near-zero idle lets a real ESC stall/desync, and the ~80–120 ms re-spin is the
   ~109 ms delay that sets the 2 Hz cycle ([plant_id](plant_id/README.md) §2). Both
   references keep a genuine min-spin (ArduPilot `MOT_SPIN_MIN`≈0.15; PX4 spool min —
   comparison §9.2). Re-verify it does not break the SITL gravity-cancel
   "armed-alive" signalling noted in the `angle_rate_controller.c` idle-floor
   comment, and confirm the delay shrinks in the twin (`VSIM_STALL_DUTY`).

## P1 — make the mixer preserve attitude under saturation (the hover-blocker)

3. **Replace the uniform anti-sat scaler** (`angle_rate_controller.c:435–466`) with
   an **airmode** policy. Today it shrinks the *whole* attitude differential to keep
   pilot throttle — sacrificing roll/pitch, the one choice that cannot hover an
   authority-starved craft. Port the reference behaviour (comparison §9.1): when the
   demanded differential exceeds headroom, **raise the collective to make room**
   (PX4 `mixAirmodeRPY`) and/or **sacrifice yaw before roll/pitch** (ArduPilot
   `MOT_YAW_HEADROOM` + the `rpy_scale` vector shrink). Keep feeding the realised
   post-clip differential into the INDI/PID anti-windup (vayu already does this for
   INDI at `:511`). **This is the single highest-leverage change.** It must pass the
   sim gates ([plant_id](plant_id/README.md) §6): kill the reproduced 2 Hz cycle with
   roll stable.

4. **Feed measured `dt` to the rate loop.** The inner loop uses a constant 1 ms;
   INDI is dt-sensitive (comparison §9.4). Wire the DWT-measured Δt — already stamped
   in `bmx160.c` — into the rate/INDI update. Cheap; ship it alongside #3.

## P2 — re-tune only on a craft that can hover

5. **INDI b-fit, or PID fallback.** The k6 INDI seed is the right starting point once
   authority exists; re-run the b-fit when the plant can hold altitude calmly (the
   contaminated-fit catch-22 disappears once it is not hunting). If `b`/`dt` are not
   yet solid, the compiled **rate-PID path is the more robust baseline** for the
   authority work — PX4/AP both ship PID for exactly this tolerance (comparison §9.5).
   Do **not** re-tune before P0/P1.

## P2b — calibrate the simulator to reality (so it stops lying)

6. **Apply the digital-twin parameters** ([sim_parity](sim_parity/README.md),
   [plant_id](plant_id/README.md)) so SITL *fails the way the real craft fails*
   (3–7× too weak / over-powered today, no transport delay). Then validate P0–P1 in
   SITL before flying. Until calibrated, "it works in SITL" is not evidence.

## P3 — clean up the estimate (only after it hovers)

7. **Add an FFT-driven harmonic notch on the gyro.** vayu has no notch and the
   campaign measured 0.2–1.5 g vibration ([sensor-analysis.md](sensor-analysis.md)).
   Decisive constraint: **these analog ESCs give no RPM telemetry**, so RPM-tracking
   is impossible and throttle-estimation is a crude proxy — **an FFT of the gyro is
   the only true in-flight frequency source** (comparison §9.3, §10.5). Implement it
   **self-contained, no CMSIS link**: an in-tree radix-2 FFT + const twiddle/window
   tables, **FPU-gated** (`VAYU_FFT_NOTCH` on `_FPU_ENABLED`, static/throttle fallback
   on no-FPU MCUs), heap-allocated scratch, run on a background task — full design in
   §10.6. Minimal config (N=64–128, decimated ~1 kHz) ≈ 0.5–1.5% CPU and ~2–4 KB heap.
   Ship a **static notch first**, then the FFT-driven one.

8. **Fix the output dispatch (secondary).** DShot is unavailable (ESCs don't support
   it), so this is not a protocol change — keep analog PWM but **event-drive the motor
   task off the rate-loop publish** (kill the ~500 Hz poll) and **raise the PWM rate
   to the ESC max**. ~Halves the output latency, no new buffers (comparison §9.6).

9. **Prop-balance / soft-mount the IMU** to bring the 0.2–1.5 g vibration down before
   trusting the attitude estimate in aggressive flight; optionally raise the telemetry
   IMU rate so the vibration *spectrum* (not just amplitude) is observable.

## Resource note

All code changes are cheap on the F401 (comparison §10): together **~1.5–2.5% CPU**
(against the EKF's existing ~17.5%) and **~3–5 KB**, **placed on the abundant heap**
(`v_malloc` at init) so `.bss` stays flat and the boot-HardFault invariant is never
approached. **No change needs DShot DMA buffers or a CMSIS link.**

## Sequencing

Authority before everything: **#1 power/weight + #2 idle floor → #3 airmode mixer
(+ #4 measured dt) → #6 sim calibration as the validation gate → then #5 re-tune, #7
notch, #8 output** once the craft holds altitude. Each code item passes the
[plant_id](plant_id/README.md) §6 sim gates before flying.

## What NOT to keep chasing

- **Gain tuning of the existing PID** — proven gain-invariant across a 2.4× sweep.
- **INDI bandwidth alone** — k6 only "works" by going limp; it is not flying.
- **The front/back trim asymmetry as a primary cause** — a minor bias; the dominant
  problem is bidirectional saturation.
- **Loop latency / faster output as a hover fix** — vayu's loop is already the
  lowest-latency of the three (comparison §8); the output fix is a latency cleanup,
  not the hover fix. Authority handling is.
