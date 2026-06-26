# Recommendations (ranked)

The campaign's failure is **control-authority starvation at the actuator**, not
tuning. Fix the authority first; tuning on top of a clip nonlinearity will keep
chasing its tail.

## P0 — restore thrust & control-authority margin

1. **Fix the power/weight margin (root cause).** Re-weigh the craft and
   bench-test static thrust per motor — *especially the previously burned arm*.
   Target a **hover throttle ≤ 0.4** with four healthy, matched motors. Until
   hover sits well below mid-stick, no controller has symmetric authority and the
   limit cycle is unavoidable. Evidence: [motor-analysis.md](motor-analysis.md),
   [session-analysis.md](session-analysis.md).

2. **Raise `MOTOR_IDLE_FLOOR`** from `0.005` to a real idle (~0.05–0.08)
   (`firmware/include/variables.h:122`). A genuine idle keeps ESCs spinning and
   restores *bidirectional* low-side authority so the mixer is not clipping
   against ~0. Re-verify it does not break the SITL gravity-cancel "armed-alive"
   signalling noted in the `angle_rate_controller.c` idle-floor comment.

## P1 — make the mixer fail gracefully

3. **Add proper air-mode / authority prioritisation.** When the demanded
   differential exceeds headroom, *raise the collective* to make room for
   attitude rather than silently scaling the differential to zero — or
   explicitly limit the rate setpoint to what the current headroom can serve, so
   the loop never commands authority it cannot get. Today's scaler
   (`angle_rate_controller.c:435–466`) trades attitude away invisibly, which is
   what closes the relay loop.

## P2 — then, and only then, re-tune

4. **Re-tune on a craft that can actually hover.** The k6 INDI seed is the right
   starting point once authority exists; re-run the b-fit when the plant can hold
   altitude calmly (the contaminated-fit catch-22 disappears once it is not
   hunting). Do **not** re-tune before P0/P1.

## P2b — calibrate the simulator to reality (so it stops lying)

The sim currently flies because its plant is **3–7× too weak and ~1.8×
over-powered** vs the real airframe (measured fresh, [`sim_parity/`](sim_parity/README.md)).
Until it's calibrated, "it works in SITL" is not evidence the real craft will fly.
Apply the modeled parity parameters to the vsim geometry:

- **`I_xx → 0.0023` (×0.33), `I_yy → 0.0011` (×0.14)** — match the measured real
  effectiveness `K` (roll 563, pitch 1381 deg/s²/u); the I_yy/I_xx ratio also
  encodes the real 2.45× pitch/roll asymmetry.
- **Thrust margin → hover ≈ 0.45** (`k_thrust ×0.31` or `mass ×3.2`).
- **Actuator `tau → ~21 ms`** (onhw sysid), **per-motor `k_thrust` asymmetry**
  (weaken the burned arm — needs a props-off bench thrust measurement).
- **Add the missing model features** (the ones that actually reproduce the real
  cycle, per the full plant ID in [`plant_id/README.md`](plant_id/README.md)):
  a **~100 ms actuator transport delay** + **idle-stall / ESC deadband**
  (thrust→0 below ~0.05–0.08 duty, ~80–120 ms re-spin) in `motor_model.cpp`, and
  **thrust-scaled vibration injection** (accel σ ≈ 0.3–1.5 g) in the sim IMU path.
  The delay is the identified root of the 1.9 Hz cycle; the sim has no
  transport-delay element today.

A calibrated sim must pass objective gates (`plant_id/README.md` §6): hover
0.42–0.48, pitch plant phase ≈ −175° at 1.9 Hz, a **bounded ~2 Hz pitch limit
cycle with roll stable**, and vibration 0.1→1.5 g. Then P0–P2 fixes are validated
in SITL before flying. (Caution: matching effectiveness by inertia *alone*
oscillates at the wrong 18 Hz mode — the transport delay is what sets 2 Hz.)

## P3 — clean up the estimate

5. **Prop-balance / soft-mount the IMU** to bring the 0.2–1.5 g accel vibration
   down ([sensor-analysis.md](sensor-analysis.md)) before trusting the attitude
   estimate in aggressive flight.

6. **(Optional) raise the telemetry IMU rate or add onboard vibe logging** so the
   vibration *spectrum* (not just amplitude) is observable — the current ~48 Hz
   stream aliases prop-band content.

## What NOT to keep chasing

- Gain tuning of the existing PID — proven gain-invariant across a 2.4× sweep.
- INDI bandwidth alone — k6 only "works" by going limp; it is not flying.
- The front/back trim asymmetry as a primary cause — it is a minor bias; the
  dominant problem is bidirectional saturation.
