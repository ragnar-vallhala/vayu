# Recommendations & next-run plan — `export-20260617-124210.bin`

What to fix before the next trial, and exactly what to capture when you re-run.
Derived from the analysis in this folder
([session](session-analysis.md) · [control](control-loop-analysis.md) ·
[motors](motor-analysis.md) · [kernel](kernel-analysis.md) ·
[sensors+fusion](sensor-analysis.md)).

> **Scope: stay on the rig.** No free flight yet. The goal of the next run is to
> get a **stable, well-instrumented rig session** — fix the oscillation and prove
> it on the bench first. Everything below is rig-only.

## 1. Fixes before the next rig trial (priority order)

### 1a. Blocker — the attitude-loop oscillation
This is the one that must change before anything else; right now the roll/pitch
loop diverges into a limit cycle whenever throttle is held up.

- **Re-tune the roll/pitch rate loop for damping.** The current tuning is the
  likely cause: almost no proportional term and **no derivative (Kd = 0)**, leaning
  on a slow integrator. Add some rate **Kd** (damping), raise **Kp** so it isn't
  integral-dominated, and reduce **Ki** / the integral clamp.
- **Change one thing at a time** and re-test on the rig after each step — small,
  conservative moves, not a big jump.
- **Tune one axis first** (roll or pitch, whichever the rig lets you isolate),
  confirm it's stable, then mirror to the other.
- Treat **yaw separately** — it's a different (much stronger) loop and showed its
  own possible limit-cycle; only revisit it after roll/pitch is calm.

### 1b. Sensor calibration (do before tuning, so the loop sees good data)
- **Accelerometer 6-side calibration** — removes the ~8 % scale error.
- **Magnetometer calibration with the frame powered** (motors drawing current) —
  the mag is uncalibrated and currently makes the fused *yaw/heading* untrustworthy.
- **Re-verify after**: at rest `|accel|` should read ~g, and `|mag|` should be
  roughly constant as you rotate the frame.

### 1c. Telemetry fields — so the *next* log is fully analysable
These are what made parts of *this* log hard to read. Fix them and the next
capture tells you far more:

- **Populate the fused body rates** (attitude angular-rate fields are all zero
  today) — without them you can't see the rate loop directly; it's the single most
  useful fix for tuning analysis.
- **Populate the per-sample IMU timestamp** (zero today) so IMU can be time-aligned.
- **Fix the compressed-IMU keyframe reference** (stuck at zero) so the high-rate
  IMU stream can be reconstructed.
- **Populate or drop CPU-load** (zero today; real timing lives in the perf stream).
- **Resolve the degrees-vs-radians inconsistency** between the control trace and
  the attitude message (or document per-field units).

### 1d. Telemetry link
- The downlink was **saturated** (continuous TX-buffer overflow, telemetry FIFOs
  100 % full). For a focused tuning run, **prioritise the streams you need**
  (control trace, attitude, IMU) and trim the rest, or raise the link bitrate.
- **Widen the FIFO drop counter** (it saturates at 65535, so true drops are
  unknown).

## 2. What to capture in the next rig run (test protocol)

Run these as distinct, labelled phases so each is easy to find in the log. Keep a
**hard throttle cap** and pull throttle the moment oscillation grows.

1. **Calibration pass** — run accel/gyro/mag calibration and capture it (the
   calibration-status messages + the `CALIBRATING` state). Confirm it stores.
2. **Static baseline** — armed, motors at idle, frame held at its level reference
   on the rig for ~15–20 s. Confirms post-cal sensor health and the resting pose.
3. **Throttle-step sweep (the key test).** With sticks centred, raise throttle in
   small steps (e.g. ~0.15 → 0.20 → 0.25 → 0.30 → 0.35), **holding each step ~10 s**.
   Watch for the oscillation-onset throttle. After re-tuning, the goal is **no
   growing oscillation at any step** — this is how you prove the fix.
4. **Disturbance-and-release.** At a safe, steady throttle, nudge the frame a few
   degrees by hand and let go; capture the recovery. A good tune **settles with
   decaying overshoot**; a bad one keeps ringing. Repeat on roll and pitch.
5. **Yaw check.** Hold yaw, then give one controlled yaw nudge and release —
   confirm it returns without a sustained spin/limit-cycle.

> Keep the **same rig setup** between runs (which axis is free, CG, props on/off)
> so results are comparable, and note it.

## 3. What specifically to capture (states & methods)

So each capture is self-documenting and the oscillation is well-sampled:

- **Record the active PID gains in the log itself** (read them back over the link
  at the start of the run). Then every capture says *which tuning* it tested — no
  guessing later.
- **Mark the phases.** Drop a marker at each phase boundary — a status-text line,
  or a dedicated RC-switch toggle, or just written-down timestamps — so phases 1–5
  above are trivially segmentable.
- **Log the state machine and command acknowledgements** (`nav_state`
  transitions, arm/cal/set-gain ACKs) so you can confirm what was actually applied.
- **Raise the rates of the signals that matter** for the duration of the test:
  the control trace, attitude (with the now-populated body rates), and full IMU.
  The oscillation is slow (~0.3–1.2 Hz), so even modest rates resolve it — but the
  IMU keyframe was very sparse (~1.6 Hz) last time; bump it.
- **Keep time-sync running** so timestamps line up across streams.
- **Note rig/environment** out-of-band: tether, CG, whether props are on, supply
  voltage.

## 4. Rig acceptance criteria (gate before *any* flight thinking)

Consider the rig "passed" only when, in one capture:

- [ ] Throttle held through the full sweep (incl. ≥ the full-authority point) for
      ≥ ~30 s with **no growing oscillation**.
- [ ] Disturbance-release **settles** (decaying, no sustained limit cycle) on roll
      and pitch, and yaw returns cleanly.
- [ ] Sensors calibrated: `|accel|` ≈ g at rest, `|mag|` ~constant when rotated.
- [ ] Telemetry fields populated (body rates, IMU timestamp) and link not saturated.

Only after all four — and reviewing that capture — move to a **tethered / very
low** check. **Open free flight stays off the table until the rig is boring.**
