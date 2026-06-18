# Findings — live maneuver demo (2026-06-18)

> **Open issue:** the ~12× attitude under-read in powered, translating flight is
> the one thread still unresolved here. It is tracked separately in
> [`docs/journal/deferred/01-attitude-estimate-underread.md`](../../../../docs/journal/deferred/01-attitude-estimate-underread.md);
> the analysis below (UPDATEs pt 2/3 + appendix) is its working record. Bugs 1 & 2
> below are also still open; only Gap 3 (guidance) has landed.

Driving the example maneuvers **through a live `vayu-headless serve` session that
the Navigator GCS was attached to** (Attach Ext + "SITL UART2"), instead of the
standalone scripts. This surfaced two concrete SDK bugs and re-confirmed the
outer-loop guidance gap with hard numbers. The rig system-ID path is sound; the
free-flight trajectory path is blocked by guidance, not by the example code.

## What was run

| Maneuver | Mode | Result |
|---|---|---|
| `step_response` (roll, amp 0.4) | tuning rig, standalone | ✅ Clean first-order rise to **+6.4°**, settled **~1.4 s**, clean return on release. 223 samples. Body rate `wx` peaked ~0.24 rad/s then decayed. |
| step (roll) | free-flight, driven via session | ✅ Visibly banked ~6° (matches the rig number), but `rc`-driven so it drifts (no altitude hold while active). |
| figure-8 (8 m, 14 s/lap) | free-flight, driven via session | ❌ Did not track — diverged to N∈[−12.7, +16.3], E up to +21 m on a ±8/±4 m commanded path. |
| figure-8 (5 m, 30 s/lap, gentler) | free-flight, driven via session | ❌ Still diverged — mean track error **16 m**; began ~19 m off-centre. |
| `chirp`, `doublet` | — | Not run live (rig data tools; standalone smoke-tested earlier). |

Note on the angle scale: a 0.4 roll command produced ~**6.4°**, not the ~12° the
example's "30°/full" label assumes — either the stick→setpoint scale or the rig
tether. Worth confirming against `MAX_ANGLE`/RC scaling before trusting the
label.

## Bug 1 — `goto`/`fly` don't re-engage guidance after `rc`

`rc` sets `pilot.active = False` (raw stick passthrough), but `Pilot.goto()` only
mutates the waypoint list — it never sets `active = True`. So after any `rc`
command the only way to re-engage the outer loop is `takeoff` (which respawns).
During the demo this looked like a "frozen" craft that wouldn't recover.

- Where: `vayu_headless/autopilot.py::Pilot.goto` (and the `goto`/`fly` branch in
  `vayu_headless/server.py::SessionServer.dispatch`).
- Fix: `goto`/`fly` should set `active = True` (issuing a go-to *is* a request to
  fly there). Low risk; add a unit assertion that `goto` re-engages.

## Bug 2 — `status` reports a stale guidance snapshot

`server.py::status_dict` reads `pilot.last` (populated only inside
`Pilot._step`, i.e. only while `active`). With guidance off (after `rc`), `last`
freezes, so `status` shows stale pose even though the craft is still flying and
`SitlSession.truth()` is updating live. This masquerades as a physics freeze.

- Where: `vayu_headless/server.py::status_dict`.
- Fix: source pose/alt/att from the live `lab.truth()`; keep `pilot.last` only for
  guidance-specific fields (target/wp). Then `status` is honest in every mode.

## Gap 3 — outer-loop position guidance — RESOLVED

**RESOLVED (2026-06-18, pt 4)** → `vayu_headless/autopilot.py::guidance_outputs`
(commit `d9df605`, validated by `software/headless-sdk/fidelity/`). Velocity
feedforward + anti-windup position integrator + raised tilt authority took the
lemniscate from **16 m → 1.7 m** mean track error (box fidelity 46→77, hover
drift 1.24→0.32 m, overall 76.6→83.2; 48/48 tests pass). Residual tracking error
is now bounded by the attitude under-read (deferred #1) below, not by guidance.

## Gap 4 — free-flight + collision world = pinball

Once the craft strays into the loaded course (testcourse.glb, restitution 0.3),
it bounces off the collision mesh, which turns the guidance wander into violent
±17 m excursions. This compounds Gap 3 and makes free-flight divergence look
worse than the guidance alone would.

- Recommendation: tune/validate the outer loop in a **collision-free world**
  first (isolate guidance from collision response), then re-introduce the mesh.

## UPDATE (2026-06-18, pt 2): the real root cause is a SITL fidelity violation

Driving the maneuvers through a live GCS-attached session and logging the full
chain (commanded stick → RC received → FC angle setpoint/measured → est vs TRUE
attitude → position) overturned the guidance theory entirely:

- True pitch swung ±17° during a dash while the **estimated** pitch read ±1–2°.
  The gyro RATE was correct (`ControlTrace` rate == truth), but the integrated
  ANGLE came out ~1/12. Accel was exonerated (hard-disabling it changed nothing).
- Root cause: **in SITL the firmware's attitude estimator never ran.**
  `host_imu_feeder.c` computed attitude with its OWN `m_mahony_filter` and pushed
  it into the attitude queue; `host_lifecycle.c` started the control tasks but
  not `attitude_task`. So SITL validated a host-side shim, not the shipped EKF —
  a fidelity violation that defeats the point of the harness.

**Fix applied (this commit):**
- `host_imu_feeder.c`: removed the host mahony + attitude pushes; it now injects
  RAW IMU only (`imu_queue_attitude_push`), and stamps a FIXED sensor cadence
  (`SITL_IMU_FEED_HZ`, sim-time) instead of wall-clock — wall-clock jittered/
  collapsed the estimator's dt when the vsim FIFO bursts.
- `host_lifecycle.c`: starts the real `attitude_task` (EKF/Mahony per
  `SF_FILTER_USED`), fed raw IMU exactly as on hardware.
- `host_navhal.c`: added `hal_cycle_counter_cycles_per_us()` (84) the task needs.
- `tools/sim_host/CMakeLists.txt`: added `src/est/attitude_task.c` to the SITL core.
- Audit: the host mahony was the ONLY firmware-bypassing compute in the shims.

**STILL OPEN (exposed, not yet fixed):** with the real EKF now running, the
estimate STILL under-reads ~12× during dynamic flight. Ruled out: accel
(disabled → no change), dt (fixed-cadence, verified 1 ms), gyro units (vsim
rad/s→deg/s at main.cpp:105, EKF `to_radians` back — consistent), gyro value
(`ControlTrace` rate matches truth), and the EKF in isolation (self-test 14/14).
So the under-integration is a subtler estimator-path issue in SITL (candidates:
the `attitude_task` decimation/`dt_sum` feeding, or the queue/sample path) — the
next thing to trace. NOTE: a 2nd discrepancy also remains — firmware
`IMU_SAMPLE_FREQ_HZ=2000` vs vsim's 1000 Hz emit (`SITL_IMU_FEED_HZ` matches the
actual 1000 Hz for now; reconcile via `SET_RATES` or a config).

## UPDATE (2026-06-18, pt 3): instrumented Tests 1 + 4 — dt exonerated, accel/tilt ambiguity confirmed

Added a SITL-only diagnostic (`attitude_task.c`, guarded by `VAYU_SITL`) that logs,
per estimator step, the predict INPUT (`step_dt`, `|gyro|`) and the accumulating
state (EKF gyro bias / Mahony integral). NB: that shim was reverted out of the FC
code afterwards — re-add it per `docs/journal/deferred/01-attitude-estimate-underread.md`
to reproduce. One rig run + one free-flight figure-8:

- **Test 1 (dt/scale) — RULED OUT.** `step_dt = 0.0075–0.008 s` with `decim=8`
  (≈8 samples × 1 ms), exactly the window it should span. The gyro the filter
  integrates matches truth. So the ~12× is NOT a dt or input-scale bug.
- **Rig = healthy.** On the tuning rig (translation pinned) the estimate tracks
  truth: peak **est pitch −6.39°** vs **true −6.3°**; EKF pitch-bias sits flat at
  ~0.06°/s. Same predict code, correct result → predict integration is sound.
- **Free flight = broken, and Test 4 shows why.** In the figure-8 the body rate
  `|g|` hits 8–13°/s but the **estimate stays pinned at ±1–2°**, while the EKF
  gyro bias **winds up to ±1.5°/s and sign-flips with each lemniscate lobe**
  (25× the rig's 0.06°/s). That oscillation is the fingerprint of the **accel
  update fighting the gyro**: the thrust-aligned specific force (`|a|≈g`, passes
  the magnitude gate) reads "level," so every step the accel correction yanks
  attitude back down and dumps the residual into the bias state.
- **Synthesis.** The under-read is the **accel/tilt ambiguity in powered
  translational flight** (§D pt 2), not dt and not pure bias-windup — the bias
  windup is a *symptom* of the same corrupted accel update. Rig (no translation)
  has clean gravity → no corruption → correct. This is consistent across both
  estimators because both fuse the same thrust-aligned accel.
- **Test 4(a) reset gap — confirmed structurally.** `estimator_reset()` (which
  zeroes EKF bias/covariance + Mahony integral) is called ONLY in tests; the
  STANDBY→ARMED hard reset (`angle_rate_controller.c:254`) resets only the rate
  PID + gyro LPF, never the estimator. So a wound-up bias persists across arms.

**Hardware-cheap fixes implied (no extra per-cycle loop work):**
1. Gate/down-weight the accel update on *dynamics*, not just `|a|≈g` — e.g. skip
   when `|ω|` (or throttle/known-thrust) says the craft is maneuvering, since the
   magnitude gate can't see thrust-aligned force. A comparison, not compute.
2. Reset the estimator on arm (call `estimator_reset()` on STANDBY→ARMED) and
   bound the EKF gyro-bias state (the Mahony Ki already clamps at ±0.5 rad/s).
3. Proper long-term: velocity/model-aided specific-force prediction (thrust+drag
   model, or GPS/optic-flow velocity) to resolve the ambiguity — larger change.

## Bottom line / next actions

1. Land Bug 1 + Bug 2 — small, correct, improve every session.
2. Outer-loop guidance tuning (Gap 3) — the substantive work; gate it on a
   collision-free tracking test (Gap 4) so the two effects are separable.
3. The rig system-ID examples (`step`/`chirp`/`doublet`) are trustworthy today —
   use them for cascade ID while the outer loop is being tuned.

---

# Technical appendix — details uncovered during the investigation

Reference material for whoever picks up the open ~12× attitude under-read. Every
number here was measured this session (collision-free 10 m north position step
unless noted), file:line against the tree at commit `db9e732`.

## A. Loop topology, rates, and dt sources
- `IMU_SAMPLE_FREQ_HZ = 2000`, `INNER_LOOP_FREQ_HZ = 1000`,
  `OUTER_LOOP_FREQ_HZ = INNER/OUTER_LOOP_DECIM`. Measured `ControlTrace.inner_dt
  = 1.000 ms`, `outer_dt = 4.000 ms` ⇒ inner 1 kHz, outer 250 Hz
  (`OUTER_LOOP_DECIM = 4`).
- **Two different dt sources, and that mattered:** the control loops report a
  perfectly-constant dt (1.000/4.000 ms) ⇒ they use a **fixed nominal** dt, so a
  wrong sample timestamp never showed up there. The estimator
  (`attitude_task.c:96`) uses a **measured** dt from sample timestamps
  (`vayu_dt_from_cycles`), so it was the *only* place the wall-clock-stamp bug
  surfaced. This is why everything "looked fine" except attitude.
- `attitude_task` decimation: `ATTITUDE_DECIM = IMU_SAMPLE_FREQ_HZ /
  ATTITUDE_EST_RATE_HZ` (`attitude_task.c:45`). It accumulates `dt_sum` over
  DECIM samples, then runs the filter once with the **latest** sample's gyro and
  `step_dt = dt_sum` (`:101-117`). A prime suspect for the remaining under-read:
  using one gyro sample for a multi-sample interval, and/or `ATTITUDE_DECIM`
  computed from the nominal 2000 Hz while the feed is 1000 Hz.

## B. The three fusion filters (audited)
Selected by `SF_FILTER_USED` (`variables.h:95` = **`SF_EKF`**, 6-state — the
active one). Constants (`variables.h`, `include/est/ekf.h`):
- EKF: `EKF_ACC_GATE 1.5`, `EKF_R_ACC_DIR 2.5e-3` (6-state), `EKF_R_ACC 9.0e-2`
  (9-state), `EKF_R_MAG_YAW 1.0e-2`, `EKF_GRAVITY 9.80665`. Accel gate
  (`ekf.c:259`) is **magnitude-only**: `|‖a‖−g| > 1.5 → skip`.
- Mahony: `SF_MAHONY_KP 3.0`, `SF_MAHONY_KI 0.0025`, `SF_MAHONY_MAG_WEIGHT 0.30`.
  Tilt-error term is **"Always applied"** (`sensor_fusion.c:265`) — no accel gate
  at all.
- Complementary: `SF_COMPLEMENTARY_ALPHA 0.98`, `SF_YAW_COMPLEMENTARY_ALPHA 0.92`
  — accel term `(1−alpha)` always blended, no gate.
None reject **lateral** specific force; but see §D — that turned out not to be
the cause.

## C. Units map (this caused real diagnostic confusion)
- `AttitudeEuler` telemetry is **radians on the wire** (`navlink_tx.c:158`,
  `att_deg->roll * ATT_DEG2RAD`), while the FC's internal attitude and
  `ControlTrace` angles are **degrees**. So est-vs-true comparisons must convert
  (est_rad × 57.2958). Forgetting this made the estimator look 9° "off" when it
  was a rad-vs-deg mismatch.
- Gyro path is **consistent end-to-end**: vsim emits body rate, converts
  rad/s → deg/s with `kRad2Deg = 57.29578` before packing (`main.cpp:105-107`);
  the EKF expects deg/s and converts back via `to_radians(gx)` (`ekf.c:194/200`).
  `ekf_predict` bias `bg` is rad/s.
- `vsim_imu_frame_t` payload = 22 floats (acc, gyr, gyr_raw, mag, mag_compensated,
  mag_fusion, temp) — **no sim timestamp** on the wire. Hence the fix stamps a
  fixed cadence host-side rather than reading sim-time from the frame.

## D. Decisive experiments (the ladder that ruled things out)
1. **Accel hard-disabled** (early `return` in `ekf_update_accel`): est still
   ~1° at 17° true → **accel is not the cause**. Pure-gyro predict already
   under-integrates.
2. **Innovation-based accel down-weight** (`R *= 1+GAIN·|z−gb|²`, GAIN up to
   5000): **zero effect**, because the innovation stays tiny (~0.02). Insight:
   on a multirotor the accelerometer is **thrust-aligned (≈ body −Z)** during
   powered flight, so it *agrees* with a wrong level estimate → innovation is
   small → no accel-innovation gate can catch lateral specific force. (This is
   why a magnitude gate *and* an innovation gate both fail; it's a fundamental
   accel/tilt ambiguity, resolvable only by model- or velocity-aided estimation.)
3. **Gyro rate vs truth:** `ControlTrace.pitch_rate_curr` tracks
   `d(true_pitch)/dt` within noise (e.g. t≈3.6 s: true +9.7°/s, FC +12.3) →
   the gyro *rate* is faithful; only the integrated *angle* is wrong.
   Conclusion: ~1/12 scale on integrated gyro, independent of accel and dt
   value → a structural integration issue, not a signal issue.

## E. Measured magnitudes (data points)
- Estimate vs truth during the dash: true pitch ramps 0 → +17–19°; est pitch
  stays +1 … +1.6° → ratio ≈ **1/11 to 1/13**, consistent across runs and across
  *both* estimators (host mahony and firmware EKF).
- Stick→angle scale: a 0.30 roll/pitch stick saturates the FC angle setpoint at
  **±2.70°** (`ControlTrace.*_angle_sp`); full stick ≈ ±9°. On the **rig**, a 0.4
  step produced **+6.4°** true (settle ~1.4 s) — i.e. true > the ~3.6° commanded,
  consistent with the controller over-driving against an under-reading estimate
  even on the rig (translation pinned just hid the divergence).
- Free-flight: 10 m north step overshot to ~12.5 m and **settled at ~7 m** (not
  10) — a steady-state miss + oscillation, all downstream of the bad attitude.
- Station-keep: stable to **sub-meter over ~6 s**, but wandered **~18 m over ~1
  min**; once it strays into `testcourse.glb` it pinballs (mesh restitution 0.3).

## F. SITL task / shim architecture (as found)
- `host_lifecycle.c` starts these firmware tasks: `angle_controller_task`,
  `angle_rate_controller_task`, `motor_task`, `imu_telemetry_task`, `flush_task`,
  `comm_processor_task` — and (now) `attitude_task`. It previously **omitted**
  `attitude_task`.
- Host shims feeding the firmware: `host_rc_feeder` (RC over a pty),
  `host_imu_feeder` (IMU over `/tmp/vsim_imu`), `host_navhal` (PWM out, UART2
  pty, clocks/cycle-counter). The IMU feeder was the only one running firmware-
  owned compute (the mahony) — now removed.
- AttitudeEuler telemetry is popped from `attitude_queue_telemetry`
  (`telemetry_task.c:115`), so before the fix even the telemetry attitude was the
  host shim's, not the firmware estimator's.

## G. vsim timing
- `kImuHz = 1000`, `kPhysicsHz = 8000` (RK4 substeps = physics/imu), pose 60 Hz
  (`main.cpp:44`). `imu_hz` is **wall-clock paced** (`outer_dur = 1e6/kImuHz`),
  runtime-tunable via `VSIM_CTL_SET_RATES`.
- The SDK/host does **not** send `SET_RATES`, so vsim runs the 1000 Hz default —
  hence the firmware's `IMU_SAMPLE_FREQ_HZ=2000` belief is unmet (discrepancy #2).
- Even "paced," the FIFO can deliver bursts; wall-clock dt between host reads then
  collapses. The fix sidesteps this by stamping a fixed cadence (a real IMU's ODR
  is fixed, not jittery) — more faithful than wall-clock regardless.

## H. Candidate causes for the still-open ~12× under-read (next trace)
Ordered by suspicion, given §A/§D ruled out accel, dt-value, gyro-units, gyro-
value, and the EKF-in-isolation (self-test 14/14):
1. **`attitude_task` decimation feed** — one gyro sample applied over a
   multi-sample `dt_sum`, and/or `ATTITUDE_DECIM` built from 2000 Hz vs the
   1000 Hz feed. Instrument `step_dt` and the gyro magnitude actually passed to
   the filter, per call.
2. **Sample/queue path** — confirm `imu_queue_attitude` delivers the same gyr the
   control queue does (no extra LPF/calibration/decimation on the attitude path),
   and that `gyr` (not `gyr_raw`) is the integrated field.
3. **EKF predict integration** at the SITL step rate — the self-test passes at
   its own cadence; reproduce the *exact* SITL call pattern (rate, dt_sum, single
   latest-sample gyro) in a unit test and watch a pure-rotation ramp.
   Add the missing regression test: a constant-rate gyro ramp must integrate to
   the matching angle through the full `attitude_task` path (not just `ekf.c`).
4. **Initial seeding + post-arm reset of accumulating states** — the strongest
   candidate for a *constant* scale error, and the only one that explains the
   under-read appearing in BOTH filters AND surviving the accel-disabled test.
   The EKF estimates a gyro bias `E.bg` (`ekf.c:46`); predict integrates the
   bias-corrected rate `w = to_radians(g) - bg` (`ekf.c:200`), and every
   accel/mag correction injects into `bg` via `ekf_inject` (`ekf.c:107-114`).
   On a multirotor the thrust-aligned accel reads ~level, so the filter can
   explain a real sustained tilt-rate as gyro BIAS and wind `bg` up until
   `w ≈ 0` → under-integration. The Mahony path has the identical failure mode
   through its integral term (`SF_MAHONY_KI 0.0025`) — which is why both filters
   under-read by the same factor. Two things to verify:
   (a) **Seeding** — `ekf_init` zeros `bg/ba` and sets `P` (`ekf.c:58-80`), but
       it runs ONCE at task start (`attitude_task.c:84`), not on arm. The static
       `E`/Mahony integrator therefore carries any bias learned in a prior flight
       (or during the pre-arm leveling transient) straight into the next arm —
       so pure-gyro predict can under-integrate from t=0 even with accel off.
       Add an `ekf_reset()`/integrator-zero on the disarm→arm transition.
    (b) **Windup bound** — there is no anti-windup / rate clamp on `bg` (or the
       Mahony Ki integral); a thrust-aligned accel can drive it arbitrarily.
       Instrument `bg` (and Mahony `integralFB`) across a maneuver: if it ramps
       toward the true body rate, this is the root cause. Fix is cycle-free on
       the F4 — a reset on arm + a bound on the bias state, no extra loop work.
