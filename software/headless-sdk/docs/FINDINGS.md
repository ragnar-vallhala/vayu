# Findings — live maneuver demo (2026-06-18)

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

## Gap 3 — outer-loop position guidance (the blocker)

Quantified this run:
- **Short station-keep is fine** — held origin to sub-meter (−0.3…−0.1 m) over a
  6 s window.
- **Long holds wander** — drifted ~18 m off the hold point over ~1 min of idle
  (slow divergence / integrator behaviour).
- **Moving-setpoint tracking fails** — chasing a lemniscate, position error grew
  to ~16 m mean; the craft never converged. Slowing the path (5 m / 30 s lap)
  did not help, so it is not a setpoint-rate problem.

Root cause: the cascade in `autopilot.py::guidance_outputs` is a damped
P-on-position → velocity → tilt with hard caps but **no acceleration limiting,
no feedforward along the path, and no anti-windup** on the (small) integrator.
It is adequate to *approach* a static waypoint (the integration golden only
asserts "advances through the course") but not to *track* a trajectory or hold
tightly for long.

- Recommended fix: accel-limited / feedforward outer loop (command the velocity
  *and* its derivative along the path), anti-windup, and a tighter,
  rate-bounded position term. This unblocks every free-flight demo.

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

## Bottom line / next actions

1. Land Bug 1 + Bug 2 — small, correct, improve every session.
2. Outer-loop guidance tuning (Gap 3) — the substantive work; gate it on a
   collision-free tracking test (Gap 4) so the two effects are separable.
3. The rig system-ID examples (`step`/`chirp`/`doublet`) are trustworthy today —
   use them for cascade ID while the outer loop is being tuned.
