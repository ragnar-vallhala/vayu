# Recommendations — on-hardware tune (combined)

Prioritised actions from the four analysis docs. The theme: **the rig made
roll/pitch look tuned because the gimbal absorbed a real left-thrust bias and the
pitch cascade was "only" 1.4 Hz; free flight removed the constraint and both took
over.** Stop flying; close these on the bench/rig first.

## P0 — before any further flight

1. **Props-off motor-command sign check** (deferred test). Arm props-off ~0.2
   throttle, tilt right-side-down by hand, read `MotorTelemetry`: the **right**
   motors should spin up. The recorded mixer correlation says roll sign is correct
   (+0.37…+0.75) — this bench-verifies it directly and closes the roll question the
   rig couldn't. [motor §mixer-sign]
2. **Find the left-thrust bias** (L−R = +24–26 % on the rig, M3 saturating 148
   frames). Per-motor thrust/current check (stand or props-off current draw), check
   **M1/M2 (right) for a weak motor/ESC**, re-check **CG** and **IMU roll mount**.
   This bias is the recorded cause of the free-flight left dive. [motor §left-bias]
3. **Fix the pitch cascade limit cycle** (~1.4 Hz, peaks 322–378 °/s). Either
   **finish pitch (and yaw) system-ID** so the inner loop is matched and damped like
   roll, **or** slow `angle_kp` below the inner-loop bandwidth. Reducing rate kp made
   it worse — do **not** chase it with inner gains. [control-loop §pitch]
4. **Persist the working tune.** Gains + the `spin=[-1,1,-1,1]` yaw fix are
   **live-only** today; a battery cycle restores the broken yaw sign and the bad
   defaults. Wire a persist/save (`PID_CONFIG_FILE_PATH`) or auto-apply on boot.
   [session, motor §recs]

## P1 — sensors / fusion

5. **One more accel 6-side cal** (+2.5 % → ≤1 %) to fully solidify the EKF gravity
   gate. [sensor §accel]
6. **Finish mag cal powered** — residual ~+25 µT z hard-iron in every log, incl.
   post-cal; required before any heading/nav use. [sensor §mag]
7. **Capture a 30 s still window** to re-verify gyro bias (not measurable in these
   logs — motion-contaminated). [sensor §gyro]

## P2 — kernel / firmware hygiene

8. **Trace the ff_001043 IPC contention** — 7,099 `ipc_timeouts` + IMU_CALIB FIFO
   saturating when a calibration runs under load (0 in steady state). Doesn't starve
   control today, but resolve before relying on in-flight calibration. [kernel §IPC]
9. **Watch `task_id 4` stack** (80 %, up from 74 %) — least headroom, growing. [kernel]
10. **Fix the carried-over telemetry bugs** (all still open from 06-17):
    `AttitudeEuler` body rates = 0, `ImuRaw.sample_time_us` = 0,
    `SystemHealth.cpu_load` = 0, `ImuCompressed.ref_seq` = 0; widen FIFO `drops`
    past u16. Body rates (#1) most impactful for replay. [sensor §reporting, kernel]

## P3 — capture protocol for next session

11. **Always GCS-record while monitoring.** The actual powered free-flight crashes
    were harness-stdout only and had to be reconstructed (`data/flight_traces.txt`).
    Keep the Navigator recorder running (or add a `--csv` flush to the harness) so
    every arm is a real `.bin`.
12. **Rig-only until AGL-real IN_AIR is boring.** Both rig "IN_AIR" latches had
    AGL ≈ 0 (gimbal-held). Define success as a *real* altitude hold on the rig with
    no pitch cycle and balanced motors, **then** fly. (Same advice as 06-17 — which
    we skipped, and the left dive is the price.)

## Status of the 06-17 recommendations (closing the loop)

| 06-17 said | this session | status |
|---|---|---|
| Re-tune rate loop (add Kd, raise Kp, lower Ki) | sysid roll → kp 0.012, kd 0.00025, safe ki | **done for roll**; pitch/yaw not yet |
| Calibrate accel (8 % scale) | +7.8 % → +2.5 % | **mostly done** (one more pass) |
| Calibrate mag (heading) | spread ~140 % → 64 %, z offset remains | **partial** |
| Fix 3 zero telemetry fields | — | **not done** |
| Investigate TX saturation (2 k overflow) | tx_overflow now 0 | **resolved** |
| Rig-only until boring | went to free flight | **not followed** → left dive |
