# FC plans

Active plans for in-flight firmware work; a plan is deleted once its feature ships.

- [`altitude-hold-and-in-air-plan.md`](altitude-hold-and-in-air-plan.md) — barometer-driven
  vertical estimation, an `ALT_HOLD` flight mode, and finally driving the dormant
  `SYSTEM_STATE_IN_AIR` transition. Phased, SITL-first. Phases 1–2 (estimator +
  baro-driven IN_AIR) shipped; Phases 3–5 (mode plumbing, ALT_HOLD controller,
  hardware bring-up) remain.
- [`imu-i2c-scheduler-redesign.md`](imu-i2c-scheduler-redesign.md) — move all I2C
  access + the acquisition scheduler out of `bmx160.c` into `i2c_manager`,
  multi-rate phase-offset scheduling, and recovery hardening for the ~320 s bus
  wedge. Design stage; refactor not yet landed.
- [`sitl-lockstep-sim.md`](sitl-lockstep-sim.md) — faster-than-realtime, deterministic
  SITL via a single virtual clock + bounded vsim backpressure. Phases 1–2 shipped
  (~3× at credit=2) and Phase 4 (real vaios scheduler on host, ~57×) achieved on
  `vayu_sitl_rtos`; remaining: sim-time harness pacing (Phase 3) + wire-up to make
  the RTOS variant the default SITL. Host/harness only; flashed firmware untouched.
- [`robust-imu-accel-gyro-calibration.md`](robust-imu-accel-gyro-calibration.md) —
  replace the fragile exact-pose accel calibration with a true full-3×3 ellipsoid
  fit over ~12 pose-tolerant holds (6 faces + edges/corners; NavLink unchanged, GCS
  wizard updated), make the gyro capture stillness-gated as a separate routine, and
  lift the whole calibration into a shared sensor-agnostic `src/calib/` module (mag
  migrated onto it). Mag behaviour deferred. Design stage.
- [`procedural-world-generation.md`](procedural-world-generation.md) — pluggable
  biome architecture for the SITL renderer. Phase 0 (meadow terrain) shipped;
  further biomes/phases remain.

Add a plan here when starting a new piece of firmware work, and remove it on completion.
