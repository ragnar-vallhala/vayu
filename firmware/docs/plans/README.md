# FC plans

Active plans for in-flight firmware work; a plan is deleted once its feature ships.

- [`vertical-velocity-vibration.md`](vertical-velocity-vibration.md) — with the
  motors running the accelerometer under-reads gravity by ~1 m/s^2, so the vertical
  filter integrated a phantom -1 m/s descent and the height controller answered with
  roughly double hover thrust. FIXED by adding the estimator's third state (accel
  bias) plus a health flag that vetoes the height mode; host-tested, NOT yet run on
  hardware. The CAUSE of the under-read is still not established — the bench
  measurement is the next step.
- [`rate-loop-saturation.md`](rate-loop-saturation.md) — the rate PID can demand
  ~2× the differential thrust the airframe can deliver at hover, so the mixer
  saturates, airmode shifts collective to preserve roll/pitch, and the aircraft
  climbs away. Records the measured mechanism, what PX4 and ArduPilot do about it
  (saturation fed back into the integrator; a bounded collective shift), and the
  fix order. **Blocks the height mode.** Design stage.
- [`altitude-hold-and-in-air-plan.md`](altitude-hold-and-in-air-plan.md) — barometer-driven
  vertical estimation, an `ALT_HOLD` flight mode, and finally driving the dormant
  `SYSTEM_STATE_IN_AIR` transition. Phased, SITL-first. Phases 1–2 (estimator +
  baro-driven IN_AIR) shipped. Phases 3–5 were then implemented by a **different**
  design — a ch6 takeoff/hold/land mode on a VL53L0X rangefinder, plus two
  pre-existing safety bugs fixed — and are now **blocked** after the first flight
  hit the ceiling; see the status box above Phase 3 and `rate-loop-saturation.md`.
- [`imu-i2c-scheduler-redesign.md`](imu-i2c-scheduler-redesign.md) — move all I2C
  access + the acquisition scheduler out of `bmx160.c` into `i2c_manager`,
  multi-rate phase-offset scheduling, and recovery hardening for the ~320 s bus
  wedge. Design stage; refactor not yet landed.
- [`sitl-lockstep-sim.md`](sitl-lockstep-sim.md) — faster-than-realtime, deterministic
  SITL via a single virtual clock + bounded vsim backpressure. Phases 1–2 shipped
  (~3× at credit=2) and Phase 4 (real vaios scheduler on host, ~57×) achieved on
  `vayu_sitl_rtos`; remaining: sim-time harness pacing (Phase 3) + wire-up to make
  the RTOS variant the default SITL. Host/harness only; flashed firmware untouched.
- [`procedural-world-generation.md`](procedural-world-generation.md) — pluggable
  biome architecture for the SITL renderer. Phase 0 (meadow terrain) shipped;
  further biomes/phases remain.
- [`monorepo-restructure.md`](monorepo-restructure.md) — promote the three buried
  components to top-level siblings: `software/`→`navigator/`, pull all simulators
  (`sim_host`/`vsim`/`gazebo`/`renode`) out of `tools/` into `sim/`, and lift the
  naked root firmware into `firmware/`. Measured blast radius + phased `git mv`
  plan (sim → navigator → firmware), each phase green before the next. Design
  stage; not yet executed.
- [`fft-dynamic-gyro-notch.md`](fft-dynamic-gyro-notch.md) — adaptive band-stop on
  the gyro that FFT-tracks the analog-ESC prop vibration (no RPM telemetry). Full
  chain built bottom-up (FFT engine → biquad → analysis front-end → notch bank →
  rate-loop glue → NavLink tuning surface), plus SD persistence (`pid.bin` PID4),
  a GCS tuning page, spectrum-derived auto-band, and the >1 kHz decimation seam.
  Host/SITL-tested and F401-cross-compiled, but **OFF by default and not
  flight-tested** — the flight test is the one remaining substantive gap.
- [`stale-docs-comment-audit.md`](stale-docs-comment-audit.md) — codebase-wide
  sweep (excl. `extern/`) fixing docs/comments that drifted from the code: the
  calibration overhaul, telemetry 166→500 Hz, channel 512→2048 B, stale
  control-loop/EKF/sysid comments, dead declarations, `calib_step` EDGE codes,
  and a few GCS fixes. Doc/comment edits + one dialect-enum regen; no behavioral
  change.

Add a plan here when starting a new piece of firmware work, and remove it on completion.
