# FC plans

Active plans for in-flight firmware work; a plan is deleted once its feature ships.

- [`altitude-hold-and-in-air-plan.md`](altitude-hold-and-in-air-plan.md) — barometer-driven
  vertical estimation, an `ALT_HOLD` flight mode, and finally driving the dormant
  `SYSTEM_STATE_IN_AIR` transition. Phased, SITL-first.
- [`sitl-lockstep-sim.md`](sitl-lockstep-sim.md) — faster-than-realtime, deterministic
  SITL via a single virtual clock + bounded vsim backpressure. Phased; host/harness
  only, flashed firmware untouched. Cuts autotune sweeps from minutes to seconds.

Add a plan here when starting a new piece of firmware work, and remove it on completion.
