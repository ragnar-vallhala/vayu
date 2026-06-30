# Recommendations

Ordered by impact. The one defect that matters in this run is the roll/pitch
rate-loop tuning; everything else is either healthy or a follow-up capture.

## 1. Fix the roll/pitch rate-loop gains (the headline)

**Problem:** roll/pitch rate `Kp = 0.0005` (36× below yaw's 0.018) with
`Kd = 0`. The proportional path contributes ~2.5% of authority at a 50°/s error,
so the loop leans entirely on a clamped `Ki = 0.01` integral and cannot track.
Evidence: corr(rate_sp, rate_curr) = 0.12/0.14 vs yaw 0.95; output never exceeds
0.13 of ±1; pitch knocked to 17.8° with a −0.025 response.

**Where:** `include/variables.h` lines 141–153
(`DEAFULT_ROLL_ANGLE_RATE_*`, `DEAFULT_PITCH_ANGLE_RATE_*`), and any persisted
PID config that overrides them at boot (`pid_config_get_rate` in
`angle_rate_controller.c`).

**Action — preferred: run autotune.** The repo has an autotune stack
(`tools/autotune/`, `navigator/src/autotune/`) that searches rate-loop gains in
SITL. Roll/pitch are the axes to target; yaw is already a reasonable reference
for the achievable authority. Autotune avoids hand-guessing the right Kp/Kd for
the simulated airframe's inertia and motor-torque arms.

**Action — quick manual sanity bump** (to confirm the diagnosis before a full
autotune, NOT a final tune): raise roll/pitch `Kp` toward the same order as yaw
and add real derivative damping, e.g. `Kp ≈ 0.006–0.012`, `Kd ≈ 0.0003–0.0008`
(the PID already has a derivative LPF, `d_lpf_rc = 0.3`, and `d_max = 0.25`, so
Kd is safe to introduce). Re-run this same SITL scenario and confirm
corr(rate_sp, rate_curr) climbs toward yaw's 0.95 and `pitch_out` actually moves
during the t≈77 s style upset. **Do not ship hand-picked numbers** — use them
only to validate, then let autotune set finals.

**Guardrail:** whatever the source of final gains, watch for the opposite
failure (limit cycle / motor buzz) by checking `roll_out`/`pitch_out` don't
start saturating or oscillating at the loop rate.

## 2. Re-run this scenario as a regression after the tune

This capture is a clean, deterministic reproduction of the failure, which makes
it an ideal **before/after** fixture. Keep `export-20260620-053528.bin` as the
"before". After the gain fix, replay the same RC/throttle script and re-run
`make_plots.py`; the pass criterion is:

- roll/pitch corr(rate_sp, rate_curr) ≥ ~0.8 (up from 0.12),
- roll/pitch `_out` actively used (peaks well above 0.13) without saturating,
- the t≈77 s class of disturbance arrested in << the current sloppy recovery.

Consider promoting this into the headless-SDK integration suite
(`navigator/headless-sdk/tests/integration/`) as a tracking-quality assertion so
the gain regression can't silently come back.

## 3. Follow-up captures (not failures, coverage gaps)

- **Gentle hover / slow vertical** — this run's ±50 m/s profile never exercised
  the near-hover regime where the vertical estimator's baro noise and accel-bias
  rejection actually matter, nor where the IN_AIR flight-phase thresholds get
  stressed. Capture a slow take-off → hover → gentle descent → land.
- **Roll/pitch step inputs** — once retuned, command actual roll/pitch angle
  steps (this run left them centred) to measure step response, overshoot, and
  settling, not just level-hold rejection.

## 4. Tooling note (already fixed in this pass)

`docs/journal/log-analysis/parse_log.py` had a stale repo-root path (computed
two levels up instead of three after the docs were moved under `journal/`), so
it raised `ModuleNotFoundError: navlink_msgs`. Fixed in this change. The generic
`make_plots.py` in that directory is still **bespoke to the 2026-06-17 hardware
session** (mag spheres, kernel PerfFifo, tilt narrative) and does **not** apply
to SITL logs — this archive ships its own `make_plots.py` instead. Worth
factoring the generic figures out of the session-specific narrative later.

## Not action items (verified healthy)

- Cascade architecture, yaw loop, loop timing (4/1 ms, zero jitter), mixer/
  anti-windup — all fine; see control-loop-analysis.md §"What is NOT wrong".
- Vertical estimator + flight-phase detector — healthy across the stress
  profile; see vertical-analysis.md.
- Decode path — 0 CRC errors, no unknown msgids.
