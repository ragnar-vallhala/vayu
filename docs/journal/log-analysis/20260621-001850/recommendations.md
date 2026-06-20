# Recommendations

Ordered by impact. The one defect that matters across all three runs is the
roll/pitch rate-loop gain; the triplet also hands us a ready-made before/after
fixture for the autotune work.

## 1. Land a stiffer roll/pitch rate gain — and make 001716's the floor

**Problem:** the roll/pitch rate loop is still under-gained. The two flights at
`Kp_eff ≈ 5–6e-5` (001607, 001850) tumble inverted; the one at `≈ 2.1e-4`
(001716, ~4×) tracks (corr 0.70/0.84) and never crosses 90°. Even that good run
sits **~45× below yaw** and uses **<12 %** of its ±1 authority — so it is not a
ceiling, it is a floor with headroom to spare.

**Evidence:** see [control-loop-analysis.md](control-loop-analysis.md) Findings
1–4 — the gain/outcome table, the tracking grid, the unused authority, the two
tumble timelines.

**Where:** `include/variables.h` (`DEAFULT_ROLL_ANGLE_RATE_*`,
`DEAFULT_PITCH_ANGLE_RATE_*`) and any persisted PID config that overrides them at
boot (`pid_config_get_rate` in `angle_rate_controller.c`).

**Action — preferred: let autotune / System-ID set it.** This capture session
*is* the autotune work. The new fast backend + analytic System-ID path
(`software/src/autotune/`, `RtosEval` + `SysId`) are built to design exactly this
gain on the loaded airframe without hand-guessing. 001716 is empirical proof the
direction is right; push the tuner to go further than 4× while watching the
guardrail below, since the authority is clearly available.

**Action — guardrail (both directions):**
- *Too soft* → the failure in this archive: corr stays low, `_out` hugs zero, the
  craft tumbles under aggressive stick. Pass criterion: roll/pitch
  corr(rate_sp, rate_curr) ≥ ~0.8 (001716 already shows 0.70/0.84) **and** no
  |roll|>90° excursion under the same flight.
- *Too stiff* → the opposite failure (limit cycle / motor buzz). Watch that
  `roll_out`/`pitch_out` don't start oscillating at the loop rate or pinning the
  ±1 rails. There is margin here (peak duty ≤ 0.78, `_out` ≤ 0.12), so a large
  increase is safe to try.

## 2. Promote the triplet to a before/after autotune fixture

These three logs are a deterministic, three-point sweep of the deciding
variable, which makes them an ideal regression fixture:

- Keep all three `export-20260621-*.bin` as the "before / borderline / after"
  set (001607 fail, 001850 borderline, 001716 good).
- After the tuner lands a final gain, replay the **001607** RC/throttle profile
  (the clearest failure) and re-run `make_plots.py`; the run must now look like
  001716 — corr ≥ 0.8, no inversion, `_out` actively used without saturating.
- Consider a tracking-quality assertion in the headless-SDK integration suite
  (`software/headless-sdk/tests/integration/`) keyed on
  corr(rate_sp, rate_curr) and a "never inverts under this script" check, so the
  gain regression that produced 001607/001850 can't silently return.

## 3. Capture cleanups for the next session

- **Log the loaded gains.** The single biggest gap: the PID constants are not in
  `ControlTrace`, so we can only recover *effective* gain from the data. A
  one-shot "active PID config" telemetry message (or a line in the session
  header) at arm time would turn "Kp_eff ≈ 2.1e-4" into "Kp = X, Kd = Y", making
  the before/after unambiguous.
- **Command yaw in every run.** Yaw was untouched in 001607, so its healthy
  reference (corr 0.99) is missing from the very run that failed worst. Always
  exercise all three axes so the working-axis baseline is present.
- **A controlled roll/pitch step set.** These were free-form manual flights;
  once retuned, fly defined roll/pitch angle steps to measure overshoot and
  settling, not just aggressive free flight.

## Not action items (verified healthy)

- **Cascade architecture, yaw loop, loop timing** (`outer_dt` 4.000 ms /
  `inner_dt` 1.000 ms, **0 µs jitter** in all three), **mixer / anti-windup**
  (no motor saturation, balanced duty) — all fine; see
  control-loop-analysis.md §"What is NOT wrong".
- **Vertical estimator** — healthy in all three (`valid = 100 %`, fused−baro RMS
  ≤ 1.08 m). The 001850 4.2 km / +202 m/s excursion is a *consequence* of the
  roll tumble, not an estimator or vertical-control fault; see
  [vertical-analysis.md](vertical-analysis.md). It disappears when the rate loop
  is fixed.
- **Decode path / link** — 0 CRC errors across ~21 k frames, no unknown msgids.
