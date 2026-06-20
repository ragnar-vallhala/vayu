# Why autotune produces a soft roll/pitch rate loop

**Status:** analysis only — no code changed yet. Captured 2026-06-20 to revisit.
**Context:** the 2026-06-17 (hardware bench rig) and 2026-06-20 (SITL) log analyses
both flagged the roll/pitch inner **rate** loop as non-tracking (corr ≈ 0.12 vs
yaw 0.95), with a 36× gain gap to yaw. This documents (a) what gains were
*actually* used in the 06-20 run, and (b) why the autotuner — both the C++ app
(`software/src/autotune/`) and the Python tool (`tools/autotune/`) — produces a
weak roll/pitch `rate_kp` on purpose.

---

## Part A — what the 06-20 SITL run actually used (vs. the reports' assumption)

The log-analysis reports assumed the `include/variables.h` **defaults**. The SITL
run instead loaded an autotuned `pid.bin` (firmware loads `0:pid.bin` at boot via
`pid_config_init()`, `tools/sim_host/src/host_lifecycle.c:168`).

The live file `/tmp/vayu_vfs/0_pid.bin` is gone (in `/tmp`, since wiped), but two
sources pin the gains: `tuned_pid.json` (repo root; pulled from that store on
2026-06-06) and the log's own empirical `Kp_eff` fit (which confirms Kp/Kd).

| Param (rate loop)  | Report assumed (default) | Actual (pid.bin) | Match? |
|--------------------|--------------------------|------------------|--------|
| roll Kp            | 0.0005                   | **0.0005**       | yes |
| pitch Kp           | 0.0005                   | **0.0005**       | yes |
| yaw Kp             | 0.018                    | **0.018**        | yes |
| roll/pitch Kd      | 0                        | **0**            | yes |
| roll/pitch **Ki**  | **0.01**                 | **0.00333**      | NO (3x lower) |
| yaw Ki             | 0.008                    | **0.0**          | NO |
| roll/pitch i_max   | 0.2                      | 0.2 (not in bin) | yes |
| angle Kp r/p / yaw | 4.0 / 1.2                | 4.0 / 1.2        | yes |

**Verdict:** the report's central claim holds — the autotune **left roll/pitch
rate Kp at 0.0005 with Kd=0** (identical to default for the P/D terms). It only
*lowered* Ki (0.01->0.00333) and zeroed the angle/yaw integrators. The one wrong
assumption (Ki) errs conservatively: the real integral authority is *less* than
the report assumed, so "the loop leans on the integrator and can't track" is, if
anything, understated. Empirical `Kp_eff` (roll 4.6e-4, pitch 3.8e-4, yaw
pulled-down 1.7e-3) confirms the 0.0005 / 0.018 split was genuinely active.

Gaps: the actual `pid.bin` is **not archived** with the 06-20 capture (only
`export.bin` is), and Ki=0.00333 is **not independently confirmed** from the
06-20 log (only Kp is). Archive `pid.bin` with every capture going forward.

---

## Part B — why the autotuner produces a weak roll/pitch `rate_kp`

The C++ app is a faithful port of the Python tool (same `Space`, `Cost`,
`structured` sweep, rollout). The 06-06 tune came from the Python tool. The
finding applies to both. **`structured` did not fail — it faithfully minimized a
cost that, by construction, does not value a stiff roll/pitch rate loop.** Four
mechanisms stack, all pushing `rate_kp` to its floor.

### 1. The cost scores roll/pitch on ANGLE tracking, yaw on RATE tracking (root asymmetry)
- Roll/pitch -> `_axis_cost` (`autotune.py:121`): IAE of `{axis}_angle_sp` vs
  `{axis}_angle_curr` — the **outer** loop's job, minimized by raising
  `angle_kp`, not `rate_kp`.
- Yaw -> `_yaw_rate_cost` (`autotune.py:150`): IAE of `yaw_rate_sp` vs
  `yaw_rate_curr` — minimized only by raising `yaw_rate_kp`.
- Same split in C++ (`Rollout.cpp:178`: `(ch==3) ? yawRateCost : axisCost`).

So the objective **structurally demands** a strong yaw rate gain and
**structurally tolerates** a near-zero roll/pitch rate gain. The autotune and the
log analysis measure the same thing and agree; autotune just never treated a
passive roll/pitch rate loop as a defect.

### 2. The chatter penalty makes `rate_kp` strictly costly with no reward
`chatter_pen = 25.0 * max(0, chatter - 0.04)` (`autotune.py:146`). Tracking is
already handled by `angle_kp`, so raising `rate_kp` adds chatter cost and reduces
nothing -> positive gradient on `rate_kp` -> minimizer drives it to the floor.

### 3. The bounds cap `rate_kp` low by design and route responsiveness to `angle_kp`
`_BASE` (`autotune.py:58`): `rate_kp` in [0.0005, 0.012] "capped below the buzz
knee (~0.01)", `angle_kp` in [0.05, 3.0] "responsiveness lever". Comment:
*"RESPONSIVENESS comes from the OUTER angle_kp."* Result `rate_kp=0.0005` is the
**exact lower bound**.

### 4. The throttle-buzz penalty is a second, stronger downward force
`throttle_buzz` (`autotune.py:337`): centered sticks, throttle sweep 1500->1750,
penalizes roll/pitch rate-output chatter at `BUZZ_SCALE=150 * (chatter - 0.02)` —
6x the in-rollout chatter scale at a lower threshold. Any `rate_kp` high enough
to buzz at hover+ is hammered. Hence `tuned_pid.json`'s "buzz-free region (low
rate_kp, high angle_kp)."

### Why `structured` specifically cements it
`structured` (`optimizers.py:187`) tunes one gain at a time, greedily, never
revisiting: order `rate_kp -> rate_kd -> rate_ki -> angle_kp`. It starts each
inner gain at its **lower bound**, then sweeps `rate_kp` *first while `angle_kp`
is held at the seed*. Angle tracking is already decent, so the 1-D minimum is the
floor -> it locks 0.0005 and moves on, then cranks `angle_kp` last. A joint
optimizer (SPSA/Nelder-Mead) might explore a "high rate_kp + rate_kd damping +
high angle_kp" basin; the greedy sequential sweep forecloses it. `structured` is
the worst optimizer for escaping this trap.

### What this means
`freeflight_check` (`autotune.py:276`) only does a gentle 1700us step with a low
bar (<60 deg tilt, settle <20 deg), which a high-`angle_kp` tune passes.
**Neither the scored rollout nor the validation exercises fast torque-disturbance
rejection** — exactly the t~=77 s upset the log analysis caught
(`pitch_out=-0.025` vs a -53 deg/s demand). The cost's blind spot *is* the
firmware's flight weakness.

Design conflict to resolve: bounds were capped low **because high
`rate_kp`/`rate_kd` buzzed**; the log analysis recommends the opposite (raise
`rate_kp`, add `rate_kd`). Honest reconciliation: the **buzz is the real problem
to fix** (gyro noise/deadband, D-term filtering, or the sim's motor/actuator
model) so a stiff, disturbance-rejecting rate loop becomes usable — rather than
neutering the inner loop to dodge it.

### Decoupling roll/pitch (prerequisite for the next step)
Currently roll and pitch **cannot** be tuned independently: `apply_gains`
(`autotune.py:103`, C++ `Rollout.cpp:64`) writes the same `rate_kp/ki/kd` +
`angle_kp` to both axes, and the rollout **sums** their per-axis costs into one
scalar. They are a single shared knob. Decoupling needs the param vector split
(`rate_kp_roll`, `rate_kp_pitch`, ...; 4->7 base params) or two single-axis
tunes, each driven by its own axis cost.

---

## Candidate next steps (not started)
1. Add a **rate-tracking term to the roll/pitch cost** (mirror `_yaw_rate_cost`)
   so the search values inner-loop tracking. Highest leverage.
2. **Split the param space** to decouple roll/pitch (per-axis gains + cost).
3. Add a **disturbance-rejection rollout** (torque kick at sp=0, score recovery)
   so the t~=77 s failure mode is in the objective.
4. **Investigate the buzz** at higher `rate_kp` — real plant limit or sim/gyro
   artifact? Decides whether raising the `rate_kp` ceiling is safe.
