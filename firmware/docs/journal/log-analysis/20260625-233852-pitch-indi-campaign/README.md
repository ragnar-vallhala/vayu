# Pitch-oscillation root-cause + INDI campaign — 2026-06-25/26

Combined archive of the 9 captures that follow `../20260625-231706-pitch-osc/`.
Two arcs: (1) proving the PID pitch oscillation is **not** a gain-tuning problem,
and (2) standing up an INDI inner loop as the fix and retuning it to a stable
seed. Each capture keeps its own subfolder (raw `.bin`, per-capture `setup.json`,
and where present its own README/plots).

## Combined analysis (start here)

A full segmented sensor→actuator analysis across all 9 bins. **Root cause: not
the controller or its gains, but throttle-headroom starvation** — at this
craft's hover throttle the mixer can't deliver the attitude differential the
loop demands, so one side saturates and the anti-sat scaler discards authority →
an actuator limit cycle whose frequency is owned by the controller (hence PID-
*and* INDI-invariant).

| doc | covers |
|---|---|
| [`session-analysis.md`](session-analysis.md) | **combined overview**, method, regime budget, pipeline diagram, the whole-campaign verdict & per-capture story |
| [`control-loop-analysis.md`](control-loop-analysis.md) | the closed limit cycle, controller-owned frequency, why PID **and** INDI both fail |
| [`motor-analysis.md`](motor-analysis.md) | the actuator bottleneck — idle floor, anti-sat scaler, **saturation flips with throttle**, authority vs demand |
| [`sensor-analysis.md`](sensor-analysis.md) | IMU timing (clean), gyro (clean), accel **vibration** (5–20× under power, secondary) |
| [`sim_parity/`](sim_parity/README.md) | **fresh headless SITL runs**: why the sim flies (plant 3–7× too weak, over-powered) + modeled parity parameters |
| [`reference-autopilots-comparison.md`](reference-autopilots-comparison.md) | **sensor→PWM timing trace of vayu vs PX4 vs ArduPilot** (time base, filtering, estimator, mixer/airmode, output, latency budgets); **§9** how PX4/ArduPilot solve each problem (with file:line); **§10** resource (CPU/RAM) analysis + FFT feasibility & a self-contained FFT design |
| [`recommendations.md`](recommendations.md) | **ranked P0–P3 action plan** drawn from the comparison (power/weight, idle floor, airmode mixer, measured-dt, sim-cal, FFT notch, output) — *the report's conclusion* |

Printable PDF: **`20260625-233852-pitch-indi-campaign-analysis.pdf`**. Regenerate
the figures + report:
```sh
python3 make_plots.py             # -> plots/*.png (campaign figures)
python3 make_comparison_plots.py  # -> plots/cmp_*.png (the 4 comparison visuals; needs matplotlib + graphviz `dot`)
python3 analyze_campaign.py       # the segmented per-window numbers
python3 ../build_pdf.py .         # combine docs -> PDF (needs pandoc, xelatex)
```

Common rig across all captures: real flight controller over the ESP/UDP NavLink-v2
bridge (`10.42.0.30:14555`). X-quad, inverted-spin motor geometry (flips
`s_mix_yaw` only). **A motor was burned earlier in the 2026-06-25 session; the
pitch axis is suspected physically asymmetric.** No tunes persisted — every run
is on compiled defaults. Decode any capture with
`python3 tools/telemetry/analyze_pitch_osc.py <subfolder>/*.bin`.

## Captures (chronological)

| # | folder | controller | conditions | headline |
|---|---|---|---|---|
| 1 | `20260625-233852-pitch-verify` | PID (wc-capped v3) | bench | Pitch tune v3 **NOT fixed** — gain-invariant limit cycle → saturation, not gain |
| 2 | `20260626-000838-throttle-up` | PID defaults | bench | Raw throttle-up stream |
| 3 | `20260626-001343-telem30s` | PID defaults | bench | **Canonical PID baseline**: 1.77 Hz cycle, pitch out railed 71 %, pitch RMS 144 vs roll 6 |
| 4 | `20260626-010112-indi-test` | INDI k=20 | bench | First on-hw INDI stream |
| 5 | `20260626-010249-indi-test` | INDI k=20 | bench | INDI ~3× calmer than PID (RMS 49 vs 144); broadband 3.7 Hz hunt; roll b-fit **contaminated** by the hunt |
| 6 | `20260626-011558-indi-freeair` | INDI k=20 | **open air** | Both axes hunt ~1–3 Hz, outputs NOT railing (5 %) → **control-law hunt, not saturation**; drove the k=20→6 retune |
| 7 | `20260626-013249-indi-k20-openair` | INDI k=20 | open air (~10 min) | Hunt freq 3.3 Hz ≈ k (3.18 Hz) → no phase margin at crossover; bandwidth too hot |
| 8 | `20260626-014848-pid-openair` | PID defaults | open air | Same-conditions comparator: **sharp 1.95 Hz limit cycle** (rel power 1.0), pitch RMS 85→155; worse than INDI at matched throttle |
| 9 | `20260626-020449-indi-k6-seed` | INDI k=6, lpf=0.010 | open air | **SEED WORKS**: 3.3 Hz hunt collapsed (peak 0.11, flat), pitch RMS −40 % vs k=20, u sat 4 %; soft but no sustained cycle |

## Arc 1 — PID: the pitch limit cycle is structural, not a tuning bug

Across three gain sets spanning 2.4× in both `rate_kp` and `angle_kp`, pitch
limit-cycles at 1.5–2.6 Hz while roll stays rock-stable. Softening the loop
*lowered* the frequency but *raised* saturation (37 %) — a roughly gain-invariant
cycle that rails the actuator is **saturation / authority-limited**, not a linear
gain issue (`pitch-verify`, `telem30s`). Physical tell: the front motor pair runs
~20 % harder than the back (pitch-axis imbalance), and sysid pitch `K = 1381` is
2.45× roll's `563` on a symmetric quad — the pitch axis was already asymmetric
when it was identified.

## Arc 2 — INDI inner loop as the fix

INDI is the better control law (~2× tighter than PID at matched throttle,
`indi-test`/`indi-freeair`), but the k=20 seed hunts at its own bandwidth
(3.3 Hz ≈ k, `indi-k20-openair`) — no phase margin at crossover, and the b values
are unfit placeholders. Open-air confirmed the hunt is a control-law issue (outputs
not railing), so the seed was retuned **k 20→6, lpf 0.005→0.010, b kept high**.
The `indi-k6-seed` run collapsed the hunt to a flat spectrum: soft/sluggish (pitch
drooped to −82°) but stable and barely saturating — calm enough to finally run a
**clean sysid b-fit**.

## Bottom line / next steps

1. PID pitch oscillation is a **saturation/authority limit cycle, gain-invariant** —
   not fixable by retuning. Real hardware asymmetry underneath (front/back thrust
   imbalance, suspect burned motor, pitch K 2.45× roll).
2. **INDI is the better law**; k=6/lpf=0.010 is a working stable seed.
3. Next: clean INDI b-fit on the calm k=6 seed, then push k back up for performance —
   and still address the pitch-axis hardware asymmetry.

See `firmware/include/control/rate_indi.h`, `firmware/include/variables.h`, and the
preceding `../20260626-001343-telem30s/` reference baseline (now under this folder).
