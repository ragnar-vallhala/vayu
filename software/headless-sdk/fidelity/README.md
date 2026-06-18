# Control-loop fidelity suite

Flies the REAL firmware in SITL through a set of maneuvers, records ground truth
vs command vs estimate, and rates how faithfully the control loop tracks. Every
run first **verifies the airframe**: it asserts vsim_d applied the loaded frame's
motor geometry (from the GCS conf or a `--vveh`), not vsim's compiled-in X3
defaults — so a fidelity number is never quietly measured against the wrong
plant.

## Run it

```sh
cd software/headless-sdk
PYTHONPATH=".:examples:fidelity" python3 fidelity/run_all.py          # whole suite + scorecard
PYTHONPATH=".:examples:fidelity" python3 fidelity/run_all.py --vveh ../../v2.vveh   # pin a frame
PYTHONPATH=".:examples:fidelity" python3 fidelity/attitude_steps.py --axis roll     # one maneuver
PYTHONPATH=".:examples:fidelity" python3 fidelity/rate_fidelity.py   # re-print scorecard from out/
```

Results (CSV + a metrics JSON sidecar per maneuver) land in `fidelity/out/`.

## Frame source (the "am I flying the right frame?" guarantee)

- Default: the **GCS conf** (`~/.config/Vayu/Vayu GCS.conf`) — whatever vehicle the
  operator selected in Navigator. Currently the **S500** (mass 1 kg, motor-diag
  ~475 mm, motor positions `±0.168`).
- Override: `--vveh <file>` pins geometry straight from a `.vveh` (the
  authoritative frame definition), bypassing the conf.
- Proof: vsim_d echoes the per-rotor positions it applied; `verify_frame()` reads
  that echo and hard-fails if it ≠ what was pushed, or if nothing was pushed
  (which would mean the X3 defaults). Look for `[frame] VERIFIED …` per run.

## Maneuvers

| Script | Rig? | Measures |
|---|---|---|
| `attitude_steps.py --axis roll/pitch` | rig | inner attitude loop: rise / overshoot / settle / SS error |
| `yaw_rotate.py` | rig | yaw-rate loop response |
| `hover.py` | free | station-keep drift, altitude RMS, attitude steadiness, est-vs-truth gap |
| `throttle_step.py` | free | altitude (vertical) step response |
| `tracking.py` | free | position tracking error over a box path |

Rig maneuvers pin translation so the attitude loops are measured cleanly; free
maneuvers exercise the full outer loop + estimator together.

## Scoring

Each maneuver gets a 0–100 fidelity score (`_fidelity.fidelity_score`); the
overall is a weighted mean (`rate_fidelity.WEIGHTS`). Scores are heuristic — meant
to flag regressions and the known defects, not to be a precise grade. The
estimator under-read (deferred issue #1) is surfaced separately as the est-vs-true
ratio; see `docs/deferred/01-attitude-estimate-underread.md`.
