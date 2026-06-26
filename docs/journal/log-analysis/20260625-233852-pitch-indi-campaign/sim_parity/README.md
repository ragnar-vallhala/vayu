# Sim↔real parity — fresh headless SITL runs

Rather than lean on old sim logs, this measures the **current** sim plant by
driving the headless harness (`vayu_headless`, real FC logic + `vsim_d` physics,
GCS-conf geometry) and comparing to the real on-hardware numbers from this
campaign. Goal: model the parameters that bring the sim closer to reality.

## How it was run

```sh
cd navigator/headless-sdk
./.venv/bin/python <campaign>/sim_parity/collect_parity.py roll pitch tone_roll tone_pitch hover
python3 <campaign>/sim_parity/fit_parity.py        # fit K / hover, compare to real
python3 <campaign>/sim_parity/parity_plot.py       # -> ../plots/09_sim_parity.png
```

- **`tone_{roll,pitch}.csv`** — a strong single 0.8 Hz tone on the rig
  (translation pinned, attitude free). We log the controller effort
  `u = ControlTrace.{roll,pitch}_out` and the TRUE body rate (`vsim` ground-truth
  `omega`). Because `omega/u` is the plant transfer regardless of how `u` is
  generated, a lock-in at the drive frequency gives a clean
  `K = 2πf·|omega|/|u|` (deg/s² per u) — the same quantity the real sysid fit.
- **`hover.csv`** — free flight: take off and hold, log throttle + the pipeline.
- **`parity.conf` + `parity_tone_*.csv`** — the verification: the GCS conf with
  roll/pitch inertia scaled to the parity target, re-run to see what the same
  firmware does on a realistic plant.

## What we measured (sim) vs real

| quantity | SIM (fresh) | REAL | gap |
|---|---:|---:|---|
| control effectiveness `K_roll` | **187** | 563 | sim **0.33×** |
| control effectiveness `K_pitch` | **199** | 1381 | sim **0.14×** |
| hover throttle | **0.252** (sd 0.008) | ~0.45 | sim over-powered ~1.8× |
| pitch_out saturation (hover) | **0 %** | 26–83 % | — |
| pitch-rate RMS (hover) | **6 dps** | 50–194 dps | — |
| loop rate / jitter | 1 kHz / 0 | 1 kHz / 0 | match |

**The sim flies because its plant is 3–7× too weak and over-powered.** The real
airframe produces 3× (roll) to 7× (pitch) more angular acceleration per unit of
control effort, and hovers with far less thrust margin.

## Verification — and why it matters

Scaling the sim's inertia to hit the real `K` (`I_xx ×0.33`, `I_yy ×0.14`; since
`K ∝ 1/I`) and re-running the same firmware: the rig **diverged to 3000–4000 °/s**
(`pitch_out` rail 74 %). Giving the sim realistic control effectiveness makes the
**identical firmware go unstable** — i.e. the controller is effectively tuned for
a plant 3–7× weaker than reality, so on the real airframe the loop gain is 3–7×
too high → over-correction → the saturation limit cycle. The rig diverges
(pinned, no aero damping) where the real craft instead saturates into a *bounded*
~2 Hz cycle and crashes — same instability, bounded differently.

This also explains the catch-22 the real INDI b-fit hit: once the plant is hot
enough to be realistic, the loop hunts, so you cannot cleanly fit on top of it.

## Modeled parity parameters

Apply to the vsim geometry (`DroneParams`/`MotorParams`, pushed via
`VSIM_CTL_SET_GEOMETRY` / the GCS conf). The first two are directly fit; the rest
close known model gaps the sim does not yet represent.

| param | sim default | parity value | basis |
|---|---|---|---|
| `inertia I_xx` (roll) | 0.00683 | **0.0023** (×0.33) | match measured `K_roll` 187→563 |
| `inertia I_yy` (pitch) | 0.00739 | **0.0011** (×0.14) | match measured `K_pitch` 199→1381 (also encodes the real pitch/roll 2.45× asymmetry) |
| thrust margin (`k_thrust` or `mass`) | hover 0.25 | hover **0.45** → `k_thrust ×0.31` *or* `mass ×3.2` | real airborne throttle |
| actuator `tau` | 12.5 ms | **~21 ms** | onhw roll sysid τ = 20.9 ms |
| per-motor `k_thrust` asymmetry | equal | weaken the burned arm (needs props-off bench thrust) | real trim + burned motor |
| idle-stall / ESC deadband | none (linear, instant) | thrust→0 below ~0.05–0.08 duty + re-spin lag | real motors floored at 0.005 stall/desync — the sim's benign idle is *why* the floor is harmless in sim |
| vibration injection | minimal | accel σ ≈ 0.3–1.5 g, scaling with total thrust | real `|acc|` RMS 0.06 g idle → 1.5 g active |

> Note: matching `K` by inertia **alone** overshoots into divergence, because the
> real cycle is *bounded by actuator saturation* and shaped by the idle-stall and
> asymmetry the sim lacks. Realistic parity needs the effectiveness fix **and**
> the actuator-nonlinearity/vibration model gaps, together — not inertia in
> isolation.

## Files

- `collect_parity.py` — headless data collection (venv python).
- `fit_parity.py` — spectral/lock-in `K` fit + hover stats (system numpy).
- `parity_plot.py` — the figure.
- `*.csv` — raw captures. `parity.conf` — the inertia-scaled verification geometry.
