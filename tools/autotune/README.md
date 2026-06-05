# SITL PID Autotuner (MVP)

Closed-loop PID gain search against the headless SITL stack. Launches the
physics daemon (`vsim_d`) + the firmware (`vayu_sitl`) in **test-rig mode**
(translation pinned, rotation free — a frictionless attitude gimbal), excites
roll/pitch attitude step doublets, reads the firmware's control-telemetry
(setpoint vs. measured for every loop) and minimizes a tracking cost.

The firmware control code is identical to hardware; only the HAL is stubbed.

## Prerequisites (build the two binaries once)

```bash
cmake --build build_vsim  --target vsim_d     -j$(nproc)
cmake --build build_sitl  --target vayu_sitl  -j$(nproc)
```

## Run

```bash
cd tools/autotune

# tune with SPSA (default), 50 rollouts, then apply + persist the best gains
python3 autotune.py --optimizer spsa --budget 50 --apply

# compare every optimizer (incl. hybrid + portfolio) on equal budgets
python3 autotune.py --compare --budget 40

# average N rollouts per evaluation to cut cost noise
python3 autotune.py --optimizer hybrid --budget 60 --repeats 2
```

Results are printed and written to `autotune_result.json` (per-optimizer best
cost, gains, and the full convergence history).

## What gets tuned

The cascaded **angle → rate** loop, symmetric across roll & pitch (4 params):
`rate Kp, rate Ki, rate Kd, angle Kp`. Pass **`--yaw`** to also tune yaw as its
own axis (4 more params + a yaw doublet) — yaw is now enabled in firmware
(`include/variables.h`; it was previously zeroed/disabled). The angle loop is
P-only. Edit `_BASE` / `_YAW` in `autotune.py` to change the space/bounds/seed
(seeds are the firmware `VAYU_SIM` defaults).

## Optimizers (`--optimizer`)

| name          | method                                                        |
|---------------|---------------------------------------------------------------|
| `spsa`        | Simultaneous-perturbation stochastic gradient (2 evals/step)  |
| `fdgd`        | Central finite-difference gradient descent (2N evals/step)    |
| `coordinate`  | Pattern / coordinate descent with step shrink                 |
| `nelder-mead` | Downhill simplex                                              |
| `random`      | Uniform random search (baseline)                              |
| `hybrid`      | Coarse random exploration → SPSA local refine                 |
| `portfolio`   | Splits budget across SPSA + Nelder-Mead + coordinate, keeps best |

`--compare` runs them all and ranks them.

## Cost function

Per axis, from the step-doublet response: mean tracking error (IAE, deg) +
overshoot penalty + oscillation penalty, summed over roll & pitch. Divergent /
failsafe gains (|angle| > 80°, NaN) get a large penalty so the search stays in
the stable region. See `_axis_cost` in `autotune.py`.

## Architecture

```
autotune.py ── cost / excitation / CLI / multi-optimizer compare
   optimizers.py ── SPSA, FDGD, coordinate, Nelder-Mead, hybrid, portfolio
   sitl.py ──────── launches & drives the headless stack:
        vsim_d  ←ctl(reset/testrig/rates)──  RC over a PTY ──→ vayu_sitl
        vayu_sitl ──control-telemetry + CMD_SET_PID over the UART2 PTY──→ here
   protocol.py ──── NavLink (CRC32, decode control-telemetry, encode SET_PID/ARM)
                    + vsim ctl/pose frame (de)serialization
```

## Notes & limitations (MVP)

- **Test-rig mode** is a new `vsim_d` control message (`VSIM_CTL_SET_TESTRIG`,
  opcode 12). It pins the 3 translational DOF and leaves rotation free.
- RC is fed over a **PTY** (not a FIFO) because the firmware calls `tcgetattr`
  on `VAYU_UART_RC_PATH`; a plain FIFO makes it fall back to synthetic hover.
- Rollouts run in **real time** (vsim_d is wall-clock paced) — ~5 s each. A
  faster-than-real-time "turbo" mode for `vsim_d` would speed this up a lot.
- The sim uses `vsim_d`'s default mass/inertia, not your airframe geometry yet.
  For hardware-transferable gains, push real geometry (`VSIM_CTL_SET_GEOMETRY`)
  and validate on a tethered bench — **never auto-tune on hardware unattended.**
- RC stick→angle has strong (cubic) expo, so the step uses ~1800 µs (~21°).
