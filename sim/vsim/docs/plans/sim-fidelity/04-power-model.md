# Plan: battery / power model

Status: 🔴 planned. Gap doc Tier-1 #6. The sim has no electrical state — the mockup
shows battery voltage / current / consumed mAh as sim telemetry. This plan adds a
lumped battery + ESC/motor electrical model that derives current draw from the motor
state already simulated, integrates consumed charge, and sags voltage under load.

Why it matters beyond a readout: **battery sag is a real non-stationarity** the
live-rig tuning study flagged — thrust per PWM drops as the pack drains, so a tune
done at full charge degrades. Simulating it lets the autotuner (and the operator) see
that effect before hardware. See
[`gcs-live-rig-tuning.md`](https://github.com/ragnar-vallhala/vayu-navigator/blob/main/navigator/docs/scratch/gcs-live-rig-tuning.md).

## Model

A lumped, per-step electrical model driven by motor angular velocity (already in the
motor model as `omega_[i]`):

**Per-motor electrical power** — mechanical shaft power plus losses, approximated
from rotor speed (mechanical power of a rotor ∝ ω³):

```
P_i = k_power · ω_i³ + P_idle              // W, per motor
P_total = Σ P_i                            // W
```

`k_power` is a config coefficient (relate to `k_thrust`/`k_moment` scale; expose it
directly). `P_idle` covers FC/ESC quiescent draw.

**Battery (Thevenin / internal-resistance model):**

```
I = P_total / V_terminal                   // A   (solve; or use V_nominal for stability)
V_terminal = V_oc(soc) − I · R_internal    // V, sag under load
```

`V_oc(soc)` = open-circuit voltage vs state-of-charge — a small piecewise-linear LUT
across the discharge curve (e.g. 4.2 V→3.0 V per cell), scaled by cell count `S`.
To avoid the algebraic loop (`I` depends on `V`, `V` depends on `I`), use the
previous step's `V_terminal` to compute `I` — fine at physics rates.

**Charge integration:**

```
mAh_used += I · dt · (1000/3600)           // accumulate
soc = 1 − mAh_used / capacity_mAh
```

**Coupling back to physics (the valuable part):** scale max thrust by the sagging
voltage so a draining pack actually flies worse:

```
voltage_ratio = V_terminal / V_full
max_omega_eff[i] = max_omega[i] · voltage_ratio
```

This makes hover throttle creep up as the pack drains — the non-stationarity the
study cares about. Gate it behind a `couple_thrust` flag so power can also be a
pure readout if desired.

## Protocol

Two directions: **config in**, **state out**.

### Config (GCS → daemon)
`sim/vsim/include/vsim_proto.h`:

```c
VSIM_CTL_SET_POWER = 17,          // body: vsim_ctl_power_t
```

```c
typedef struct {
    int32_t enable;        // power model on
    int32_t cells_s;       // series cell count (S)
    float capacity_mAh;    // pack capacity
    float r_internal;      // ohms (total pack)
    float k_power;         // W per (rad/s)^3
    float p_idle;          // W quiescent
    float soc_init;        // initial state of charge [0..1]
    int32_t couple_thrust; // 1 = sag reduces max thrust
} vsim_ctl_power_t;        // 32 B
```

### State (daemon → GCS)
Power state rides the **existing pose frame** so no new stream is needed. Extend
`vsim_pose_frame_t` (and `SimSnapshot`, SimWorker.cpp ~414–432):

```c
float batt_voltage;    // V terminal
float batt_current;    // A
float batt_mah_used;   // consumed
float batt_soc;        // [0..1]
```

## Daemon

`sim/vsim/src/main.cpp`, new `case`:

```c
case VSIM_CTL_SET_POWER: {
    vsim_ctl_power_t p; std::memcpy(&p, cmd.body, sizeof(p));
    ctl.setPower(p);
    std::fprintf(stderr, "vsim_d: power %dS %.0fmAh Ri=%.3f couple=%d\n",
                 p.cells_s, p.capacity_mAh, p.r_internal, p.couple_thrust);
    break;
}
```

Reset (`VSIM_CTL_RESET`, ~line 224) should also reset `soc`/`mAh_used` to
`soc_init` so each run starts from a known charge.

## Physics integration

A `PowerModel` advanced once per `PhysicsCore::step()` after the motor model has
updated `omega_[i]`:

1. Sum `P_i` from current `omega_[i]`.
2. Compute `I`, `V_terminal` (using prev-step `V`), integrate `mAh_used`, update
   `soc`.
3. If `couple_thrust`, set `voltage_ratio` and apply it to the motor model's
   effective `max_omega` for the *next* step (motor_model.cpp ~line 18, where
   `target_omega = duty · max_omega`).
4. Stamp the four floats into the outgoing pose frame.

## GCS — SimWorker & UI

`SimWorker`:

```cpp
void sendPower(const PowerConfig& p);   // VSIM_CTL_SET_POWER
```

`SimulatorWidget` — a "Power / Battery" group (vehicle or world tab):
config spinboxes for cells/capacity/R/k_power/idle/soc + a `couple_thrust` checkbox,
wired to `pushPower()` (hot group).

**Readouts:** `SimHudWidget` gains a battery block (paintEvent, near the status text
~243): `V` (color-warn below a per-cell threshold), `A`, `mAh`, and a SoC bar reusing
the throttle-bar drawing helper (~229–241). Optionally a small voltage-vs-time plot.

## Tests

- **Unit:** zero throttle → `I ≈ P_idle/V`, `mAh` accrues slowly; full throttle →
  higher `I`, faster drain; `V_terminal` monotonically decreases as `soc` falls;
  `V_terminal < V_oc` whenever `I > 0` (sag sign check).
- **Coupling:** with `couple_thrust` on, hold fixed throttle and confirm altitude
  decays as the pack drains (effective thrust drops) — the headline behaviour.
- **Manual SITL:** run the autotuner near a low SoC and confirm the higher hover
  throttle / different optimum vs full charge.

## Effort

~1 day. The model is small; most effort is the two-way protocol (config in + 4 state
fields out on the pose frame) and the HUD battery block. Coupling-to-thrust is the
one subtle piece — keep it flagged and default-off until validated.
