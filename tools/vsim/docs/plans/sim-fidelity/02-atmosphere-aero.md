# Plan: atmosphere & aerodynamics

Status: 🔴 planned. Gap doc Tier-1 #2. Adds three coupled atmosphere effects:
**air density** (scales thrust + drag), **ground effect** (extra lift near the
ground), and **air-relative airspeed** (the readout + the quantity drag actually
depends on). Builds directly on [01-wind-turbulence.md](01-wind-turbulence.md) —
airspeed is `‖v_world − v_wind‖`, so wind must land first (or ship together).

## Physics model

### Air density `ρ`
Today thrust is `k_thrust · ω²` with `k_thrust` implicitly folded for sea-level air.
Make density an explicit multiplier so altitude/temperature matter:

```
thrust_i = ρ_ratio · k_thrust[i] · ω_i²       // ρ_ratio = ρ / ρ_0,  ρ_0 = 1.225 kg/m³
```

`ρ_ratio` also scales aerodynamic drag (drag ∝ ρ). Expose `airDensity` directly
(mockup `airDensity`); `ρ_ratio = airDensity / 1.225`. Default 1.0 = no change, so
this is a pure superset of current behaviour.

### Ground effect
Rotor thrust rises near the ground (cushion of air). Standard Cheeseman–Bennett
form, matched to the mockup's curve (`app.js` ~724):

```
GE(z) = 1 + k_ge / (1 + (z / h_ref)²)         // z = height AGL [m]
thrust_i *= GE(z)                              // when geOn
```

mockup uses `k_ge = 0.35`, `h_ref = geHeight`. `z = −s.pos_w.z() − ground_z`
(NED). Clamp `z ≥ 0`. As `z → ∞`, `GE → 1` (effect vanishes). This is a thrust
multiplier applied in the motor model, identical seam to the density multiplier.

### Air-relative airspeed & drag
The drag in [01](01-wind-turbulence.md) already uses `v_rel = v_world − v_wind`.
Here we (a) scale that drag by `ρ_ratio`, and (b) surface `airspeed = ‖v_rel‖` as a
first-class readout (mockup `airSpd`, `gAirSpd`).

```
F_drag = −ρ_ratio · linear_drag · v_rel
airspeed = ‖v_rel‖
```

(If/when a richer aero model is wanted, this is where a `½ρ Cd A v_rel²` quadratic
term would replace the linear coefficient — out of scope for parity, noted for
future.)

## Protocol

`tools/vsim/include/vsim_proto.h`:

```c
VSIM_CTL_SET_ATMOS = 15,          // body: vsim_ctl_atmos_t
```

```c
typedef struct {
    float air_density;    // kg/m^3            (airDensity; 1.225 = SL default)
    int32_t ge_enable;    // ground effect on  (geOn)
    float ge_height;      // h_ref [m]         (geHeight)
    float ge_gain;        // k_ge (default 0.35 if <=0)
} vsim_ctl_atmos_t;       // 16 B
```

## Daemon

`tools/vsim/src/main.cpp`, new `case` beside `SET_WORLD` (~line 292):

```c
case VSIM_CTL_SET_ATMOS: {
    vsim_ctl_atmos_t a; std::memcpy(&a, cmd.body, sizeof(a));
    ctl.setAtmos(a);
    std::fprintf(stderr, "vsim_d: atmos rho=%.3f ge=%d h=%.2f\n",
                 a.air_density, a.ge_enable, a.ge_height);
    break;
}
```

Store `ρ_ratio` and the GE config on the controller; pass both into the motor model
each step.

## Physics integration

`tools/vsim/src/motor_model.cpp`, the thrust line (`thrust_i = k_thrust[i] *
omega_[i]²`, ~line 32) becomes:

```cpp
float ge = ge_on_ ? (1.0f + ge_gain_ / (1.0f + (z_agl/ge_h_)*(z_agl/ge_h_))) : 1.0f;
float thrust = rho_ratio_ * ge * k_thrust[i] * omega_[i]*omega_[i];
```

`MotorModel::update()` needs two new inputs: `rho_ratio_` (scalar) and `z_agl`
(passed from `PhysicsCore` which knows `pos_w.z()` and `ground_z`). Reaction torque
(`k_moment · ω²`, line 50) also scales with `ρ_ratio` (aero torque), but **not** with
ground effect (GE is a lift phenomenon) — scale only by `rho_ratio_` there.

Density-scaled drag: in `derive()` multiply the (already wind-relative) drag term by
`rho_ratio_` — one extra factor on physics_core.cpp line 30.

## GCS — SimWorker

`software/src/vsim/SimWorker.{h,cpp}`:

```cpp
void sendAtmos(const AtmosConfig& a);   // VSIM_CTL_SET_ATMOS, mirrors sendWorld
```

## GCS — UI (World tab)

New "Air & Thrust" group in the World tab (beside Wind). Mirror mockup ids:

| Mockup id | Control | Field |
|-----------|---------|-------|
| `airDensity` | `QDoubleSpinBox` (default 1.225) | `air_density` |
| `geOn` | `QCheckBox` | `ge_enable` |
| `geHeight` | `QDoubleSpinBox` | `ge_height` |
| `geFac` | read-only label | live `GE(z)` |
| `gGndEff` | mini-plot | live `GE(z)` history |
| `airSpd` / `gAirSpd` | label + mini-plot | `‖v_rel‖` |

Wire to a `pushAtmos()` slot (hot group). The live readouts (`geFac`, `airSpd`) come
from the pose stream: add `airspeed` and `ge_factor` floats to `vsim_pose_frame_t` →
`SimSnapshot` (SimWorker ~414–432) so the GCS displays the *actual* simulated values
rather than re-deriving them.

## HUD

`SimHudWidget`: add an **airspeed** tape on the left beside the existing ground-speed
tape (paintEvent ~218) — the gap between them visualises headwind/tailwind at a
glance. Add a small "GE ×1.34" annotation near the altitude tape when ground effect
is active.

## Tests

- **Unit:** `GE(0) = 1 + k_ge`; `GE(h_ref) = 1 + k_ge/2`; `GE(∞) → 1`. Density:
  `thrust(ρ_ratio=2) = 2·thrust(1)`.
- **Physics:** at fixed hover throttle, lowering the drone into `z < h_ref` should
  make it climb (GE adds lift); set `air_density` low (high altitude) and confirm the
  same throttle no longer holds hover (thrust deficit).
- **Manual SITL:** sweep `airDensity` and watch the hover throttle the autotuner
  needs shift; confirm `airSpd` readout matches `‖v_world − v_wind‖`.

## Effort

~1 day on top of wind. Two multipliers in the motor model + one in drag, plus the
UI group and three pose-frame readouts.
