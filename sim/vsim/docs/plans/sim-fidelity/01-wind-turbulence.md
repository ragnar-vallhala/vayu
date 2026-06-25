# Plan: wind & turbulence

Status: ✅ shipped. Gap doc Tier-1 #1. Implemented via `sim/vsim/include/wind_model.h`,
the `VSIM_CTL_SET_WIND` opcode (`vsim_proto.h`), `SimWorker::sendWind`, and the
`WorldEditorWidget::windApplied` wiring (see [00-phasing.md](00-phasing.md), Phase 1 ✅).
Adds a world-frame wind field — steady
component + periodic gusts + band-limited turbulence — that the airframe feels as
a relative-velocity drag force. This is the single highest-value sim-fidelity
addition: it turns the sim into a genuinely harder, more honest test for the
autotuner (a controller that only ever saw still air is not validated).

Pairs with [02-atmosphere-aero.md](02-atmosphere-aero.md): both consume the same
**air-relative velocity** `v_rel = v_world − v_wind`. Build wind first; atmosphere
reuses `v_wind`.

## Physics model

Wind is a world-frame velocity field `v_wind(t)` [m/s, NED] with three additive
parts:

```
v_wind(t) = v_steady + v_gust(t) + v_turb(t)
```

- **Steady:** constant `v_steady = (windN, windE, windD)`.
- **Gust (periodic, deterministic):** a `1−cos` ramped sinusoid so it's smooth and
  repeatable (good for A/B tuning comparisons):
  ```
  v_gust(t) = gustAmp · sin(2π t / gustPeriod) · ĝ
  ```
  `ĝ` = horizontal unit vector along `v_steady` (or N if steady is zero). `gustAmp`
  [m/s], `gustPeriod` [s].
- **Turbulence (stochastic, band-limited):** discrete **Dryden**-style first-order
  shaping filter driven by white noise — far more realistic than raw white noise and
  cheap:
  ```
  v_turb[k] = α · v_turb[k−1] + σ_turb · √(1−α²) · N(0,1)   per axis
  α = exp(−dt / T_turb)
  ```
  `σ_turb` = turbulence intensity [m/s] (mockup `windTurb`), `T_turb` = correlation
  time (derive from a length scale `L` and airspeed, or expose a fixed ~1 s default).
  The `√(1−α²)` factor makes the steady-state RMS equal `σ_turb` regardless of `dt`.

The force on the airframe is **relative-velocity linear drag** — reuse the existing
`linear_drag` coefficient rather than inventing a second one:

```
v_rel   = s.vel_w − v_wind(t)                 // world frame
F_drag  = −linear_drag · v_rel                // world frame, N
```

Today `derive()` (physics_core.cpp line 30) applies `−linear_drag/mass · vel_w`
(drag toward the still world). The change is to subtract `v_wind` from `vel_w`
first, so a hovering drone in steady wind is *pushed* (the drag no longer zeroes at
zero ground speed). That single substitution is the whole physics change for steady
wind; gust+turb just make `v_wind` time-varying.

## Protocol

`sim/vsim/include/vsim_proto.h`:

```c
VSIM_CTL_SET_WIND = 14,           // body: vsim_ctl_wind_t
```

```c
typedef struct {
    float steady[3];      // v_steady NED [m/s]   (windN, windE, windD)
    float gust_amp;       // [m/s]                (windGust)
    float gust_period;    // [s], <=0 disables    (windPeriod)
    float turb_sigma;     // turbulence RMS [m/s] (windTurb)
    float turb_tau;       // correlation time [s] (default 1.0 if <=0)
    int32_t enable;       // 0 = no wind at all (fast bypass)
} vsim_ctl_wind_t;        // 32 B
```

## Daemon

`sim/vsim/src/main.cpp`, new `case` in the dispatch switch (~line 408, beside
`SET_FAULTS`):

```c
case VSIM_CTL_SET_WIND: {
    vsim_ctl_wind_t w; std::memcpy(&w, cmd.body, sizeof(w));
    ctl.setWind(w);   // stores config on the controller / env state
    std::fprintf(stderr, "vsim_d: wind steady=(%.1f,%.1f,%.1f) gust=%.1f/%.1fs turb=%.2f\n",
                 w.steady[0], w.steady[1], w.steady[2], w.gust_amp, w.gust_period, w.turb_sigma);
    break;
}
```

The controller owns a `WindState` (config + the running `v_turb[3]` filter state +
a sim-time accumulator). Advance it once per physics step and expose
`Vec3 windAt(double t, float dt)`.

> RNG note: the turbulence filter must use the **sim's** seeded RNG (the same one
> `SET_NOISE` uses, see [05-sensor-error-models.md](05-sensor-error-models.md)), not
> a fresh `rand()`, so runs are reproducible for tuner A/B comparison.

## Physics integration

In `PhysicsCore::derive()` (physics_core.cpp ~line 28–31), replace the still-air
drag with wind-relative drag. The cleanest seam: have the daemon compute
`v_wind = ctl.windAt(t, dt)` once per `step()` and pass it in (extend `step()`
signature or stash it on `PhysicsCore` via a setter `setWind(Vec3)` called each
tick). Then:

```cpp
// was: a -= s.vel_w * (linear_drag / mass);
Vec3 v_rel = s.vel_w - wind_w_;
a -= v_rel * (params_.linear_drag / params_.mass);
```

RK4 caveat: `v_wind` is sampled once per `step()` (held constant across the 4
sub-evaluations) — correct to first order and standard for environment forcing.

## GCS — SimWorker

`navigator/src/vsim/SimWorker.{h,cpp}`, mirror `sendNoise` (lines ~205–224):

```cpp
void sendWind(const WindConfig& w);   // builds VSIM_CTL_SET_WIND frame, ::write(ctl_fd_, …)
```

`WindConfig` = the six fields above (POD in SimWorker.h beside `WorldConfig`).

## GCS — UI (World tab)

`navigator/src/ui/widgets/SimulatorWidget.cpp`, a new "Wind & Turbulence" group box in
the World tab (~after the aerodynamics section, near line 533). Mirror the mockup
(`app.js` `updateEnv()` ~705–736):

| Mockup id | Control | Field |
|-----------|---------|-------|
| `windN/windE/windD` | 3× `QDoubleSpinBox` | `steady[]` |
| `windGust` | `QDoubleSpinBox` | `gust_amp` |
| `windPeriod` | `QDoubleSpinBox` | `gust_period` |
| `windTurb` | `QDoubleSpinBox` | `turb_sigma` |
| `gWindSpd` / `gWindDir` | 2× live mini-plot | computed readout |

Wire every spinbox `valueChanged` → a `pushWind()` slot (live-apply, like
`pushNoise()` line ~937) → `m_sim->sendWind(cfg)`. This is a **hot** group — do not
lock it in `setMode()`.

**Live graphs / HUD:** the daemon should publish the instantaneous `v_wind` so the
GCS can plot speed/direction without recomputing. Cheapest path: add `wind_w[3]` to
the pose frame (`vsim_pose_frame_t` → `SimSnapshot`, SimWorker.cpp ~414–432) and
feed `#gWindSpd = ‖wind_xy‖`, `#gWindDir = atan2(E,N)`. A wind-vane glyph + speed
readout in `SimHudWidget` (lower-right) reuses the existing rolling-history plot
helper (SimHud paintEvent ~270–311).

## Tests

- **Unit (daemon-side, headless):** drive `WindState` for N steps with a fixed seed;
  assert (a) steady-only → constant `v_wind`; (b) gust → sinusoid of the right
  amplitude/period; (c) turbulence RMS over a long window ≈ `turb_sigma` (the
  `√(1−α²)` normalisation is the thing to verify) and mean ≈ 0.
- **Physics:** hover the drone (gravity-cancelling thrust) in steady 5 m/s N wind;
  assert it accelerates north then settles at `v_rel ≈ 0` drift (i.e. ground speed →
  wind speed) — the "weathervane drift" sanity check.
- **Manual SITL:** enable turbulence mid-flight, confirm the autotuner cost rises
  (harder plant) and the wind graphs animate.

## Effort

~1–1.5 days. Physics change is tiny (one substitution + a small filter); most of the
work is the protocol/daemon/UI plumbing and the two live graphs.
