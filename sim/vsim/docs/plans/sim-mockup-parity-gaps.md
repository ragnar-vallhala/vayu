# Simulator mockup-parity gaps

Status: 🗺️ gap analysis. What the UI mockup (`software/docs/ui-mockup/`,
`index.html` + `app.js`) shows for the **Simulator** that the real
`SimulatorWidget` + `tools/vsim` physics do **not** implement yet. Companion to
[mockup-to-app.md](../../../../software/docs/reference/mockup-to-app.md); the broad mockup-parity effort already
shipped the app shell + most pages, so this drills into the sim specifically.

**Headline:** autotune and the core sim are essentially at parity. The gaps
cluster in three areas — **environment / atmosphere realism**, **sensor error
models**, and **power / extended telemetry**. The single highest-value cluster is
wind + atmosphere (a real physics addition that makes the sim a harder, more
honest test for tuning).

## Tier 1 — whole subsystems absent

| # | Feature | Mockup (id / fn) | Implementation status |
|---|---------|------------------|------------------------|
| 1 | **Wind & turbulence** — steady wind N/E/D, gust amplitude/period, turbulence intensity, + live wind-speed/direction graphs | `windN/windE/windD`, `windGust`, `windPeriod`, `windTurb`, `gWindSpd/gWindDir`; `updateEnv()` (app.js ~708) | **✅ Shipped.** World-frame wind field (`wind_model.h`, `VSIM_CTL_SET_WIND`, `SimWorker::sendWind`, `WorldEditorWidget::windApplied`) — see [`sim-fidelity/01-wind-turbulence.md`](sim-fidelity/01-wind-turbulence.md). |
| 2 | **Atmosphere / aero** — air density, ground effect (enable + height + live GE-factor graph), air-relative airspeed (+ graph) | `airDensity`, `geOn`, `geHeight`, `geFac`, `airSpd`, `gGndEff`, `gAirSpd` | **Absent.** Constant gravity only; no density, ground-effect, or airspeed. |
| 3 | **GPS + GPS glitch** — GPS sensor + 4 glitch modes (single jump / constant offset / position-file / timed-CSV) | `gpsGlitchEn`, `gpsGlitchMode`, jump/offset N/E/D, file/CSV loaders | **Absent.** Fault checkbox is a **disabled placeholder** ("No GPS is simulated yet"). |
| 4 | **Barometer sensor** | sensor list `buildSensors()` (app.js ~1294) | **Absent.** No pressure/altitude sensor sim. |
| 5 | **Magnetic-field environment** — total intensity / declination / inclination → computed N/E/D field vector | `magMag/magDec/magInc/magVec`; `updateMagVec()` (app.js ~1334) | **Absent** (software models mag *sensor noise* but not the *field* config). Verify. |
| 6 | **Power model** — battery V / current / mAh | sim telemetry readouts | **Absent.** No battery/power state. |
| 7 | **Advanced sensor error models** — per sensor: scale-factor, saturation, spikes/outliers, vibration (blade-pass ∝ throttle), latency, temp-dependent bias; CSV noise replay; Gaussian/CSV source selector; editable per-sensor rate | `SENS_COMPS` (app.js ~1279), `setNoiseSrc()` | **Shallow.** Software has only white-noise σ + bias-clip + enable for accel/gyro/mag (`SimWorker::sendNoise`). |

## Tier 2 — partial / shallower than the mockup

| # | Feature | Mockup | Implementation status |
|---|---------|--------|------------------------|
| 8 | **Autotune live dashboard** — 4 plots (attitude, body-rates, controller-outputs, cost-convergence) in the 3-pane reflow | `gAtAng`, `gAtRate`, `gAtOut`, `gAuto`; `atDash()` (app.js ~1062) | Software has the single **roll** `ResponsePlot` + gains table + a "live dashboard" checkbox. Verify whether the dashboard opens the richer set; add rates/outputs/cost plots if not. |
| 9 | **RC mapping UI** — 4-channel table (visual bar + µs + **invert**), USB-joystick source, UART device/baud | `rcm0..3`, `rcv0..3`, invert checkboxes | RcBridge + acro + UART connect/baud exist; the per-channel mapping/invert table + joystick source are richer in the mockup. |
| 10 | **Spawn pose** — editable spawn N/E/D + heading | HTML ~641 | Reset spawns at hardcoded ~5 cm; arbitrary pose only via the **rig** controls. No dedicated spawn-pose config. |
| 11 | **Configurable surface friction** (Coulomb μ) | HTML ~612 | Physics uses a fixed tangential factor; not user-configurable. |

## Tier 3 — minor / convenience

| # | Feature | Mockup | Implementation status |
|---|---------|--------|------------------------|
| 12 | **Sim speed selector** (1.0× / 0.5× / 2.0× / Max) | toolbar dropdown HTML ~454 | Absent. |
| 13 | **Motor RPM / ESC temperature readouts** | sim telemetry | RPM data exists internally (`motor_omega`) but isn't displayed; ESC temp absent. |
| 14 | **Motor-coefficient "copy from motor N"** | `copyMotorCoef()` (app.js ~1252) | Convenience; absent. |
| 15 | **Pause / resume** | (start/stop only) | Protocol has `VSIM_CTL_PAUSE` (`vsim_proto.h`); no UI. |

## At parity (no action needed)
Sim start/stop/reset + status/pose; world mesh (.glb/.obj) + ground + obstacles
(box/sphere/cylinder, add/remove/edit); vehicle mesh import + mass + **full 3×3
inertia / CoM**; motor position/axis/spin/k\_thrust/k\_moment/max-ω/**τ** editor;
rig/tether (hard pin + soft critically-damped spring) + pose sliders; FPV HUD +
horizon & down-cam PiPs (draggable/resizable) + toggles; accel/gyro/mag noise
(σ + bias-clip + enable/dropout); **fault injection** (motor-kill M1–M4, RC-loss,
IMU dropout); **autotune** (optimizer choice, budget, excitation amplitude,
step/chirp + f0–f1, repeats, soft-rig tether, tune-yaw, compare-all-optimizers,
throttle-buzz check, free-flight validation, optimizer + sim-noise seeds) +
apply-to-firmware; prop audio.

## Detailed plans
Per-subsystem implementation plans for the large Tier-1 tasks now live in
[`sim-fidelity/`](sim-fidelity/README.md) (wind & turbulence, atmosphere/aero,
magnetic field, power model, advanced sensor error models). **GPS/glitch (#3) and
barometer (#4) are deliberately deferred** there — they need an estimator consumer
to be meaningful.

## Suggested build order
1. **Wind + atmosphere (Tier 1 #1, #2).** The highest-value addition and a real
   physics change: add wind (steady + gust + Dryden-ish turbulence), air density,
   ground effect, and air-relative airspeed to `physics_core.cpp` + the vsim
   control protocol; surface the controls + the four live env graphs in the World
   tab. Makes the sim a meaningfully harder test for the autotuner.
2. **Rich sensor error models (Tier 1 #7).** Extend `SET_NOISE` (scale/sat/
   spikes/vibration/latency/temp-bias + CSV replay + per-sensor rate) and the
   sensor panel; add barometer (#4). Improves sim-to-hardware transfer fidelity.
3. **GPS + glitch (#3)** and **magnetic-field config (#5)** — needed once
   position/heading estimation is exercised in the sim.
4. **Power model (#6)** + **RPM/ESC-temp readouts (#13)** — enables the
   battery-sag / thermal realism that the live-rig tuning study flagged as a
   real-world non-stationarity (see [gcs-live-rig-tuning.md](../../../../software/docs/scratch/gcs-live-rig-tuning.md)).
5. **Tier 2/3 polish** — autotune 4-plot dashboard, RC mapping table, spawn-pose
   config, sim-speed selector, copy-coefficients, pause.

## Sources
- Mockup: `software/docs/ui-mockup/{index.html,app.js,styles.css}` (Simulator
  page ~HTML 434–747; sim JS ~app.js 703–1341).
- Implementation: `software/src/ui/widgets/SimulatorWidget.{h,cpp}`,
  `SimHudWidget.*`, `software/src/vsim/SimWorker.*`, `software/src/autotune/`,
  `tools/vsim/src/physics_core.cpp`, `tools/vsim/include/vsim_proto.h`.

## Changelog

| Date       | Author          | Description                          |
| ---------- | --------------- | ------------------------------------ |
| 16/06/2026 | ragnar-vallhala | Initial simulator mockup-parity gap analysis |
