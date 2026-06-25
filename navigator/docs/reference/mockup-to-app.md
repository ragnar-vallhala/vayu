# Mockup → app: reconciling the Navigator UI mockup with the codebase

The interactive mockup in [`ui-mockup/`](ui-mockup/) is the design target
for Navigator's UI. This doc reconciles it against what the app **already has**,
so "implement all this" stays honest: most of the mockup is *already built* or
in flight — only a few initiatives are genuinely new.

The mockup's own **analysis mode** (`Ctrl+D`) tags every element by
implementation difficulty (🔴 structural / 🟡 additive / 🟢 easy). Use this doc to
map those verdicts onto real source and the existing [`requirements.md`](requirements.md)
phase plan.

## What the mockup shows that the app already has

A large share of the mockup corresponds to shipped or in-progress widgets — the
mockup is partly a *catch-up* spec, not all new work:

| Mockup screen / feature | Already in the app | Requirement |
|---|---|---|
| Attitude / PFD, 3D airframe | `AttitudeWidget`, `HorizonHud`, `Drone3DWidget` | FR-UI / §2.3 |
| IMU + telemetry sparklines | `ImuPanel`, `RealTimeGraph` | FR-UI-* |
| RC channels | `RcChannelsWidget` | §2.4 |
| Motors + per-motor coefficients / thrust axis | `MotorStatusWidget`, `GeometryEditorWidget` | FR-SIM-11 ✅ |
| Control loop plots | `ControlLoopPlot` | §2.4 |
| Calibration | `CalibrationWidget` | §2.4 |
| Packet analyzer + decoder + **filter expression** + **freq ribbon** | `PacketAnalyzerWidget`, `PacketDetailWidget`, `PacketFilterExpr`, `FrequencyRibbon` | §2.1 |
| Kernel perf | `PerfWidget` | §2.8 |
| Link stats / network | `LinkStatsPanel` | §2.8 |
| Simulator + world config | `SimulatorWidget`, `WorldEditorWidget`, `SimRendererWidget`, `vsim_d` | §2.6 |
| Autotune live plots | `TuneChart` | (sim tuning) |
| Settings panes | `SettingsManager` + settings widgets | §2.7 |
| Theme, toasts, tooltips, basic shortcuts (`Ctrl+1..8`,`K`,`L`,`F11`) | `Theme`, `Notify`, Phase-0 0f–0o | Phase 0 ✅ |
| Status bar, LIVE indicator, view nav | `MainStatusBar`, `MainToolbar`, `QStackedWidget` | §4 |

> When the mockup differs from one of these (e.g. the gated calibration wizard,
> the two-bar replay scrubber, sensor-noise injection FR-SIM-04), treat the
> mockup as the **updated spec** for that widget — an *enhancement*, not a rebuild.

## What is genuinely new (the actual roadmap)

Only three initiatives in the current mockup have no home in the codebase yet.
They are the real subject of "implement all this":

1. **Whole-GCS log replay + crop/loop scrubber** — 🔴 structural.
   Refines and widens the planned Phase-2 `2c` (FR-LOG-05 / FR-UI-19, `src/replay/`)
   from "play a `.bin` through `DroneProtocol`" into a *session-wide* read-only
   replay where every panel is fed from the log and a crop region loops a
   sub-range. → [gcs-log-replay.md](../journal/shipped/gcs-log-replay.md)

2. **Editable command/shortcut system + command palette + MRU view switcher** —
   🔴/🟡. Generalises the ad-hoc `QShortcut`s from Phase-0 `0l` into a central
   command registry with editable keybindings, a fuzzy palette, and a
   Firefox-style `Ctrl+`` ` recent-views cycle. **FR-UX-19 / FR-UX-20 / FR-UX-21**.
   → [command-registry-and-shortcuts.md](../journal/shipped/command-registry-and-shortcuts.md)

3. **Help surface — Documentation + About** — 🟢 easy.
   `AboutDialog` (static, version/build) + Documentation entry that opens the
   bundled docs via `QDesktopServices::openUrl`. Folded into the command-registry
   doc (both hang off the menu/command layer). **FR-UX-22 / FR-UX-23**.

Everything else the mockup added this cycle is an *enhancement of an existing
widget* and belongs as a line item against that widget's requirement — see the
table above and the per-widget notes in the initiative docs.

## Not for the app

- **Analysis mode (`Ctrl+D`)** is a *mockup-only* design instrument — it
  annotates the prototype with implementation verdicts. It does **not** ship in
  Navigator; it lives only in `ui-mockup/`.

## Suggested ordering

These slot into the existing phase plan rather than replacing it:

- **Now / Phase 1 tail:** the command registry + editable shortcuts + palette +
  MRU (independent of firmware; pure GCS; unblocks a cleaner menu/action layer
  and supersedes the scattered `QShortcut`s). Help dialog rides along.
- **Phase 2:** whole-GCS replay — it is the `2c` slot, upgraded. Do it after the
  recording/`.bin` format is settled, since replay consumes what recording
  writes.
- **Enhancements** (calibration wizard gating, sensor-noise injection
  FR-SIM-04, world-config additions, stream-rate config) fold into their
  owning widgets as normal Phase-1/2 line items.

See [feature-matrix.md](feature-matrix.md) for the element-by-element verdict map.
