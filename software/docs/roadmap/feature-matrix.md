# Feature matrix — mockup verdict → disposition

Every feature group from the mockup's `Ctrl+D` analysis mode, mapped to a
**tier** (🔴 structural / 🟡 additive / 🟢 easy) and a **disposition**:

- **shipped** — already in the app; mockup is catch-up spec.
- **enhance** — widget exists; mockup adds behaviour → line item on that widget.
- **new** — no home yet → has a roadmap doc.

Effort: S (days) · M (1–2 wk) · L (3–4 wk) · XL (5+ wk).

| Feature (mockup) | Tier | Disposition | Real class / target | Effort |
|---|---|---|---|---|
| **Whole-GCS log replay + crop/loop** | 🔴 | **new** | `src/replay/`, `ITelemetrySource`, `ReplayBar` — see [gcs-log-replay.md](gcs-log-replay.md) | XL |
| **Editable shortcuts + command registry** | 🔴/🟡 | **new** | `core/CommandRegistry`, `ShortcutsManager`, `ShortcutsEditorDialog` — see [command-registry-and-shortcuts.md](command-registry-and-shortcuts.md) | L |
| **Command palette** | 🟡 | **new** | `CommandPalette` (same doc) | M |
| **Recent-views (`Ctrl+`` `) switcher** | 🟡 | **new** | `ViewHistory` + `RecentViewsOverlay` (same doc) | M |
| **Documentation + About** | 🟢 | **new** | `AboutDialog` + `QDesktopServices` (FR-UI-20) | S |
| SITL FPV / camera render | 🔴 | enhance | `SimRendererWidget` (render infra exists; add cam views) | L |
| Sensor fault / noise injection | 🔴 | enhance | `vsim_d` sensor model + `vsim_ctl` opcode; `WorldEditorWidget` (FR-SIM-04) | L |
| RC bridge into SITL | 🔴 | enhance | new `vsim_ctl` input path + `SimulatorWidget` | M |
| In-GCS autotune (current/best) | 🔴 | new-ish | C++ optimizer over `SimWorker` eval loop; `TuneChart` exists for plots | XL |
| 3D attitude / airframe | 🔴 | shipped | `Drone3DWidget` | — |
| World config (wind/mag/terrain/spawn) | 🟡 | enhance | `WorldEditorWidget` (+ FR-SIM-08 wind) | M |
| Per-motor coeffs / thrust axis / MoI | 🟡 | shipped | `GeometryEditorWidget` (FR-SIM-11) | — |
| Kernel-perf observability | 🟡 | shipped/enhance | `PerfWidget` (needs RTOS perf packet for full fidelity) | S–M |
| Calibration wizard (gated, fig-8) | 🟡 | enhance | `CalibrationWidget` — add step state machine + animation | M |
| Packet filter expression | 🟡 | shipped | `PacketFilterExpr` + `PacketFilterProxy` | — |
| Per-type frequency ribbon | 🟡 | shipped | `FrequencyRibbon` | — |
| Attitude indicator / PFD | 🟡 | shipped | `AttitudeWidget`, `HorizonHud` | — |
| RC stick visualiser | 🟡 | shipped | `RcChannelsWidget` | — |
| Telemetry stream config (rate) | 🟡 | enhance | `SettingsManager` + `DroneProtocol` SET_STREAM_RATE (firmware-paired) | M |
| Autotune live plots | 🟡 | shipped | `TuneChart` | — |
| Packet analyzer / decoder | 🟢 | shipped | `PacketAnalyzerWidget`, `PacketDetailWidget`, `PacketLogModel` | — |
| Link / network stats | 🟢 | shipped | `LinkStatsPanel` | — |
| Telemetry readouts / sparklines | 🟢 | shipped | `ImuPanel`, `RealTimeGraph` | — |
| Control-loop telemetry | 🟢 | shipped | `ControlLoopPlot` | — |
| Settings forms (units/theme/etc.) | 🟢 | shipped | `SettingsManager` + settings widgets | — |
| View nav / 2D-3D toggle | 🟢 | shipped | `QStackedWidget`, `MainToolbar` | — |
| Link connect (transport/port/baud) | 🟢 | shipped | `SerialManager`/`UdpManager`/`PortArbiter` | — |
| ARM / Disarm | 🟢 | enhance | `MainToolbar` (blocked on firmware FR-TX-02 / `0a`) | S |
| Status bar | 🟢 | shipped | `MainStatusBar` | — |
| **Analysis mode (`Ctrl+D`)** | — | **mockup-only** | not shipped — design instrument only | — |

## Reading the table

- Most rows are **shipped** — the mockup is, in large part, documentation of what
  already exists, plus a handful of enhancements.
- The **🔴 new** rows are the real architectural work and each has a dedicated
  roadmap doc.
- **enhance** rows are normal Phase-1/2 line items against an existing widget;
  they don't need their own doc — note them on the widget's requirement.

Keep this matrix in step with the mockup: if a `Ctrl+D` tag changes tier or a new
screen appears, add/adjust a row here.
