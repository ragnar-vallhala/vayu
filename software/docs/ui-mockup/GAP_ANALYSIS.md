# Vayu GCS — UI Mockup vs. Qt Implementation Gap Analysis

Comparison of `docs/ui-mockup/index.html` against the existing Qt/QWidget app in
`software/src`. Read-only audit; no code changed. References are `file:line` into
`software/src` and the mockup.

> **Framing:** this is **not** a one-way "make Qt match the mockup" list. The mockup is a
> design prototype with synthetic data. In two areas (Packet Analyzer, Autotune backend)
> the **Qt app is already more capable than the mockup**, and the mockup is a simplified or
> aspirational view. Treat divergences case-by-case: sometimes the mockup is the target,
> sometimes the existing Qt behavior is correct and the mockup is just under-drawn.

---

## At-a-glance

| Page | State | Headline gaps |
|------|-------|---------------|
| App shell | Mostly matches | Toolbar uses "Link/transport" not plain Port; extra RC/CALIB buttons |
| Flight Dashboard | Matches w/ gaps | **Rolling-σ traces**, **vehicle-state color band**, **vertical temp+battery gauges** all missing on graphs |
| RC Channels | Significant gaps | **Stick gimbals**, **AUX 2/3-way switches**, **channel-history graph**, embedded link-health all missing |
| Motors | Moderate gaps | **Spinning-prop overlays**, floating %/dir labels, **Power readout group** (V/A/ESC-temp) missing |
| Control Loop | Strong match | Only the header **Pause** button is missing |
| Calibration | Moderate gaps | **Status panel**, **Level-Horizon** action, instruction text missing |
| Settings | Minor gaps | **Theme selector** + persistence missing; title says "Settings" not "Configuration" |
| Packet Analyzer | **Qt ahead** | Qt has Wireshark-style filters, streaming, dev-filter, tree dissection. Cosmetic: segmented Dir control, CRC/Payload columns |
| Simulator HUD | Strong match | **Draggable/resizable PiPs** (down-cam + horizon) missing |
| Sim Vehicle/World | Moderate gaps | **Sensor-models table**, **fault-injection**, **motor τ** missing from config + VsimTypes |
| Autotune | **Qt ahead, incomplete UI** | Backend exists (autotune.py + TuneChart). Missing: **chirp/step conditional UI**, live response plot |

---

## Shell + Flight Dashboard

**Palette verified:** every token the mockup claims to mirror is real in `core/Theme.h:18-39`
(surfaces, borders, text, semantics, axis roll/pitch/yaw). Mockup palette is faithful.

**Matches:** window chrome + menu bar (`MainWindow.cpp:26-174`), toolbar with port/baud/connect/
ARM/LIVE (`MainToolbar.cpp:31-141`), status-bar segments (`MainStatusBar`), 2D ADI with pitch
ladder + roll arc + heading tape (`AttitudeWidget.cpp:61-210`), 3D airframe + toggle
(`Drone3DWidget.cpp`, `MainWindow.cpp:359-364`), multi-series IMU graphs (`ImuPanel.cpp:20-180`,
`RealTimeGraph.cpp`), state pill, log panel, splitter layout.

**Diverges:**
- Toolbar: mockup labels are plain *Port / Baud*; Qt uses *Link: [Serial/UDP]* transport selector
  and adds *RC* and *CALIB* shortcut buttons (`MainToolbar.cpp:116-129`).
- Heading: mockup is a horizontal scrolling **tape**; Qt draws a circular compass overlay in the ADI
  (`AttitudeWidget.cpp:155-210`).
- Temperature: mockup is a **vertical gauge**; Qt renders temp as another time-series graph
  (`ImuPanel.cpp:51,144`).

**Missing (mockup → no Qt equivalent):**
1. **Rolling-σ dotted traces** with a right-hand 0..stdMax axis on each IMU graph. Qt computes σ via
   `RollingStats` but shows it only as a text label, not a plotted trace. Needs `RealTimeGraph`
   extension for a secondary series + dual axis (`RealTimeGraph.h`).
2. **Vehicle-state color band** behind traces (blue init / green standby / red armed / orange
   failsafe). No background-fill API in `RealTimeGraph`; add `setState()` + `setStateColors()`.
3. **Vertical battery gauge** (0–100%, red→green gradient). No battery display on the home page.
4. **Vertical device-temp gauge** (distinct from the time-series graph above).
5. Log panel **Pause** + **Export** buttons (Qt has only Auto-scroll + Clear, `LogPanel.cpp:20-34`).

---

## RC Channels + Motors

**Matches:** RC channel rows + PWM values (`RcChannelsWidget.cpp:35-87`); motor cards with %/dir +
per-motor σ (`MotorStatusWidget.cpp:154-212`); motor-speed 10s graph (`:189-194`); X-frame schematic
with QPainter arc rings (`:47-94`); link metrics + throughput (`LinkStatsPanel.cpp:139-157`);
`RealTimeGraph` multi-series + per-series pen styles.

**Diverges:**
- Motor schematic is QPainter (arc rings) but lacks the floating per-motor `M# · DIR` label + the
  large center `%` overlay the mockup floats above each ring (`MotorStatusWidget.cpp:72-94`).
- Link-health lives in a standalone `LinkStatsPanel` (Packet-Analyzer utility), **not embedded** in
  the RC page as the mockup shows.

**Missing:**
1. **Stick gimbals** — throttle/yaw + pitch/roll crosshair boxes with a live dot. No gimbal drawing
   in `RcChannelsWidget`.
2. **AUX 2-way / 3-way segmented switch controls** (ANGLE/HORIZON/ACRO, SAFE/ARM…). Qt draws every
   channel as a linear bar; no segmented control.
3. **Channel-history graph** (8-trace, 10s, **dotted** for aux channels 4–8). Not integrated; also
   `RealTimeGraph` has no "dotted from index N" preset.
4. **Spinning-prop overlays** on the motor schematic (animated blades). QPainter schematic has none.
5. **Power readout group** on the Motors page — Avg Throttle, Max Spread, **Battery V**, **Current A**,
   **ESC Temp**, **mAh Used**. No power telemetry on the motor page at all.

---

## Control Loop + Calibration + Settings

**Control Loop — strong match.** 2×2 grid (`ControlLoopPlot.cpp:87-89,164-253`), dashed-setpoint vs
solid-actual (`:176-206`), per-trace colored stat readouts (`:118-133,322-359`), dt ±σ
(`:135-162,291-312`), gyro-scale input (`:38-47,376-389`), Export CSV (`:56-74`).
- **Missing:** header **Pause** button (mockup line 801).

**Calibration — moderate gaps.**
- **Missing Status panel** — per-sensor ✓OK / Needs-cal / Not-run fields (mockup 821-827). Qt jumps
  straight to sensor buttons.
- **Missing Level-Horizon** action — Qt has Accel/Gyro/Mag only (`CalibrationWidget.cpp:24-42`).
- **Missing instruction text** paragraph (mockup 836).

**Settings — minor gaps.**
- Telemetry (time-sync, graph-window, dropout) and Connection (auto-reconnect) all map to real
  `SettingsManager` keys (`SettingsWidget.cpp:25-83`, `SettingsManager.h:11`). ✅
- **Missing Theme selector** (Dark/Midnight/High-Contrast) — no widget and **no `theme` field** in
  `GcsSettings`.
- Title says "Settings"; mockup says "Configuration" (`SettingsWidget.cpp:16`).

---

## Packet Analyzer  — **Qt is ahead of the mockup**

**Matches:** frequency ribbon (`FrequencyRibbon.cpp:65-135`), direction filter, per-type toggle
chips (`PacketAnalyzerWidget.cpp:61-93`), text/hex search (`PacketFilterProxy.cpp:72-78`), row-click
decoding (`PacketDetailWidget.cpp:69-86`), color-coded raw hex (`HexView.cpp:47-92`), CRC display.

**Diverges (cosmetic, mockup is simpler):**
- Dir control: mockup = segmented buttons; Qt = QComboBox (`:87`).
- Table columns: mockup `Time|Dir|Type|Dev|Len|CRC|Payload`; Qt `No|Time|Dir|Type|Dev|Len|Info`
  (CRC + payload hex moved to the detail pane) (`PacketLogModel.h:36`).
- Toolbar: mockup *Pause|Clear|Export CSV*; Qt *Back|Auto-scroll|Clear|Start Streaming|Save Table*.

**Qt ahead of mockup (do NOT regress these):**
- **Wireshark-style display-filter language** — `==,!=,<,<=,>,>=,contains, &&,||,!()` over
  `type/dir/dev/size/origin/info` (`PacketFilterExpr.*`). Mockup only has plain search.
- **Device-ID filter field**, **live CSV streaming mode**, **tree-based dissection with
  field→byte highlight**, and **real protocol types** (11 incl. COMMAND, IMU_COMPRESSED, PERF_*),
  vs the mockup's 7 invented types.

> Recommendation: keep Qt's superset; only adopt mockup cosmetics worth having (segmented Dir
> control, maybe a CRC column).

---

## Simulator + Autotune

**HUD — strong match.** `SimHudWidget.cpp` already paints horizon + pitch ladder (`:69-99`),
heading tape (`:137-160`), speed/alt tapes (`:211-215`), motor bars (`:225-235`), state text
(`:238-299`), VS readout (`:217-221`). Vehicle (`GeometryEditorWidget`) and World
(`WorldEditorWidget`) config editors cover load/save, mesh load, mass/inertia, motor positions +
coefficients, environment, terrain. RC bridge fully present (`RcBridge.*`, `SimulatorWidget.h:155-176`).

**Missing:**
1. **Draggable/resizable PiP overlays** — Down-Cam + Horizon windows over the FPV canvas. Qt's
   `HorizonHud` is a fixed-corner widget, not a movable/resizable PiP, and there is **no down-cam
   view** at all. Would live in `SimulatorWidget` (child QWidgets + mouse/resize handling).
2. **Sensor-models table** (per accel/gyro/mag/baro/gps: rate / noise σ / bias / enable). No fields
   in `VsimTypes.h`, no UI, no `.vveh` serialization.
3. **Fault-injection** (kill motor M1–M4, sensor dropout, RC-loss, GPS glitch). No UI or backend.
4. **Motor time-constant τ** (spin-up lag). Not in `MotorConfig` (`VsimTypes.h:40-50`) or the editor.
5. On-canvas **accel/gyro mini-plots** — buffers exist (`SimHudWidget` `accHist_/gyrHist_`) but are
   not painted in `paintEvent`; the plots live on a separate telemetry page instead.
6. World **BVH status badge** — BVH is built at runtime but no ✓/✗ indicator in `WorldEditorWidget`.

**Autotune verdict:** **Exists in Qt and is actually more built-out than the mockup's backend** —
`SimulatorWidget.cpp:735-1000+` has method selector, budget, step amplitude/repeats, rig tether,
yaw toggle, validation/compare/buzz checkboxes, Start/Stop, log view, `TuneChart` convergence chart,
proposed-gains table, and external `tools/autotune/autotune.py` invocation. **But the UI is
incomplete vs. the mockup's polish:** no **step-vs-chirp conditional settings panel** with
show/hide, and **no live step-response / chirp-sweep plot** (mockup 1001-1019). The mockup's gains
table is read-only output in both.

---

## Suggested prioritized backlog (for when we move past analysis)

**Tier 1 — shared infra that unlocks several pages**
- Extend `RealTimeGraph`: (a) secondary rolling-σ series + dual axis, (b) vehicle-state background
  band, (c) "dotted from series N" preset. Unblocks Dashboard IMU graphs **and** RC channel history.

**Tier 2 — high-visibility dashboard polish**
- Vertical battery + device-temp gauges (Dashboard).
- RC stick gimbals + AUX segmented switches + channel-history graph.
- Motors Power group (V/A/ESC-temp/mAh) + spinning-prop overlays + floating labels.

**Tier 3 — sim depth**
- Sensor-models table + motor τ (needs `VsimTypes.h` struct changes + `.vveh` schema bump).
- Fault-injection panel (needs SimWorker/daemon signal path).
- Draggable/resizable PiPs (down-cam + horizon).
- Autotune chirp/step conditional UI + live response plot.

**Tier 4 — small/cosmetic**
- Calibration Status panel + Level-Horizon + instructions.
- Settings theme selector (+ `GcsSettings.theme`).
- Control-Loop Pause button; Log Pause/Export buttons.
- Packet Analyzer: segmented Dir control, optional CRC column (keep Qt's superset filters).
</content>
</invoke>
