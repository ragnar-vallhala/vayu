# Navigator — Requirements

Status: living document. Tracks what the Navigator GCS must do, what it
already does, and what's next. Engineering spec, not a marketing
brochure — every requirement here exists because the code either has
to satisfy it or doesn't yet and should.

Five sections:

1. **System requirements** — what's needed to build and run.
2. **Functional requirements** — what Navigator does, by subsystem,
   with current status.
3. **Phase plan** — what gets built in what order.
4. **Architecture map** — where to find each component.
5. **Maintenance rules** — how this doc stays honest.

Cross-references:
- [`journal/changelog/gcs-in-app-simulator-and-world-collision.md`](../journal/changelog/gcs-in-app-simulator-and-world-collision.md) — the embedded SITL design (`vsim_d` daemon).
- [`navlink/docs/reference/messages/`](../../../navlink/docs/reference/messages/) — wire format authority.
- [`firmware/docs/reference/coordinate_ref.md`](../../../firmware/docs/reference/coordinate_ref.md) — NED conventions.
- [`journal/shipped/`](../journal/shipped/) — design records of shipped GCS features.

---

## 1. System requirements

### 1.1 Toolchain

| Item              | Version    | Hard? | Notes                                                           |
|-------------------|------------|-------|-----------------------------------------------------------------|
| C++ standard      | C++17      | hard  | `CMAKE_CXX_STANDARD_REQUIRED ON`                                |
| CMake             | ≥ 3.20     | hard  | `cmake_minimum_required` in `navigator/CMakeLists.txt:1`         |
| Qt                | 6.x        | hard  | `Core Widgets SerialPort OpenGL OpenGLWidgets`                  |
| Compiler          | gcc/clang  | hard  | Anything that ships C++17 + Qt6 binding works                   |
| OpenGL            | 3.3 core   | hard  | Requested in `src/app/main.cpp:7`; MSAA x4                      |
| pthreads          | yes        | hard  | `vayu_sitl_core` spawns task threads inside Navigator           |
| Python            | none       | n/a   | Killed off with the Gazebo bridge                               |

### 1.2 Platform support

| Platform | Tier  | Notes                                                                       |
|----------|-------|-----------------------------------------------------------------------------|
| Linux    | tier 1| Primary dev target. Real serial via `/dev/ttyUSB*`, SITL via pty.           |
| macOS    | tier 2| Bundle metadata in `navigator/CMakeLists.txt:138`; SITL untested.            |
| Windows  | tier 3| `WIN32_EXECUTABLE TRUE` set, but POSIX FIFO / pty paths in vsim are Linux.  |

### 1.3 Build outputs

- `navigator/build/Navigator` — single executable.
- Statically linked against `vayu_sitl_core` (firmware + host shims from
  `sim/host/`). The `NAVIGATOR_HAS_SITL=1` compile define gates
  SITL features; **when `sim_host` is absent the SITL sources are not
  compiled in**, the Simulator menu entry disappears, and the executable
  links cleanly without `vayu_sitl_core` (CMake conditionally adds
  `GCS_SITL_SOURCES` only if `sim/host/CMakeLists.txt` exists).

### 1.4 Runtime resources

- Serial device read permission (`dialout` group on Linux) or
  `/tmp/vayu_uart2_pty` for the SITL pty path.
- Write access to two log roots:
  - `<repo>/logs/sim-<ts>.bin` — per-run SITL UART2 byte log
    (managed by `SimulatorWidget`; path configurable in the page).
  - `QStandardPaths::AppDataLocation/logs/navigator-YYYY-MM-DD.log` —
    persistent text log of `LogPanel` lines (managed by `core/Logger`;
    rotates at 5 MB, keeps 10 files).
- `QSettings` writable location (default OS-specific) for window
  geometry, splitter state, last page, port + baud, calibration prefs.
- Qt6 platform plugins on `LD_LIBRARY_PATH` / discovered via standard
  Qt deployment.

### 1.5 Non-functional requirements

| ID       | Requirement                                                                                  |
|----------|----------------------------------------------------------------------------------------------|
| NFR-01   | UI thread never blocked > 16 ms by sim/serial work. Heavy work runs in `SimWorker`/timers.   |
| NFR-02   | SITL physics steps at 1 kHz, IMU emit 200 Hz, snapshot 60 Hz (paced in `vsim_d`, `sim/vsim/src/main.cpp`). |
| NFR-03   | No data copies between firmware and renderer beyond the queued `SimSnapshot` (trivially copyable). |
| NFR-04   | `DroneProtocol` parser handles partial reads and drops at most 1 byte per CRC failure.       |
| NFR-05   | Heartbeat round-trip drift surfaced to the user, colour-banded at ±20 ms / ±100 ms.          |
| NFR-06   | All firmware-thread → GUI-thread transitions use `Qt::QueuedConnection`. No shared QObjects. |
| NFR-07   | `core/Logger` is thread-safe (mutex-guarded `QFile`); SITL firmware threads can call it without crossing onto the GUI thread first. |
| NFR-08   | `SerialManager` auto-reconnect uses exponential backoff (1→2→4→8 s, cap 5 attempts) and is suppressed on user-initiated disconnect. |
| NFR-09   | The dark theme is the single source of styling truth: zero inline `QPushButton { background: … }` literals in widget code; per-widget tokens reference `core/Theme.h`. |

---

## 2. Functional requirements

Status markers: ✅ implemented · 🟡 partial · ❌ missing · 🔥 known-broken.

### 2.1 Wire protocol (rx)

Parser lives in `src/protocol/DroneProtocol.cpp` + `PacketDecoder.cpp`.
Frame: `sync(0x56) | type<<4|ver | len | dev_id | ts[4] | payload[len] | CRC32[4]`.
CRC32: poly `0x04C11DB7`, init `0xFFFFFFFF`, MSB-first byte-wise.

| ID       | Packet               | Type | Status | Notes                                                |
|----------|----------------------|------|--------|------------------------------------------------------|
| FR-RX-01 | Heartbeat            | 0x0  | ✅     | Emits `heartbeatReceived(ts, dev_id)`                |
| FR-RX-02 | IMU full (40 B)      | 0x1  | ✅     | 10 floats (acc/gyr/mag/temp)                         |
| FR-RX-03 | IMU delta (20 B)     | 0x2  | ✅     | 10× float16 deltas, base from last full              |
| FR-RX-04 | Attitude             | 0x4  | ✅     | 3 floats (roll/pitch/yaw deg)                        |
| FR-RX-05 | RC channels          | 0x5  | ✅     | 14× u16 µs                                           |
| FR-RX-06 | System state         | 0x6/0x04 | ✅ | 1 float, mapped to named state strings              |
| FR-RX-07 | Control loop data    | 0x6/0x05 | ✅ | 18 floats (setpoints, currents, outputs, dts)       |
| FR-RX-08 | Calibration update   | 0x6/0x01 | ✅ | Accel 12-pose (faces+edges) + gyro + mag; pose / axis-coverage progress |
| FR-RX-09 | Log text             | 0x7  | ✅     | Free-form ASCII                                      |
| FR-RX-10 | Motor telemetry      | 0x8  | ✅     | 4 floats                                             |
| FR-RX-11 | Unknown packet       | —    | ✅     | Forwarded raw to listener (Packet Analyzer + log)    |
| FR-RX-12 | Resync on CRC fail   | —    | ✅     | Drop 1 byte, keep scanning                           |
| FR-RX-13 | Protocol version gate| —    | ✅     | Anything ≠ v1 dropped 1 byte at a time               |

### 2.2 Wire protocol (tx)

GCS → drone is the weak side.

| ID       | Command                       | Status | Notes                                                        |
|----------|-------------------------------|--------|--------------------------------------------------------------|
| FR-TX-01 | Heartbeat (5 s cadence)       | ✅     | `MainWindow::onTimeSyncRequested`                            |
| FR-TX-02 | ARM / DISARM                  | ⛔ blocked | Awaits firmware packet handler; button intentionally disabled with explanatory tooltip until then. |
| FR-TX-03 | Start calibration             | 🟡     | Calibration widget emits `commandRequested` raw bytes; gated when disconnected |
| FR-TX-04 | Cancel calibration            | 🟡     | Same path as FR-TX-03                                        |
| FR-TX-05 | Parameter get / set           | ❌     | No param tree on either side yet                             |
| FR-TX-06 | Mission upload / clear        | ❌     | Out of scope until autonomy work begins                      |
| FR-TX-07 | Mode change (manual/althold/…)| ❌     | Depends on firmware mode plumbing                            |
| FR-TX-08 | RC override / failsafe trigger| ❌     | Defer; sim_bridge MCU owns RC                                |

### 2.3 Telemetry panels (home screen)

| ID        | Panel                  | Status | Source signal                            |
|-----------|------------------------|--------|------------------------------------------|
| FR-UI-01  | Attitude (2D HUD)      | ✅     | `attitudeReceived`                       |
| FR-UI-02  | Attitude (3D)          | ✅     | Same; toggled via `m_attStack`           |
| FR-UI-03  | Numeric R/P/Y + σ      | ✅     | `RollingStats` over rx                   |
| FR-UI-04  | IMU graphs (acc/gyr/mag)| ✅    | `imuReceived` → `ImuPanel`; CSV export via `core/CsvExport` |
| FR-UI-05  | Log scrollback         | ✅     | `logReceived` + unknown packets; tees to persistent file via `core/Logger` |
| FR-UI-06  | Live-blink + sync drift| ✅     | Heartbeat colour band ±20/±100 ms        |
| FR-UI-07  | System state pill      | ✅     | Colour-graded ARMED/FAILSAFE/STANDBY…    |

### 2.4 Auxiliary pages (`QStackedWidget`)

| ID        | Page                | Status | Notes                                                     |
|-----------|---------------------|--------|-----------------------------------------------------------|
| FR-UI-10  | Packet Analyzer     | ✅     | Both rx and tx logged with timestamp + decode             |
| FR-UI-11  | RC Channels Monitor | ✅     | 14-channel bar display                                    |
| FR-UI-12  | Calibration         | 🟡     | Per-sensor flow works; UX polish pending                  |
| FR-UI-13  | Motor Status        | ✅     | 4-motor speed bars; CSV export                            |
| FR-UI-14  | Control Loop Plot   | ✅     | Inner/outer PID setpoint vs current vs output; CSV export |
| FR-UI-15  | Simulator           | ✅     | In-app SITL panel; see §2.6                               |
| FR-UI-16  | Settings            | 🟡     | Sync period + graph window + dropout. Missing: theme, units, port profile |
| FR-UI-17  | Parameter editor    | ❌     | Blocks on FR-TX-05                                        |
| FR-UI-18  | Mission planner / map| ❌    | Blocks on FR-TX-06                                        |
| FR-UI-19  | Log replay          | ✅     | Session-wide **read-only** replay UI: a crop/loop scrubber (`ReplayBar`) gated by the `SourceController` FSM (the read-only / source authority). Open via File ▸ Open Log. See FR-LOG-05 and [`gcs-log-replay.md`](../journal/shipped/gcs-log-replay.md). |

### 2.5 Connection layer

| ID        | Requirement                                                            | Status |
|-----------|------------------------------------------------------------------------|--------|
| FR-CONN-01| Auto-list `/dev/ttyUSB*` / `/dev/ttyACM*`                              | ✅     |
| FR-CONN-02| Editable port combo (custom `/dev/pts/N` paths)                        | ✅     |
| FR-CONN-03| Auto-offer SITL pty from `/tmp/vayu_uart2_pty`                         | ✅     |
| FR-CONN-04| Baud presets 9600 – 921600, default 115200                             | ✅     |
| FR-CONN-05| Reconnect on hot-unplug / error surface                                | ✅     | Opt-in via Settings → "Auto-reconnect on serial error"; exponential backoff, 5 attempts |
| FR-CONN-06| Disconnect cleans state, disarms UI, stops sync timer                  | ✅     |
| FR-CONN-07| Last port + baud persisted across restarts                             | ✅     | QSettings round-trip in `restoreUiState`/`persistPortBaud` |

### 2.6 Embedded SITL (`vsim/` + `SimulatorWidget` + `vsim_d`)

Design record: `docs/changelog/gcs-in-app-simulator-and-world-collision.md`. Wire protocol: `sim/vsim/include/vsim_proto.h`.
Navigator-side code: `src/vsim/`, `src/ui/widgets/SimulatorWidget.{h,cpp}`.
Physics daemon: `sim/vsim/` (builds the standalone `vsim_d` binary).

Two-process architecture (since the vsim_d split). The firmware threads
still run *in-process* inside Navigator via `vayu_sitl_start`, but the
physics + sensor simulation moved out to the standalone `vsim_d`
process. The two talk over four latest-wins FIFOs (see `vsim_proto.h`):
firmware → `/tmp/vsim_pwm` → vsim_d, vsim_d → `/tmp/vsim_imu` → firmware,
vsim_d → `/tmp/vsim_pose` → Navigator renderer, Navigator →
`/tmp/vsim_ctl` → vsim_d (reset/control). `SimWorker` no longer runs
physics; it spawns/supervises `vsim_d` and decodes pose frames.

| ID       | Requirement                                                              | Status |
|----------|--------------------------------------------------------------------------|--------|
| FR-SIM-01| Run firmware in-process via `vayu_sitl_start(&iface)`                    | ✅     | Physics now out-of-process in `vsim_d`; firmware threads still in Navigator |
| FR-SIM-02| 1 kHz physics (RK4), 200 Hz IMU emit, 60 Hz pose snapshot                | ✅     | Driven by `vsim_d` (`sim/vsim/src/main.cpp`); SimWorker only reads pose |
| FR-SIM-03| Drone parameters (mass, inertia, motor geometry) editable from UI        | ✅     | `GeometryEditorWidget`: mass + 4-motor layout editable; inertia tensor derived from the airframe mesh; pushed live via `VSIM_CTL_SET_GEOMETRY`. See FR-SIM-11. |
| FR-SIM-04| Sensor noise / bias parameters editable from UI                          | ✅     | Vehicle ▸ Sensor Models panel (`SimulatorWidget`): per-sensor white-noise σ, bias clip, and enable toggle, pushed via `SimWorker::sendNoise()` / `VSIM_CTL_SET_NOISE` |
| FR-SIM-05| Pose snapshot rendered live (OpenGL)                                     | ✅     |
| FR-SIM-06| Per-run raw UART byte log to `logs/sim-<ts>.bin`                         | ✅     |
| FR-SIM-07| Hot reset of physics / firmware state between runs                       | 🟡🔥  | Physics resets via `VSIM_CTL_RESET` (`SimWorker::sendReset`); firmware state still can't (`vayu_sitl_start` one-shot — Navigator restart needed) |
| FR-SIM-08| Wind / external-force injection                                          | ✅     | World-frame wind field (steady + gust + turbulence) felt as relative-velocity drag. `wind_model.h`, pushed via `VSIM_CTL_SET_WIND` (`SimWorker::sendWind`), edited from the World tab (`WorldEditorWidget::windApplied`). Design: [`01-wind-turbulence.md`](../../../sim/vsim/docs/plans/sim-fidelity/01-wind-turbulence.md). |
| FR-SIM-09| Ground-contact model (tipping, friction)                                 | ❌     | Intentional: hard clamp only (`sim/vsim/src/physics_core.cpp`) |
| FR-SIM-10| FIFO transport to the standalone firmware binary                         | ✅     | `vsim_d` FIFOs are the only transport; the standalone `vayu_sitl` binary attaches to the same `/tmp/vsim_{pwm,imu}` paths |
| FR-SIM-11| Mesh-derived mass properties + motor-mapping editor                      | ✅     | Import STL/glTF (assimp) → full 3×3 inertia tensor via `MassProperties` (closed-polyhedron integral); 4-motor position/axis/spin/coeff editor (`GeometryEditorWidget`) with interactive Blender-style 3D gizmos (click-select, G/R + X/Y/Z, sim-stopped only); pushed to `vsim_d` over `VSIM_CTL_SET_GEOMETRY`. Daemon integrates the full tensor (`Mat3` in `vsim_math.h`). All dynamics are about the CoM — the mesh is recentered and motor arms made CoM-relative GCS-side (`physicsConfig()`), so the model origin need not coincide with the CoM. Design: [`sim-geometry-moi-motor-editor.md`](../journal/shipped/sim-geometry-moi-motor-editor.md). |
| FR-SIM-12| World/environment editable from UI                                       | ✅     | World tab: gravity (runtime), ground height, restitution, linear/angular drag. Pushed via `VSIM_CTL_SET_WORLD`; composes with geometry (disjoint `DroneParams` fields). `WorldEditorWidget`. Wind: see FR-SIM-08 (✅). |
| FR-SIM-13| Blender-style simulator UI                                               | ✅     | `SimulatorWidget` is a Vehicle\|World mode bar + 3D viewport + categorized (collapsible, `CollapsibleSection`) properties panel. Vehicle = airframe config (locked while running); World = environment design + run controls. FPV telemetry HUD (`SimHudWidget`) overlays the viewport while running. |
| FR-SIM-14| RC transmitter input (USB joystick)                                      | ✅     | `RcBridge` reads a USB RC TX (Linux `js0`, e.g. Artery PPM / FlySky-via-PPM dongle), maps axes→channels (roll/pitch/throttle/yaw/arm) and streams CSV µs frames over a pty that `$VAYU_UART_RC_PATH` points at, so the firmware RC feeder flies from the sticks. Toggle in World→Simulation; enable before first Start. |

### 2.7 Persistence (`SettingsManager`)

| ID        | Requirement                                       | Status |
|-----------|---------------------------------------------------|--------|
| FR-CFG-01 | Persist sync period, graph window, dropout rate   | ✅     |
| FR-CFG-02 | Persist last port + baud                          | ✅     |
| FR-CFG-03 | Persist log dir + repo root for simulator         | ✅     |
| FR-CFG-04 | Per-airframe profiles (params, motor layout)      | ❌     |
| FR-CFG-05 | Export / import settings (json)                   | ❌     |
| FR-CFG-06 | Persist window geometry, splitter state, last page| ✅     |
| FR-CFG-07 | Persist auto-reconnect preference                 | ✅     | Lives in `GcsSettings`; v2 serialisation appends the bool with `atEnd()` fallback. |

### 2.8 Logging & diagnostics

| ID        | Requirement                                                | Status |
|-----------|------------------------------------------------------------|--------|
| FR-LOG-01 | In-UI scrollback (`LogPanel`)                              | ✅     |
| FR-LOG-02 | Raw UART byte capture per SITL run                         | ✅     |
| FR-LOG-03 | Persistent text log of all rx/tx with timestamps           | ✅     | `core/Logger.{h,cpp}`; tees from `LogPanel::appendLog` to `AppDataLocation/logs/navigator-YYYY-MM-DD.log` with 5 MB / 10-file rotation. |
| FR-LOG-04 | CSV export of IMU / attitude / control-loop traces         | ✅     | `core/CsvExport`; per-widget "Export CSV" buttons on ImuPanel, ControlLoopPlot, MotorStatusWidget. Shared timestamp axis across all series. |
| FR-LOG-05 | Replay a logged `.bin` through `DroneProtocol`             | ✅     | Widened to **whole-GCS** replay: every panel fed from the log via a `RecordSink` (record tee, gated by the `recordOnConnect` setting) + `ITelemetrySource` seam + `ReplaySource` (seekable playback clock). Pairs with FR-UI-19. Design: [`gcs-log-replay.md`](../journal/shipped/gcs-log-replay.md). |

### 2.9 UX / styling

The Navigator is functional but rough. Styling is duplicated inline at
every site, half the buttons lie about their state, errors only reach
the log, and window state evaporates on restart. The items below close
those gaps. None of this is new functionality — it's the existing
functionality made usable.

| ID        | Requirement                                                                 | Status | Notes |
|-----------|-----------------------------------------------------------------------------|--------|-------|
| FR-UX-01  | Single source of truth for theme / colors / spacing                          | ✅     | `core/Theme.{h,cpp}` + `resources/styles/dark.qss`; applied once in `main.cpp` via `Theme::apply()`. |
| FR-UX-02  | No copy-pasted `QPushButton { background: … }` literals at call sites        | ✅     | Primary/Success/Danger/Ghost/Back/Toggle/FilterPill/SensorCard subclasses + object-name selectors in dark.qss; `grep "QPushButton { background"` returns empty across `navigator/src/`. |
| FR-UX-03  | ARM button reflects real state, not local toggle                             | 🟡     | Local-toggle removed; button disabled with explanatory tooltip until FR-TX-02 ships. Will become ✅ once 0a lands. |
| FR-UX-04  | Confirmation step before sending ARM                                         | ❌     | Modal or "hold to arm" affordance. Required before FR-TX-02 ships. |
| FR-UX-05  | One canonical place to show connection state                                 | ✅     | Status-bar pill is canonical. The attitude-panel pill is now a firmware-state indicator only (idle "—" when no state packet has arrived). |
| FR-UX-06  | Toast / inline notification for actionable events                            | 🟡     | `core/Notify.{h,cpp}` provides `Notify::ok/info/warn/error`; call-sites pending adoption. |
| FR-UX-07  | Tooltips on every interactive toolbar / page control                         | ✅     | Toolbar + all per-page primary controls (back/launch/stop/start/cancel/sim/calib/refresh/filter pills) carry tooltips. |
| FR-UX-08  | Keyboard shortcuts for navigation + critical actions                         | ✅     | `Ctrl+1..8` page jump, `Ctrl+K` connect, `Ctrl+L` clear log, `F11` fullscreen (in addition to `Ctrl+H`/`Ctrl+P`). |
| FR-UX-09  | Persist window geometry + splitter positions                                 | ✅     | `closeEvent` saves `saveGeometry` + splitter `saveState`; ctor restores. |
| FR-UX-10  | Persist last selected page                                                   | ✅     | `m_stackedWidget->currentIndex()` round-trips via `QSettings`. |
| FR-UX-11  | Drop the `"  (SITL UART2)"` suffix hack from the port combo                  | ✅     | Port combo uses `userData` (`addItem(label, path)`); `onConnectClicked` reads `currentData()`. |
| FR-UX-12  | Errors surface visibly, not just into the log scrollback                     | 🟡     | Serial errors flash the conn-status label and are appended to log, but transient errors (CRC fail rate, dropped IMU, fifo write fail) disappear silently. Surface counters in the status bar. |
| FR-UX-13  | Icons on toolbar buttons + page tabs                                         | ❌     | `resources/icons/` exists; `resources.qrc` references it but the toolbar is text-only. Add at minimum: connect/disconnect, arm, refresh, sim, calibrate. |
| FR-UX-14  | Consistent "Back" affordance on every sub-page                               | 🟡     | Pages emit `backToHomeRequested` but each one re-implements its own button + style. One reusable `PageHeader` widget. |
| FR-UX-15  | High-DPI scaling correctness                                                 | ❌     | Fixed pixel sizes (`setFixedWidth(40)`, `70x22`, font sizes in px) won't scale on 4K / 200 % displays. Audit + replace with `em`-relative metrics or `QFontMetrics`. |
| FR-UX-16  | Replace inline hex colours with named tokens                                 | ❌     | `#61AFEF`, `#98C379`, `#E06C75`, `#ABB2BF` recur ~50× across the codebase. Define once in the theme (`color.accent`, `color.ok`, `color.warn`, `color.danger`, `color.muted`). |
| FR-UX-17  | LIVE blinker / heartbeat indicator readable at a glance                      | 🟡     | The exp-fade on `m_liveLabel` works but the "Diff: ±N ms" pill next to it is hard to scan. Consider a single combined widget with a coloured dot + signed delta. |
| FR-UX-18  | Disable controls that depend on connection while disconnected                | ✅     | CalibrationWidget gated via `setConnected()` (sensor cards + Start disabled, footer reads "NOT CONNECTED"). Motor / Control-loop pages stay viewable as display-only. |
| FR-UX-19  | Command registry + editable keyboard shortcuts                               | ✅     | Central command set (`core/CommandRegistry`, one `QAction` per command); VS Code-style searchable rebind table with conflict detection + persisted overrides (`ShortcutsManager`, `ShortcutsEditorDialog`). Generalises the FR-UX-08 `QShortcut`s. Design: [`command-registry-and-shortcuts.md`](../journal/shipped/command-registry-and-shortcuts.md). |
| FR-UX-20  | Command palette (`Ctrl+Shift+P`)                                             | ✅     | Fuzzy runner over `CommandRegistry` (`CommandPalette`). Same roadmap doc. |
| FR-UX-21  | Recent-views (MRU) switcher                                                  | ✅     | Firefox/VS Code-style hold-to-cycle over recently-viewed pages (`ViewHistory` + `RecentViewsOverlay`, `Ctrl+Tab`); depth setting `recentViewsCount`. IDE idiom, not a GCS convention — kept behind the registry. Same roadmap doc. |
| FR-UX-22  | About dialog                                                                 | ✅     | Static `AboutDialog` — name / version / build / Qt + protocol / flight-stack components. |
| FR-UX-23  | Documentation entry                                                          | ✅     | Help ▸ Documentation opens the bundled `docs/` via `QDesktopServices::openUrl` (`AboutDialog::docsPath`). |

> **Note.** Unlike FR-UX-01–18 (the Phase-0 cleanup batch — *existing*
> functionality made usable), FR-UX-19–23 are genuinely **new** affordances: a
> command layer + help surface that supersede the ad-hoc `0l` shortcuts. Their
> design lives in [`command-registry-and-shortcuts.md`](../journal/shipped/command-registry-and-shortcuts.md).

---

## 3. Phase plan

Three phases. Anything not listed below is not the current quarter's
problem.

### Phase 0 — close the obvious holes  *(now)*

Small, mechanical, removes existing TODOs and embarrassment. Two
groups: protocol/plumbing fixes (0a–0e) and a UX cleanup pass (0f–0o)
covering the items in §2.9. The UX work is mechanical too — no new
features, just making the existing surface honest, persistent, and
themeable.

**Plumbing / protocol**

| # | Item                                                              | Status | Notes                                |
|---|-------------------------------------------------------------------|--------|--------------------------------------|
| 0a| **Wire ARM / DISARM tx**. Define packet type with firmware first. | ⛔ blocked | Awaiting firmware-side packet handler. Tracked separately. |
| 0b| Move the loose `// Wait…` and `// Actually…` notes out of `PacketDecoder.cpp` and `crc.cpp`. | ✅ |  |
| 0c| Persist last port + baud.                                          | ✅     | See FR-CONN-07.                      |
| 0d| `SimulatorWidget`: hide / disable controls when `NAVIGATOR_HAS_SITL=0`. | ✅ | Vsim sources now conditionally added to the target; SimulatorWidget creation + menu entry + showSimulator() gated with `#ifdef`. |
| 0e| Auto-reconnect-on-error toggle.                                    | ✅     | Settings checkbox; `SerialManager` remembers last (port, baud), exponential backoff, max 5 attempts. Suppressed on user-initiated disconnect. |

**UX cleanup pass**

| # | Item                                                                                 | Status | Notes                                                |
|---|--------------------------------------------------------------------------------------|--------|------------------------------------------------------|
| 0f| **Extract one theme.** `core/Theme.{h,cpp}` + `resources/styles/dark.qss`. | ✅ | Applied once in `main.cpp` via `Theme::apply()`. |
| 0g| **Subclass buttons.** | ✅ | `core/ui/Buttons.h` subclasses + qss object-name selectors (`PrimaryButton`/`SuccessButton`/`DangerButton`/`GhostButton`/`BackButton`/`ToggleButton`/`FilterPill`/`SensorCard`); zero `QPushButton { background…}` literals remain. |
| 0h| **ARM button stops lying.** | ✅ | Disabled + tooltip until 0a; local-toggle stripped. Becomes fully ✅ once FR-TX-02 lands. |
| 0i| **One connection-state indicator.** | ✅ | Status-bar pill is canonical; attitude-panel pill demoted to firmware-state-only (muted "—" when idle). |
| 0j| **Toast / status-bar notification helper.** | ✅ | `core/Notify.h` provides `Notify::ok/info/warn/error`; helper lives — call-site adoption is incremental. |
| 0k| **Tooltips on every toolbar + per-page control.** | ✅ |  |
| 0l| **Keyboard shortcuts.** `Ctrl+1..8`, `Ctrl+K`, `Ctrl+L`, `F11`. | ✅ |  |
| 0m| **Persist window geometry + splitter state + last page.** | ✅ |  |
| 0n| **Fix port-combo suffix hack.** | ✅ | `addItem(label, userData)` + `currentData()`. |
| 0o| **Gate disconnect-dependent pages.** | ✅ | CalibrationWidget disables sensor cards + Start when disconnected; Motor/Control-loop kept viewable as display-only. |

Exit criteria:

1. Every Phase-0 row above is ✅ or explicitly bumped (with a one-line
   reason in the table). **Met** — 14/15 ✅, 0a blocked on firmware and
   documented inline.
2. `grep -nE 'QPushButton \{ background' navigator/src/` returns nothing
   — all button styling lives in the theme. **Met.**
3. Restarting Navigator restores the last window size, splitter
   positions, page, port, and baud. **Met.**
4. No control on screen depends on a `// TODO` comment to behave
   correctly. ARM either works or is visibly disabled. **Met** — ARM
   is disabled with a tooltip pointing at FR-TX-02.

Phase 0 closed. Next work is Phase 1.

### Phase 1 — make the GCS pleasant to use  *(next)*

The Navigator is read-rich and write-poor. Phase 1 widens the outbound
surface and breaks `MainWindow.cpp` apart before it hits 1500 LOC.

| # | Item                                                              | Touches                              |
|---|-------------------------------------------------------------------|--------------------------------------|
| 1a| Split `MainWindow.cpp` into `Toolbar`, `StatusBar`, `Dispatcher`. Pure refactor, no behaviour change. **🟡 partial** — MainToolbar + MainStatusBar extracted (272 + 136 LOC respectively); MainWindow.cpp 859 → 775. `Dispatcher` (data-slot routing) still pending; could land alongside Phase-1 1b parameter tree work. | `src/ui/main/` |
| 1b| **Parameter tree (FR-TX-05 + FR-UI-17)**. Needs paired firmware work — keep the GCS side behind a feature flag until both ends ship. | new `src/params/`                    |
| 1c| Persistent text log with rotation (FR-LOG-03). **✅ shipped.**     | `core/Logger.{h,cpp}`                 |
| 1d| CSV export of telemetry traces (FR-LOG-04). **✅ shipped.**        | `core/CsvExport.{h,cpp}`; ImuPanel/ControlLoopPlot/MotorStatusWidget |
| 1e| Sim parameter editor: mass, inertia, k_thrust, max_omega, noise (FR-SIM-03, -04). **✅ shipped** — mesh-derived mass/inertia + per-motor position/axis/spin/k_thrust/k_moment/max_omega via `GeometryEditorWidget` (FR-SIM-11); sensor-noise editing (FR-SIM-04) via the `SimulatorWidget` Sensor Models panel. | `GeometryEditorWidget`, `MeshLoader`, `MassProperties`, `SimulatorWidget` |
| 1f| CRC32 lookup table (perf nit; only if profile shows it).          | `core/crc.cpp`                       |
| 1g| **✅ Command layer** (FR-UX-19–23): command registry + editable shortcuts, palette, recent-views switcher, About/Docs. Pure GCS, no firmware; supersedes the `0l` `QShortcut`s. Design: [`command-registry-and-shortcuts.md`](../journal/shipped/command-registry-and-shortcuts.md). | `core/CommandRegistry`, `ShortcutsManager`, `ViewHistory`, `src/ui/widgets/{ShortcutsEditorDialog,CommandPalette,RecentViewsOverlay,AboutDialog}` |

Exit criteria for Phase 1:

1. **Observability**: every persistent telemetry trace can leave the
   GCS as a CSV file, and every log line is captured to disk across
   restarts. **Met** (1c + 1d).
2. **Refactor**: `MainWindow.cpp` no longer mixes toolbar, status bar,
   data dispatch, and orchestration. **🟡 partial** — toolbar + status
   bar extracted; data-slot dispatch still in MainWindow. Acceptable
   as-is unless 1b lands and adds more dispatch weight.
3. **Sim tunability**: drone + sensor parameters editable from the UI
   without recompiling. **❌ pending** (1e).
4. **Firmware-paired commands** (parameter tree) shipped end-to-end.
   **❌ blocked on firmware** (1b). Tracked but not gating Phase 1
   closure on the GCS side — when the firmware opcode lands this
   becomes a normal task, not an exit blocker.

### Phase 2 — autonomy-adjacent  *(later)*

Don't start until Phase 0/1 are in. Speculative scope until firmware
direction lands.

| # | Item                                                              | Notes                                |
|---|-------------------------------------------------------------------|--------------------------------------|
| 2a| Mode switching (FR-TX-07).                                        | Blocks on firmware mode plumbing.    |
| 2b| Sim wind / external force injection (FR-SIM-08).                  | Test surface for control hardening.  |
| 2c| **✅ Log replay** (FR-LOG-05, FR-UI-19): **whole-GCS** read-only replay — every panel fed from the log via an `ITelemetrySource` seam + `ReplaySource`, with a crop/loop scrubber and `SourceController` (FSM) read-only gating. Design: [`gcs-log-replay.md`](../journal/shipped/gcs-log-replay.md). | `src/replay/` (`RecordSink`, `ReplaySource`), `ReplayBar`, `SourceController` |
| 2d| In-process firmware hot-reset (FR-SIM-07).                        | Requires `host_lifecycle.c` rework — non-trivial. |
| 2e| Map / mission planner (FR-UI-18, FR-TX-06).                       | Defer until there's a real autonomy stack to talk to. |
| 2f| Per-airframe profiles (FR-CFG-04).                                | Falls out of 1b + 1e.                |

---

## 4. Architecture map

A new reader's "where do I find things" cheatsheet. File paths under
`navigator/src/` unless noted.

**Entry point**
- `app/main.cpp` — sets GL 3.3 core, applies theme, inits Logger,
  constructs `MainWindow`.

**Coordinator**
- `ui/main/MainWindow.{h,cpp}` — owns `SerialManager`, `DroneProtocol`,
  the `QStackedWidget` of pages, all data-slot dispatch.
- `ui/main/MainToolbar.{h,cpp}` — port/baud combos, Connect / ARM,
  page-jump buttons, LIVE blinker. Emits intent signals.
- `ui/main/MainStatusBar.{h,cpp}` — connection pill, sync drift,
  packet counter; static `flashLive`/`fadeLive` for the toolbar's
  LIVE label.

**Protocol + transport**
- `comm/SerialManager.{h,cpp}` — `QSerialPort` wrapper with
  auto-reconnect.
- `protocol/DroneProtocol.{h,cpp}` — frame parser, emits typed signals.
- `protocol/PacketDecoder.{h,cpp}` — payload decoder, returns
  `std::variant<…>`.

**Core utilities** (`core/`)
- `Theme.{h,cpp}` + `resources/styles/dark.qss` — single source of styling.
- `ui/Buttons.h` — `PrimaryButton`/`SuccessButton`/`DangerButton`/
  `GhostButton`/`BackButton` subclasses (qss object-name selectors).
- `Notify.{h,cpp}` — typed toast wrapper over `QStatusBar::showMessage`.
- `Logger.{h,cpp}` — thread-safe persistent text log, date + size rotation.
- `CsvExport.{h,cpp}` — combines `RealTimeGraph` data into one CSV file
  along a unified timestamp axis.
- `SettingsManager.{h,cpp}` + `Settings…` widgets — persistent prefs.
- `RollingStats.h` — O(1) rolling mean/stddev for the numeric pills.
- `crc.{h,cpp}` — bytewise CRC32 mirroring the STM32 HAL block config.

**Pages** (`ui/widgets/`)
- `ImuPanel`, `AttitudeWidget`, `Drone3DWidget` — home-screen panels.
- `PacketAnalyzerWidget` + `PacketDetailWidget` + `FrequencyRibbon` —
  protocol introspection.
- `RcChannelsWidget`, `CalibrationWidget`, `MotorStatusWidget`,
  `ControlLoopPlot` — auxiliary control / monitoring.
- `LogPanel` — in-UI scrollback, tees to `core/Logger`.
- `SimulatorWidget` — SITL control surface (gated on `NAVIGATOR_HAS_SITL`).
- `RealTimeGraph` — the per-trace ring-buffered plotter every panel uses.

**Embedded SITL** — two processes (Navigator side only when `NAVIGATOR_HAS_SITL=1`)
- `sim/vsim/` — the standalone `vsim_d` physics daemon. Holds
  `physics_core` + `motor_model` + `sensor_models` (RK4 6-DOF + rotor
  dynamics + IMU noise, NED throughout) composed by `sim_controller`,
  plus `fifo_transport`. `vsim_proto.h` defines the four-FIFO wire
  format (`/tmp/vsim_{pwm,imu,pose,ctl}`). These classes used to live
  in `navigator/src/vsim/` in-process; they moved here in the vsim_d split.
- `src/vsim/SimWorker.{h,cpp}` — thin supervisor: spawns `vsim_d` via
  `posix_spawnp`, reads pose frames off `/tmp/vsim_pose`, writes control
  (e.g. reset) to `/tmp/vsim_ctl`. No physics, no IMU push, no PWM pull.
- `src/vsim/VsimTypes.h` — Qt type aliases (`Vec3`/`Quat`) for the
  `SimSnapshot` the renderer consumes; physics structs now daemon-side.
- `src/vsim/SimRendererWidget` — OpenGL view of the airframe pose.

---

## 5. Maintenance rules for this doc

- Bump status markers in the tables as PRs land. The tables are the
  contract; the narrative around them is commentary.
- New requirements get IDs at the end of their section (FR-RX-14,
  FR-TX-09, …). IDs are stable — never re-number.
- If a requirement is dropped, leave the row and mark it `❌ (dropped:
  <reason>)` so the ID isn't reused.
- The phase plan is editable. Move rows between phases freely; the
  requirements table is what stays stable.
