# UI ↔ data-rate decoupling

**Status:** in progress (2026-06-14)

## Problem

Symptoms seen against a physical board (≈84 Hz telemetry):

- **Jittery attitude** in the 2D/3D views despite plenty of data — uneven, steppy motion.
- **"Diff" (link latency readout) climbs to 100–180 ms and pins there.**

Neither is a *too-little-data* problem. Both come from the UI being driven at the
**data rate** instead of a fixed render rate, and from telemetry ingest sharing
the GUI thread.

There are two distinct decouplings missing:

1. **Render decoupling** — repaint at a fixed rate (~30/60 Hz) from cached state,
   not once per incoming packet.
2. **IO/processing decoupling** — drain the socket and parse off the GUI thread,
   so the event loop is free to render and to service the socket promptly.

`Diff = GCS wall-clock at the moment a heartbeat is processed − drone timestamp`
(`MainWindow.cpp` `onHeartbeatReceived`). It climbs because the GUI thread is
busy repainting and doesn't return to the event loop to drain the kernel UDP
buffer; packets are processed late, so the measured age grows until the buffer
saturates (the ~180 ms plateau). The same saturation makes repaints irregular →
jitter.

## Reference patterns already in the tree (do it this way)

- `MainWindow::onUiTimer` — 50 ms / 20 Hz pump. IMU panel + attitude **numeric**
  labels render from cached `m_latestImu` / `m_latestAtt`.
- `PacketLogModel` — per-packet `enqueue()` only buffers; a 33 ms timer
  batch-flushes rows to the view.
- `FrequencyRibbon` — per-packet handlers bump counters; a 200 ms timer renders.

## Two shared root causes

1. `RealTimeGraph::appendData()` calls `update()` on **every** sample
   (`RealTimeGraph.cpp`). Any per-packet caller is therefore coupled.
   (`appendSigma()` already buffers without repainting — correct.)
2. Live "instrument" widgets repaint inside their data setters
   (`setAttitude` / `setSnapshot` / `setPos` …) with no render timer in front.

## Coupling inventory

### Tier 1 — GL repaint at data rate (highest cost)

| Widget | Type | Driven by |
|---|---|---|
| `AttitudeWidget` (2D ADI) | QOpenGLWidget | `attitudeReceived` per packet → `setAttitude→update()` |
| `Drone3DWidget` (3D) | QOpenGLWidget | per packet **+** own 30 fps spin timer (per-packet call redundant) |
| `SimRendererWidget` (sim viewport) | QOpenGLWidget | `SimWorker::poseUpdated` ~60 Hz → `setSnapshot→update()` |
| `SimRendererWidget` (Down-Cam PiP) | QOpenGLWidget | second per-frame repaint |

### Tier 2 — CPU custom-paint at data rate

| Widget | Driven by |
|---|---|
| `ControlLoopPlot` (4 graphs + ~16 labels) | `controlLoopDataReceived` **and** `imuReceived` (full IMU rate) |
| `RcChannelsWidget` (sticks, aux, 8 bars, 8 graphs) | `rcReceived` ~50 Hz → `updateChannels` |
| `MotorStatusWidget` (schematic + 4 graphs + labels) | `motorReceived` → `setMotorSpeeds` (schematic `update()` redundant w/ 30 fps anim timer) |
| `SimHudWidget` (FPV HUD) | `poseUpdated` + `setImu` at IMU rate |
| `HorizonHud` (PiP) | `poseUpdated` → `setAttitude→update()` |
| Sim pose label + prop-audio | per pose frame |

### Tier 3 — per-packet but low-rate / cheap (optional change-guards)

- State pill & flight-mode pill `setStyleSheet` per packet (state-filtered, near
  edge-triggered) — add a "changed?" guard.
- `LogPanel::appendLog` per packet — fine normally; unknown-packet spam inserts
  at raw rate, could batch.

### Already decoupled / acceptable (no action)

IMU panel, attitude numeric labels, packet-log model, frequency ribbon,
`PerfWidget` (~1 Hz reports), `CalibrationWidget` (event-driven), autotune
`TuneChart` / `ResponsePlot` (per-eval, sub-Hz).

## Plan (staged)

1. ✅ **DONE — Fix `RealTimeGraph` once.** `appendData()`/`pushState()` now buffer
   and set a dirty flag; an internal ~30 Hz timer (`m_repaintTimer`) repaints only
   when dirty and only while shown (start/stop on show/hide). Decouples **every**
   graph at once — IMU panel, RC, Motor, Control-Loop. (`RealTimeGraph.cpp/.h`)

2a. ✅ **DONE — Home attitude instruments.** `onAttitudeReceived` now caches only;
   `onUiTimer` (bumped 50 ms → **33 ms / 30 Hz**) repaints the 2D ADI + 3D airframe
   from `m_latestAtt`. `Drone3DWidget::setAttitude` no longer calls `update()` (its
   spin timer repaints). (`MainWindow.cpp`, `Drone3DWidget.cpp`)
   Verified: dashboard renders correctly, no regression (live smoothness/Diff to be
   confirmed on hardware).

2b. ⬜ **TODO — Sim viewport/HUD.** Cache the latest `SimSnapshot` in
   `SimulatorWidget`; one ~60 Hz render timer drives both `SimRendererWidget`s +
   `SimHudWidget` + `HorizonHud` + pose label instead of the per-pose-frame
   `poseUpdated` fan-out (both normal-sim and tune-sim paths). Keep `propAudio` on
   the data slot (cheap, latency-sensitive).

2c. ⬜ **TODO — Per-widget label/bar decoupling (RC, Motor, Control-Loop).** Their
   *graph* repaints are already decoupled by step 1, but `setText`/`setValue`/
   `StickGimbal::setPos`/`AuxSwitch::setFromChannel` still run per packet. Cache the
   latest `RcData`/`MotorData`/`ControlLoopData` in the slot; push to the widgets
   from a fixed-rate timer (mirror step 2a). Also drop the redundant per-packet
   `update()` on the motor schematic (its 30 fps anim timer already repaints).

3. ⬜ **TODO — IO/processing off the GUI thread.** Move `QUdpSocket` /
   `SerialManager` + `DroneProtocol` parse onto a worker thread; deliver parsed
   structs to the GUI via queued, coalesced signals. This is the real fix for the
   climbing *Diff*. Higher risk — validate against the board / SITL.

Steps 1 + 2a (done) fix the reported dashboard jitter and decouple all graphs.
Steps 2b/2c finish the render-decoupling layer; step 3 is the larger architectural
change that guarantees the socket is drained regardless of UI load.
