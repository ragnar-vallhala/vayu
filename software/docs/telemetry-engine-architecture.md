# Telemetry engine architecture (single engine, 3 sources, decoupled UI)

**Status:** proposed (2026-06-14)
**Follows from:** [ui-rendering-decoupling.md](ui-rendering-decoupling.md) — render throttling
cut *Diff* from ~180 ms to ~90–100 ms, but it persists and the 3D view still lags,
because parsing + GL composite still share the GUI thread.

## Goal

One processing engine, three interchangeable entry points, and a UI that is
**totally decoupled** from data processing — it renders at a fixed rate from a
state snapshot and never runs at the packet rate.

```
   ┌─ LiveSource (UDP + serial)  ┐
   ├─ ReplaySource (recorded .bin)│   bytes     ┌──────── worker thread ────────┐
   └─ SimSource (in-app vsim)     ┘ ─────────▶  │  TelemetryEngine               │
        (exactly one active)        (queued)    │   DroneProtocol parser         │
                                                │   → VehicleState store (mutex/  │
   commands  ◀───────────────────── (queued)   │      double-buffer)            │
                                                └───────────────────────────────┘
                                                          ▲ snapshot()
                                    GUI thread            │ (lock-free copy)
   widgets ◀── RenderClock (30/60 Hz) ── reads ──────────┘
```

## What already exists (the 80% we keep)

- `ITelemetrySource` emits `bytesReceived(QByteArray)` (`comm/ITelemetrySource.h`).
- `LiveSource` fans in serial+UDP; `ReplaySource` replays a `.bin`; the in-app sim
  emits bytes via `SimulatorWidget::dataReceived`. All three already feed the same
  `DroneProtocol::processData`.
- `DroneProtocol` is the single parser → typed signals.
- `SessionState` already models live vs replay.

So the "single engine, 3 sources" seam **already exists at the byte level.** The
work is threading it and changing how the UI consumes it.

## What's wrong today

1. **Parser on the GUI thread.** `QUdpSocket`, the replay timer, and the sim all
   run on the GUI thread, so parsing competes with rendering. When the thread does
   a GL composite, the socket isn't drained → kernel buffer backs up → *Diff*
   climbs.
2. **UI wired to per-packet signals.** Every widget update is driven by
   `DroneProtocol::xxxReceived` at the packet rate (partially mitigated by the
   render-throttle work, but the *dispatch* still happens per packet on the GUI
   thread).

## Target design

### `VehicleState` — the canonical model
A single struct holding the latest of everything the UI shows: attitude, imu,
baro, rc, motors, flight-mode, vehicle state, heartbeat/link stats, packet
counters, and per-field `lastUpdateMs` for staleness. Owned by the engine,
guarded by a mutex (or published as an immutable double-buffered snapshot). The
UI never touches engine internals — it calls `engine.snapshot()` to get a cheap
copy.

### `TelemetryEngine` — the single processing engine (worker thread)
- Owns `DroneProtocol` + the `VehicleState` store + the active `ITelemetrySource`
  + the transports.
- One input: `feedBytes()` (the active source's `bytesReceived`, a same-thread or
  queued connection).
- Per parsed packet: update the store, bump counters. **No per-packet signal to
  the UI.**
- Emits only **low-rate, event-like** signals as queued connections (log lines,
  status/flight-mode *transitions*, errors) — these are naturally sparse.
- Raw packets for the Packet Analyzer go out as a **batched** queued signal (the
  analyzer already batches in `PacketLogModel`).

### Sources — three entry points, one active
`LiveSource | ReplaySource | SimSource`, all `ITelemetrySource`. `SessionState`
selects exactly one; `engine.setSource(...)` swaps it. They live on the worker
thread (or push bytes to it via a queued `feedBytes`).

### UI — fixed-rate, pull-based
A single `RenderClock` QTimer (30/60 Hz) on the GUI thread calls `engine.snapshot()`
and pushes the latest values to every widget — the same cache→timer pattern we
already use for the IMU panel, generalised to the whole dashboard. **All
per-packet `connect(...Received...)` wiring is deleted.**

### Commands (outbound)
ARM, SET_PID, time-sync, hello, etc. The transports live on the worker thread, so
the UI must not write the socket directly (Qt socket affinity). UI calls
`engine.send(bytes)` → queued onto the worker thread → transport write.

## Why this fixes both symptoms
- Socket is drained on the worker thread the instant data arrives, regardless of
  what the GUI/GL is doing → *Diff* ≈ true link latency.
- UI work is bounded by the render clock and reads an O(1) snapshot → no
  per-packet UI cost, smooth 3D.
- One obvious place for all telemetry state and all source-switching.

## Key decisions (see PR discussion)
1. **UI consumption:** pull a snapshot at render rate (recommended) vs push a
   single coalesced "changed" signal.
2. **Command path:** route outbound through `engine.send()` (queued) so transports
   live on the worker thread.

## Migration (incremental, each phase shippable & testable)

- **Phase A — decouple the UI (single-threaded first).** Add `VehicleState` +
  `snapshot()`, updated by the *existing* GUI-thread parser. Add the `RenderClock`
  and move every widget update into it, reading the snapshot. Delete the per-packet
  UI wiring. *Result:* UI fully decoupled; still one thread, but now trivial to
  move. Verifiable on the board (smoothness) without threading risk.
- **Phase B — move the engine to a worker thread.** Put `TelemetryEngine`
  (parser + store + sources + transports) on a `QThread`; UI reads `snapshot()`
  cross-thread; commands via queued `send()`. *Result:* fixes *Diff*.
- **Phase C — unify sources.** Make sim + replay first-class `ITelemetrySource`s
  behind `engine.setSource()`, with one lifecycle/switching path. *Result:* the
  clean "3 entry points" model.

Phase A delivers most of the perceived smoothness with near-zero risk; Phase B is
the latency fix; Phase C is the cleanup that makes it "clear and manageable".
