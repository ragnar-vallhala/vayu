# Whole-GCS log replay with a crop/loop scrubber

Status: ✅ shipped — `src/replay/RecordSink.{h,cpp}` + `ReplaySource.{h,cpp}` and the
`ui/widgets/ReplayBar.{h,cpp}` scrubber are live. Refines/widens **FR-LOG-05** (record
`.bin`) and **FR-UI-19** (replay), the `src/replay/` slot from Phase-2 `2c` in
[`../requirements.md`](../requirements.md). The body below remains as design rationale.

> As-built note: the read-only / source authority is the `SourceController` FSM (see
> [`gcs-source-state-machine.md`](gcs-source-state-machine.md)), **not** the 2-state
> `SessionMode` this doc's Architecture section assumed.

## Context

`requirements.md` 2c frames replay narrowly: *"play a `.bin` through
`DroneProtocol` for offline analysis."* The mockup (`ui-mockup/` replay bar)
asks for more: a **session-wide replay mode** where *every* panel — attitude,
IMU, packets, perf, link stats, control loop — is driven from the recorded log
exactly as if live, the whole UI goes **read-only** (no tx, no ARM, no connect),
and a **crop region** lets you select and **loop** a sub-range with a draggable
playhead and variable speed.

The Simulator is the one exception: it stays interactive, because `vsim_d` is a
separate live process, not a consumer of the replayed link.

## Why it's structural

Today the data path is one-way and live-only:

```
SerialManager / UdpManager ──frames──▶ DroneProtocol ──typed signals──▶ widgets
MainWindow                  ──tx────▶ DroneProtocol ──▶ link
```

Widgets bind directly to `DroneProtocol`'s live signals; `MainWindow` owns the
one `SerialManager`. To replay the *whole* GCS you must (a) swap the frame
*source* without touching any widget, and (b) introduce a global **session mode**
that gates every outbound path. Both are cross-cutting — hence structural.

## Architecture

Introduce a **telemetry-source abstraction** and a **session mode**.

```
            ┌────────────────────────────┐
 live ─────▶│  ITelemetrySource           │
 replay ───▶│   • LiveSource (serial/udp) │──frames──▶ DroneProtocol ──▶ widgets
            │   • ReplaySource (.bin)     │            (decode path UNCHANGED)
            └────────────────────────────┘
 SessionMode {Live, Replay}  ──gates──▶ all tx (Connect/ARM/cmd emitters)
```

- **`comm/ITelemetrySource`** — interface emitting `frameReady(QByteArray)` (raw
  framed bytes). `LiveSource` wraps the existing `SerialManager`/`UdpManager`
  (via `PortArbiter`); `ReplaySource` reads a `.bin`. `MainWindow` connects the
  *active* source's `frameReady` into `DroneProtocol`'s existing parser, so the
  entire decode → typed-signal → widget chain is byte-for-byte identical to live.
  This keeps the structural change at the *seam*, not in the widgets.

- **`src/replay/RecordSink`** (FR-LOG-05) — tees every received frame to a
  timestamped `.bin` (length-prefixed `[t_us:u64][len:u32][frame]`). Lands
  *first*; replay consumes its format. A header records protocol version + start
  wall-clock.

- **`src/replay/ReplaySource`** — memory-maps / indexes the `.bin` into a
  `(t_us → offset)` table on open. A `QTimer`-driven **playback clock** emits
  frames whose `t_us` falls due, scaled by `speed`. API:
  `open(path)`, `play()`, `pause()`, `seek(t_us)`, `setSpeed(x)`,
  `setRange(t0,t1)`, `setLoop(bool)`. On reaching `t1` with loop on, jumps to
  `t0`. The clock is decoupled from wall-clock — that decoupling is what makes
  scrub/crop/speed possible.

- **`SessionMode` in `MainWindow`** — `enum {Live, Replay}`. Entering replay:
  detach `LiveSource`, attach `ReplaySource`, broadcast `sessionModeChanged`.
  `MainToolbar` disables Connect/ARM and shows REPLAY on the LIVE pill;
  `DroneProtocol` (and any command emitter — calibration start, autotune apply,
  mode/RTL/etc.) refuses tx in `Replay`. **One enum, checked at every tx site**,
  is the read-only guarantee. The mockup's read-only lock (`.window.replay`) and
  toast ("Read-only in replay") map directly to this.

- **`ui/widgets/ReplayBar`** — the transport widget (mockup `#replayBar`):
  two stacked scrubbers —
  - **crop overview** (full timeline, draggable in/out handles → `setRange`),
  - **playback** (zoomed to the crop, draggable playhead → `seek`),
  plus play/pause, step, skip-to-crop-edge, and a speed combo. Hosted by
  `MainWindow`, shown only in `Replay`. Mirrors the mockup's `rpUpdate` /
  `rpGrab` / crop-as-fraction model.

## Work items

1. **Record format + `RecordSink`** (FR-LOG-05) — frame tee to `.bin`, header,
   rotation; settings toggle "record telemetry on connect". Unit-test the
   round-trip (write frames → read back identical).
2. **`ITelemetrySource` seam** — extract a `LiveSource` from the current
   `MainWindow`↔`SerialManager` wiring; no behaviour change (pure refactor,
   verify live still works). Pairs well with Phase-1 `1a` `Dispatcher` split.
3. **`ReplaySource`** — `.bin` index + playback clock + seek/range/loop/speed.
   Headless test: known log → frames emitted in order at the right virtual times.
4. **`SessionMode` + read-only gating** — enum on `MainWindow`,
   `sessionModeChanged` signal, tx guards at every emitter, toolbar/pill state.
   The Simulator stays enabled in `Replay`.
5. **`ReplayBar` widget** — two-scrubber crop UI, bound to `ReplaySource`. Reuse
   `core/Theme.h` + `core/ui/Buttons.h`; match the mockup layout.
6. **Menu / command** — File ▸ Open Log… enters replay; `Esc` / Exit → Live.
   Register as commands (see command-registry doc) so they're shortcut-bound.
7. **Docs/tests** — extend `requirements.md` FR-LOG-05 / FR-UI-19 to the
   session-wide scope; smoke test that a replayed log lights up every panel with
   no tx leaking.

## Decisions

- **Re-inject raw frames, not decoded objects.** Feeding `DroneProtocol`'s
  parser keeps one decode path and means new packet types replay for free.
- **One `SessionMode` enum is the read-only authority** — not per-widget
  disabling. Cheaper to reason about and audit (`grep` the guard).
- **Simulator excluded from read-only** — it's a separate live process.

## Out of scope (v1)

Bookmarks/annotations on the timeline, multi-log stitching, exporting a cropped
sub-log, frame-accurate (sub-packet) scrubbing, synchronized video overlay.

## Coordination

The `.bin` header carries the protocol version; a replay of an older log against
a newer `DroneProtocol` must degrade gracefully (decode what it can). Settle the
record format before building `ReplaySource`.
