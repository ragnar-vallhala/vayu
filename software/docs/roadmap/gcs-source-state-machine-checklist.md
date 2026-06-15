# GCS source state machine — runtime test checklist

Manual verification for the `SourceController` migration (see
[gcs-source-state-machine.md](gcs-source-state-machine.md)). Unit tests cover the
FSM logic headless; this checklist covers the live wiring that needs the real
sim + board. Tick `[x]` as you go and jot results in the Notes.

**Where to look:**
- **Connection pill** — bottom status bar, far left (`● Connected: …`). Driven by
  the FSM: Fc → port, Sim → `SIM`, Autotune → `AUTOTUNE`, Idle/Replay → `Disconnected`.
- **LIVE / REPLAY badge** — top-right toolbar chip (LIVE heartbeat blink; flips to
  REPLAY in the Replay state).

Build/run: `cmake --build build -j"$(nproc)"` then `./build/Navigator` (real GL
needed — the 3D view crashes under `QT_QPA_PLATFORM=offscreen`).

---

## 1. Per-state entry (pill + feed)
- [*] **Idle** at startup — pill "Disconnected", no telemetry, serial controls enabled.
- [*] **Fc (serial)** — Connect to the board → pill shows the port, telemetry flows, LIVE blinks, time-sync Diff converges (green).
- [*] **Fc (UDP)** — Connect `udp:14555` → same, over the WiFi bridge.
- [*] **Sim** — Simulator ▸ Start → pill `SIM`, telemetry flows, serial controls locked.
- [*] **Autotune** — start a tuning run → pill `AUTOTUNE`, roll-response plot updates, no main-panel telemetry feed. // TODO The stop button is not a quick exitm it takes times to complete the current iter, this can be made instantanios
- [*] **Replay** — File ▸ Open a `.bin` → toolbar shows REPLAY, every panel plays back, scrubber works. // TODO the bottom bar still shows disconncted can be written Replay

Notes:

---

## 2. Transitions (fully-connected graph — exactly one feed at a time)
- [*] Fc → Replay (open a log while connected): live feed stops, replay drives, link closed.
- [*] Fc → Sim (Start sim while connected): board link drops, sim feeds.
- [*] Fc → Autotune: board link drops (release + close), tuner runs isolated.
- [*] Sim → Replay.
- [*] Sim → Autotune.
- [*] Replay → Fc / Sim / Autotune (no need to "exit replay" first).
- [*] Autotune → Fc / Sim / Replay.
- [*] Every "→ Idle": Disconnect (Fc), Stop sim (Sim), exit Replay, tuner finish/cancel (Autotune).
- [*] **No double telemetry** ever (board + sim at once) — watch packet rate / IMU traces.

Notes:

---

## 3. Gating invariants
- [*] tx allowed only in Fc & Sim: ARM/DISARM, PID set, Calibrate, time-sync work in Fc/Sim; blocked (warn toast) in Replay/Idle/Autotune.
- [*] Apply tuned gains needs Fc: in Autotune it warns "not connected"; Autotune → connect board (Fc) → Apply → gains sent. // TODO this will be next phase of tuning live drone on same algorithm and method that we tune sim ones on
- [*] Serial controls (port/baud/Connect) enabled only in Idle/Fc.
- [*] Export Log enabled only in Fc/Sim (and while an export is running).
- [*] Record-on-connect starts in Fc/Sim, not Replay.
- [*] LIVE blinker + link-loss watchdog fire only in Fc (unplug board → "link lost", auto-reconnect recovers).

Notes:

---

## 4. Edge cases
- [ ] Connect to a bad port → lands Idle, no stuck state, PortArbiter released (can retry).
- [ ] Open a bad/corrupt `.bin` → error toast, stays Idle (no half-replay).
- [ ] Re-open a different log while already replaying → old drops, new plays.
- [ ] App exit from each state (esp. Sim, Autotune, Replay) → clean shutdown, no hang/SIGABRT, sim daemon + tuner threads gone.
- [ ] Known gap — confirm severity: open a Replay while the interactive sim is running → replay feeds but the sim daemon keeps spinning (pill becomes REPLAY). Acceptable, or worth the follow-up fix? // TODO always do a clean exit by closing all non needed threads, even for the tuning one

Notes:

---

## 5. Regression (unchanged, but verify nothing broke)
- [*] Replay transport: play/pause/seek, speed, loop, crop.
- [*] Time-sync still locks on the board after the migration.
- [*] Recording → produces a replayable `.bin`; live export works.

Notes:

---

| Date | Tester | Build / commit | Result |
| ---- | ------ | -------------- | ------ |
|      |        |                |        |

---

## Fixes applied (2026-06-15) — re-verify §4
From the first pass's TODOs:
- **§1 Replay pill** — the bottom bar now shows a distinct **`● Replay`** (accent
  colour), not "Disconnected". `MainStatusBar::setReplayStatus()`.
- **§4 clean exit** — `teardownSim`/`teardownAutotune` are now wired: entering any
  other state (or app exit) stops the sim daemon / cancels the tuner. Reconcile
  handlers are state-guarded so a stale "stopped" can't demote the wrong state.
- **Autotune Stop is now instant** — a cancel flag threads through `runRollout` +
  `waitLevel` (cancel-aware sleeps), so Stop aborts within ~10 ms instead of
  finishing the current iteration.
- Added 5 teardown-cleanliness unit tests in `tst_source_controller`.

Please re-run **§4** (esp. clean exit from each state, and the former
"replay-while-sim" gap — the sim should now stop), and confirm Autotune Stop is
snappy. Remaining deferred: live-drone tuning (next phase).
