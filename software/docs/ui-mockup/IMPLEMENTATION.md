# Navigator GCS — Implementation Plan & Notes

This doc is the bridge between the mockup (`index.html` + `styles.css` + `app.js`,
which shows **what** the UI looks like) and the production C++/Qt GCS. It carries
two things:

1. **The implementation plan** — every mockup↔app gap, bucketed by depth, ordered,
   and assigned to a branch (this file's main body).
2. **Per-control behaviour notes** — runtime intent a static page can't express
   (the `AT-*` / `SIM-*` notes at the bottom).

Read it alongside [`../roadmap/`](../roadmap/) — the roadmap holds the per-initiative
design rationale; this file holds the **sequencing and the contract** for building it.

---

## Status (2026-06-13)

**Phases 1–3 are shipped and merged to `main`** — the entire GCS-UI roadmap.
Each work item landed on its own branch with unit tests (`software/tests/`,
QtTest + ctest); the FRs are marked ✅ in [`../requirements.md`](../requirements.md).

- **Phase 1 (seams):** 1A command registry · 1B telemetry-source seam ·
  1C record format + `RecordSink` · 1D `SessionMode` read-only gating.
- **Phase 2 (additive):** 2A shortcuts editor + `ShortcutsManager` ·
  2B command palette · 2C recent-views switcher · 2D `ReplaySource` ·
  2E `ReplayBar` + Open Log (whole-GCS replay end-to-end).
- **Phase 3 (small UI):** `AboutDialog` + Documentation entry.

**Remaining — autotune apply (AT-1/AT-2) + sim track.** The firmware already
supports live PID updates: `CMD_SET_PID` (0x000A, `comm/comm_types.h`) is handled
by `pid_config_apply_command` (apply to the live controller + persist). So AT-1
is **GCS-side** — a command encoder (`protocol/CommandCodec`, done) plus the
propose-not-apply UI: a Proposed Gains table and an explicit *Apply Gains to
Firmware* button that emits `CMD_SET_PID` per (rate, axis), replacing the
auto-apply checkbox. AT-2 needs the autotuner to stream the current/best gain
*vectors* each eval (today `#EVAL` streams only cost/best scalars) so the GCS can
show current-vs-best live. The deeper sim items — sensor-noise (FR-SIM-04), RC
bridge, SITL FPV — depend on `vsim_d` / `vsim_ctl`.

---

## How the gaps are sequenced

The mockup's `Ctrl+D` analysis mode tags every element with a tier; the
[feature matrix](../roadmap/feature-matrix.md) is the snapshot. Those tiers collapse
into **three gap classes**, and we build them strictly in this order:

| Class | Tier | What it is | Risk |
|---|---|---|---|
| **Deep structural** | 🔴 | New cross-cutting *seams* — an abstraction or mode the rest of the app routes through | high |
| **Long additive** | 🟡 | Substantial features that sit **on top of** a seam; localized to a few new classes | medium |
| **Small UI** | 🟢 | Self-contained surfaces (a dialog, a menu entry); no architecture change | low |

**Two hard rules** (both stated by the build directive):

- **Deep → additive → small.** Land the seams first; everything else binds to them.
- **No work may depend on a *later* phase.** The dependency graph below points only
  *backward*. A Phase-2 item may use Phase-1 seams; a Phase-1 item may not assume any
  Phase-2/3 code exists.
- **Each major work item = its own branch off `main`**, named in the tables. Branches
  merge back to `main` in phase order; a later branch starts from `main` *after* its
  prerequisites have merged (or rebases onto them if developed in parallel).

### Dependency graph (all edges point backward — earlier enables later)

```
Phase 1 (seams)                  Phase 2 (additive)            Phase 3 (small)
─────────────────                ──────────────────            ───────────────
1A command-registry ─────┬─────▶ 2A shortcuts-editor
                         ├─────▶ 2B command-palette
                         ├─────▶ 2C recent-views
                         ├─────▶ 2E replay-bar  ◀─┐    ┌─────▶ 3A about-dialog
                         └────────────────────────┼────┴─────▶ 3B docs-entry
                                                  │
1B telemetry-seam ──┬──▶ 1C record-sink ──┐       │
                    ├──────────────────────┴─────▶ 2D replay-source ─▶ 2E replay-bar
                    └──▶ 1D session-mode ────────────────────────────▶ 2E replay-bar
```

No arrow runs right-to-left across a phase boundary, so any prefix of the plan is a
shippable GCS on its own.

---

## Phase 1 — Deep structural (the seams)

Four foundational changes. 1A and 1B have **no prerequisites** and can be built in
parallel; 1C/1D depend only on earlier Phase-1 work. Each is a pure-seam change with
**no user-visible behaviour change on its own** (1A/1B are refactors; 1C adds an
opt-in toggle; 1D adds an unreachable-until-Phase-2 mode).

| # | Work | Branch | Depends on | FR / roadmap | Effort |
|---|---|---|---|---|---|
| **1A** | **Command registry** — `core/Command` + `core/CommandRegistry`; migrate the existing `installShortcuts()` `QShortcut`s and `buildMenuBar()` `QAction`s into registered commands; one `QAction` per command shared by menu + toolbar + shortcut; `when`-context gating via `QAction::setEnabled`. **Pure refactor — every existing shortcut must still fire.** | `feat/command-registry` | — | FR-UX-19 (core), [doc](../roadmap/command-registry-and-shortcuts.md) | M |
| **1B** | **Telemetry-source seam** — `comm/ITelemetrySource` emitting `frameReady(QByteArray)`; extract `LiveSource` wrapping the current `SerialManager`/`UdpManager` (via `PortArbiter`); `MainWindow` feeds the *active* source into `DroneProtocol`'s parser. **Pure refactor — decode path byte-for-byte identical; live link unchanged.** | `feat/telemetry-source-seam` | — | [replay doc](../roadmap/gcs-log-replay.md) WI-2 | M |
| **1C** | **Record format + `RecordSink`** — tee every received frame to a length-prefixed `.bin` (`[t_us:u64][len:u32][frame]`) with a header (protocol version + start wall-clock); settings toggle "record on connect"; round-trip unit test. Lands *before* replay because replay consumes its format. | `feat/record-sink` | 1B | FR-LOG-05 | M |
| **1D** | **`SessionMode` + read-only gating** — `enum {Live, Replay}` on `MainWindow`, `sessionModeChanged` signal, tx guards at **every** emitter (connect / ARM / calibrate / autotune-apply / mode / time-sync), toolbar shows `REPLAY` on the LIVE pill. One enum is the read-only authority. Simulator stays interactive. Mode is reachable only once 2D/2E exist, so this ships dormant. | `feat/session-mode` | 1A, 1B | [replay doc](../roadmap/gcs-log-replay.md) WI-4 | M |

**Deviation from the roadmap sketch (1A):** the roadmap assumes
`SettingsManager` is a key-value store (`shortcuts/<id>`), but it is actually a
fixed-struct binary blob (`GcsSettings` + versioned `QDataStream`). So shortcut
overrides need a **dedicated store** — either a small `shortcuts.ini` via `QSettings`
or a new keyed blob — *not* an extension of `GcsSettings`. Decide this in 1A; the
`ShortcutsManager` persistence (used by 2A) reads from it.

---

## Phase 2 — Long additive (features on the seams)

Each binds to a Phase-1 seam and is otherwise self-contained. 2A–2C hang off the
command registry (1A) and are independent of each other; 2D/2E complete replay.

| # | Work | Branch | Depends on | FR / roadmap | Effort |
|---|---|---|---|---|---|
| **2A** | **`ShortcutsEditorDialog`** — VS Code-style searchable rebind table: click-to-record chord (`QKeyEvent`→`QKeySequence`), conflict detection, per-row + global reset, "modified" markers; persists overrides via `ShortcutsManager`. `Help ▸ Keyboard Shortcuts` / `Ctrl+Alt+K` opens it. | `feat/shortcuts-editor` | 1A | FR-UX-19 | M |
| **2B** | **`CommandPalette`** (`Ctrl+Shift+P`) — `QLineEdit` + filtered `QListView` over `CommandRegistry::all()`; fuzzy match on title/category; Enter triggers the `QAction`. | `feat/command-palette` | 1A | FR-UX-20 | M |
| **2C** | **Recent-views switcher** — `core/ViewHistory` (MRU stack pushed on every `setCurrentIndex`) + `RecentViewsOverlay` (hold-`Ctrl`, tap to cycle, Shift reverses, Esc cancels; tap = toggle-previous). Depth is a setting (`ui/recentViewsCount`, default 5). Default bind `Ctrl+Tab` (interceptable in Qt; mockup used `` Ctrl+` `` only because browsers reserve `Ctrl+Tab`). | `feat/recent-views` | 1A | FR-UX-21 | M |
| **2D** | **`ReplaySource`** — `.bin` index `(t_us→offset)` on open; `QTimer`-driven playback clock decoupled from wall-clock; `play/pause/seek/setSpeed/setRange/setLoop`; loops `t0..t1`. Headless test: known log → frames emitted in order at the right virtual times. | `feat/replay-source` | 1B, 1C | FR-UI-19 | L |
| **2E** | **`ReplayBar`** — two stacked scrubbers (crop overview with draggable in/out handles → `setRange`; playback zoomed to crop with draggable playhead → `seek`), play/pause/step/skip-to-edge + speed combo; hosted by `MainWindow`, shown only in `Replay`. `File ▸ Open Log…` (registered command) enters replay; `Esc`/Exit → Live. | `feat/replay-bar` | 2D, 1D, 1A | FR-UI-19 | L |

### Independent additive track — Simulator/firmware features

These are 🔴/🟡 **enhancements gated on `vsim_d` / `vsim_ctl` / firmware**, *not* on the
GCS-UI seams above. They share no dependency with Phases 1–2 and can proceed on their
own branches in parallel — **do not let them block the UI critical path.**

| Work | Branch | Gated on | FR / note |
|---|---|---|---|
| **In-GCS autotune (current/best)** — C++ optimizer over the `SimWorker` eval loop streaming `current` + `best` each evaluation; **propose-only**, never auto-apply (see **AT-1 / AT-2**). | `feat/gcs-autotune` | sim eval loop | `TuneChart` exists |
| **Sensor fault / noise injection** | `feat/sim-sensor-noise` | `vsim_d` sensor model + `vsim_ctl` opcode | FR-SIM-04 |
| **RC bridge into SITL** | `feat/sim-rc-bridge` | new `vsim_ctl` input path | matrix row |
| **SITL FPV / camera render** | `feat/sim-fpv` | `SimRendererWidget` cam views | matrix row |

### Widget-enhancement line items (small-to-medium, fold into the owning widget)

Normal additive line items against an existing widget — each its own short branch.
Firmware-paired ones are gated and tracked but not on the UI critical path.

- **Calibration wizard** (gated, fig-8) — step state machine + animation on
  `CalibrationWidget`. `feat/calibration-wizard` · M.
- **World config** (wind/mag/terrain/spawn) — `WorldEditorWidget` (+ FR-SIM-08 wind).
  `feat/world-config` · M.
- **Telemetry stream-rate config** — `SettingsManager` + `DroneProtocol`
  `SET_STREAM_RATE`. **Firmware-paired** — gated. `feat/stream-rate` · M.

---

## Phase 3 — Small UI (🟢 easy)

Self-contained, low-risk. Both Help surfaces only need the command/menu layer (1A).

| # | Work | Branch | Depends on | FR | Effort |
|---|---|---|---|---|---|
| **3A** | **`AboutDialog`** — static: name / version / build / protocol version / flight-stack components. `Help ▸ About`. | `feat/about-dialog` | 1A | FR-UX-22 | S |
| **3B** | **Documentation entry** — `Help ▸ Documentation` opens the bundled `docs/` via `QDesktopServices::openUrl`. | `feat/docs-entry` | 1A | FR-UX-23 | S |

3A and 3B are tiny and may share one `feat/help-surface` branch if preferred.

---

## Not for the app

- **Analysis mode (`Ctrl+D`)** is a mockup-only design instrument; it does **not**
  ship in Navigator. It lives only in `ui-mockup/`.

## Branch & commit conventions (carry forward)

- Branch off **`main`**; merge back in phase order. No AI-attribution trailers. No
  company names in source/comments — vendor-neutral, technical reasons only.
  Commit/push **only when asked**.
- Internal `localStorage`/settings keys stay `vayu.*` (not user-facing).
- The real app is **further along than the mockup implies** — always check
  `software/src/ui/widgets/` before assuming a feature is unbuilt.

---

# Per-control behaviour notes

How specific controls must *behave* in the C++/Qt GCS — intent and intentional
deviations from the *current* app that a static page can't express. Each note has a
stable id (`AT-1`, `SIM-2`, …) referenceable in code review and commits.

> **Context** — where in the UI · **Now** — current app behaviour ·
> **Required** — what it must do · **Where** — UI element + code touchpoints ·
> **Why** — one line.

These feed the **In-GCS autotune** item in the independent additive track above.

## AT-1 — Autotune *proposes* gains; applying to firmware is an explicit click

**Context.** Autotune tab → **Proposed Gains** table + **Apply Gains to Firmware** button.

**Now.** The optimizer writes the tuned gains **directly** — the `--apply` /
"Apply + persist best gains on finish" path pushes the winning P/I/D to the shared gain
store / firmware automatically when the run finishes.

**Required.** The autotuner must **only find** the gains and surface them in the
**Proposed Gains** table. It must **never write to the firmware or the live gain store
on its own.** Committing is a **separate, explicit operator action**: review the
proposed P/I/D, then click **Apply Gains to Firmware** to send them. Until that click,
the running vehicle keeps its existing gains.

- The tuner may still **persist its result to disk** (e.g. `autotune_result.json`) as a
  record and to populate the table — that is a draft, **not** a live apply.
- There is **no auto-apply option**. The mockup's old *"Apply + persist best gains on
  finish"* checkbox has been **removed** to make this unambiguous; the C++ port must not
  re-introduce an auto-apply path (`--apply` becomes a no-op / is dropped).
- **Apply Gains to Firmware** is the *only* path that emits `CMD_SET_PID`.

**Where.**
- UI: Autotune tab — Proposed Gains table (`#ag-roll-*`, `#ag-pitch-*`, `#ag-yaw-*`) and
  the **Apply Gains to Firmware** button.
- Code: `SimulatorWidget` autotune runner; the C++ port of
  `tools/autotune/autotune.py` (`apply_gains`, `--apply`). The Apply button →
  `DroneProtocol` `CMD_SET_PID`.

**Why.** Auto-writing tuned gains to a live airframe is unsafe and surprising; an
explicit, reviewed apply is the safe default and matches how the mockup presents it.

---

## AT-2 — Gains panel shows *current* and *best* live, not a static result

**Context.** Autotune tab → **Gains — current vs best** table (right pane).

**Now.** The mockup originally showed one static "Proposed Gains" table, as if the gains
only exist at the end of the run.

**Required.** Mirror the matplotlib dashboard: throughout the run the panel shows **two
live values per gain** —
- **current** — the gain vector under evaluation *right now* (jumps around as the
  optimizer explores the space);
- **best** — the best-so-far vector (improves monotonically, settles as it converges).

The **proposed tune is simply `best` at completion** — there is no separate "final"
computation. `best` turns green (converged) when the run finishes. Before a run, `best`
holds the seed/last gains and `current` shows `—`.

Per **AT-1**, **Apply Gains to Firmware** commits **`best`** (never `current`).

**Where.**
- UI: Gains table — best cells `#ag-{axis}-{0..2}`, current cells `#agc-{axis}-{0..2}`.
- Code: the tuner must stream both the per-eval candidate vector *and* the running best
  to the GCS each evaluation (not just a final result), as `tools/autotune` already does
  for the matplotlib plotter (`current_x` / `best_x`).

**Why.** Seeing `current` move and `best` lock in is how the operator judges whether the
search is converging — a single end-of-run number hides all of that.

---

## Open reconciliations

Things in the mockup that still contradict a note above and should be fixed in the mock:

- _(none)_
