# Vayu GCS — Implementation Notes

The mockup (`index.html` + `styles.css` + `app.js`) shows **what the UI looks like**.
This file records **how it must behave** in the production C++/Qt GCS — the intent and
the intentional deviations from the *current* app that a static page can't express.

Read this alongside the mockup before implementing a screen. When a control's runtime
behavior matters (when it applies, what it writes, what's gated), it belongs here.

### Entry format

Each note has a stable id (`AT-1`, `SIM-2`, …) so it can be referenced in code review
and commits. Keep them short:

> **Context** — where in the UI · **Now** — current app behavior ·
> **Required** — what it must do · **Where** — UI element + code touchpoints ·
> **Why** — one line.

---

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
