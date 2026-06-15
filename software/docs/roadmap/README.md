# Navigator GCS — Roadmap

Forward-looking design notes for larger GCS features, one file per
initiative. These are *roadmap* documents: the intended shape of a
feature and the plan to build it. Once a feature ships, its status moves
into [`../requirements.md`](../requirements.md) (the living contract);
the roadmap entry stays as the design rationale.

| Doc | Initiative | Status |
|-----|-----------|--------|
| [mockup-to-app.md](mockup-to-app.md) | Reconcile the UI mockup with the codebase; what's shipped vs genuinely new | 🗺️ overview |
| [feature-matrix.md](feature-matrix.md) | Every mockup feature → tier · disposition · target class · effort | 🗺️ index |
| [gcs-log-replay.md](gcs-log-replay.md) | Whole-GCS read-only log replay with a crop/loop scrubber | 🔴 planned (FR-LOG-05 / FR-UI-19) |
| [command-registry-and-shortcuts.md](command-registry-and-shortcuts.md) | Command registry, editable shortcuts, palette, recent-views switcher, About/Docs | 🟡 planned (FR-UX-19–23, Phase-1 1g) |
| [gcs-source-state-machine.md](gcs-source-state-machine.md) | Telemetry-source FSM (Idle/FC/Sim/Autotune/Replay) unifying sources behind `engine.setSource()` | 🔴 planned (Phase C) |
| [gcs-live-rig-tuning.md](gcs-live-rig-tuning.md) | Reuse the autotuner to tune a real drone in a rig over the live link | 🔬 study |
| [gcs-live-rig-tuning-prior-art.md](gcs-live-rig-tuning-prior-art.md) | Engineer's teardown of comparable autotune systems (PX4/ArduPilot/CIFER/MathWorks/SafeOpt) | 🔬 study |
| [gcs-live-rig-tuning-prior-art-deep.md](gcs-live-rig-tuning-prior-art-deep.md) | From-core deep dive per system: control-theory background, block/flow/sequence diagrams, signal sketches | 🔬 study |
| [gcs-sim-mockup-parity-gaps.md](gcs-sim-mockup-parity-gaps.md) | Simulator features in the UI mockup not yet implemented (wind/atmosphere, sensor error models, GPS, power) | 🗺️ gap analysis |
| [sim-geometry-moi-motor-editor.md](sim-geometry-moi-motor-editor.md) | Mesh geometry import, computed moment-of-inertia, motor-mapping editor | ✅ shipped (FR-SIM-11) |

The mockup itself (`../ui-mockup/`) carries a `Ctrl+D` **analysis mode** that tags
every element with its implementation verdict (🔴 structural / 🟡 additive / 🟢
easy); [feature-matrix.md](feature-matrix.md) is the snapshot of those verdicts.
