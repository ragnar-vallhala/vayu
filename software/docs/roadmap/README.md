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
| [sim-geometry-moi-motor-editor.md](sim-geometry-moi-motor-editor.md) | Mesh geometry import, computed moment-of-inertia, motor-mapping editor | ✅ shipped (FR-SIM-11) |

The mockup itself (`../ui-mockup/`) carries a `Ctrl+D` **analysis mode** that tags
every element with its implementation verdict (🔴 structural / 🟡 additive / 🟢
easy); [feature-matrix.md](feature-matrix.md) is the snapshot of those verdicts.
