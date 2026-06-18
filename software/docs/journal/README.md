# Journal — Navigator (GCS)

The persistent, time-ordered record of the Navigator, kept to gauge trajectory.
Nothing here is ever deleted: it documents how shipped features were designed and
how the app evolved.

## `shipped/` — design records of already-shipped features

- [command-registry-and-shortcuts.md](shipped/command-registry-and-shortcuts.md)
  — central command registry, editable shortcuts, command palette, recent-views
  switcher, About/Docs (FR-UX-19–23).
- [gcs-log-replay.md](shipped/gcs-log-replay.md) — whole-GCS read-only log replay
  with a crop/loop scrubber (FR-LOG-05 / FR-UI-19).
- [gcs-source-state-machine.md](shipped/gcs-source-state-machine.md) — the
  telemetry-source state machine (Idle / Fc / Sim / Autotune).
- [gcs-source-state-machine-checklist.md](shipped/gcs-source-state-machine-checklist.md)
  — verification checklist for the source FSM.
- [sim-geometry-moi-motor-editor.md](shipped/sim-geometry-moi-motor-editor.md) —
  mesh-derived mass properties + motor-mapping editor (FR-SIM-11).

## `changelog/` — history of major Navigator updates

- [implement-rc-telemetry-and-gcs-ui.md](changelog/implement-rc-telemetry-and-gcs-ui.md)
- [gcs-in-app-simulator-and-world-collision.md](changelog/gcs-in-app-simulator-and-world-collision.md)
