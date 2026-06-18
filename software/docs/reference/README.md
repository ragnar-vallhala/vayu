# Reference — Navigator (GCS)

The living contract for the Navigator: architecture explainers for newcomers and
the formal requirements. Edited in place as the code changes; not deleted.

- [gcs-architecture.md](gcs-architecture.md) — **the whole Navigator in diagrams**:
  the worker-thread telemetry engine + ~30 Hz UI decoupling, the source FSM, RX decode,
  command tx, in-app sim hosting, and record/replay (Mermaid; verified against source).
- [requirements.md](requirements.md) — the engineering spec: system + functional
  requirements (by subsystem, with status), phase plan, architecture map, and
  maintenance rules. The tables are the contract.
- [feature-matrix.md](feature-matrix.md) — every feature group from the UI
  mockup mapped to a tier and a disposition (shipped / enhance / new).
- [mockup-to-app.md](mockup-to-app.md) — reconciles the interactive UI mockup
  against what the app already has, so "implement all this" stays honest.
- [ui-mockup/](ui-mockup/) — the interactive HTML/JS/CSS design prototype that is
  the UI target.
