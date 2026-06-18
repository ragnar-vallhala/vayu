# Vayu roadmap

Living progress trackers for cross-cutting efforts that span the firmware
(`src/`, `tools/sim_host/`) and the GCS (`software/`, `tools/vsim/`)
instances. Each file is a punch-list kept current as work lands.

Status legend: ⬜ todo · 🟡 in progress · ✅ done · ⛔ blocked

> **Note:** much of the work tracked here has shipped. The **living tracker** is
> now `docs/changelog/` (e.g. the in-app-simulator/world-collision and
> modular-refactor entries) plus the `software/headless-sdk/` README. These
> roadmaps are kept for the open items and rationale.

## Roadmaps
- [sim-integration.md](sim-integration.md) — firmware ↔ GCS SITL **arm → fly**
  loop (largely shipped via the headless SDK; open items remain).
- [huge-world-import.md](huge-world-import.md) — large-world mesh import for the
  in-app sim renderer (delivered; some P4 items deferred).

> Per-area design docs live elsewhere and are referenced from the
> roadmaps: firmware standard in [`../firmware/`](../firmware/), GCS sim
> design in [`../../software/docs/`](../../software/docs/), and the
> headless harness in [`../../software/headless-sdk/`](../../software/headless-sdk/).
> (The old `build/HANDOFF.md` has been removed; the changelogs are the
> current integration record.)
