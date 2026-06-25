# Vayu Documentation

Start here. Documentation is organized **per component**, and within each
component into **four layers by lifecycle**. This page defines both, then
indexes the Flight Controller (FC) docs, which live in this directory.

> For the whole-system picture — how the four components fit together and the
> runtime topologies — see [`../ARCHITECTURE.md`](../ARCHITECTURE.md).

## Components

Each major part of the stack owns its own docs tree. Keep cross-coupling
minimal — components may *refer* to each other, but each owns its own contract.

| Component | What it is | Docs root |
|-----------|------------|-----------|
| **FC** | Flight-controller firmware (this repo's `src/`, `include/`) | `docs/` *(here)* |
| **NavLink** | Wire protocol / generated codec | [`navlink/docs/`](../navlink/docs/README.md) |
| **GCS** | Navigator ground-control station (`navigator/`) | [`navigator/docs/`](../navigator/docs/README.md) |
| **Sim** | `vsim_d` physics daemon + SITL harness | [`sim/vsim/docs/`](../sim/vsim/docs/README.md) |

## The four layers (lifecycle)

Every component's docs are split into these four directories. The directory a
doc lives in tells you how it is maintained and when it dies:

| Layer | Holds | Lifecycle |
|-------|-------|-----------|
| **`reference/`** | Architecture explainers (for newcomers) + formal requirements | **Living contract.** Edited *in place*; entries are not deleted, only revised. |
| **`plans/`** | Active plans for in-flight work | **Volatile.** A plan is **deleted once its feature ships** — the as-built behavior then lives in `reference/`. |
| **`journal/`** | Time-ordered record kept to gauge trajectory: log captures + system-state analysis (`journal/`), shipped-feature design records (`journal/shipped/`), changelogs (`journal/changelog/`) | **Persistent.** Append-only, never deleted — this is how we look back at how we got here. |
| **`scratch/`** | Pre-planning thought: studies, explorations, "planning before planning" | **Ephemeral.** Deleted once the corresponding feature ships (or is abandoned). |

Rule of thumb: a new contributor reads `reference/`; an active contributor lives
in `plans/`; to understand *why* something is the way it is, read `journal/`.

---

## Flight Controller (FC) docs

The FC component is rooted here in `docs/`.

- **[reference/](reference/README.md)** — firmware architecture & requirements (living)
  - **[software-flow.md](reference/software-flow.md)** — the whole runtime in diagrams (start here for a system picture)
  - [requirements.md](reference/requirements.md) · [coding-guidelines.md](reference/coding-guidelines.md) · [firmware-overview.md](reference/firmware-overview.md)
  - [firmware-control.md](reference/firmware-control.md) · [pipeline-overview.md](reference/pipeline-overview.md) · [coordinate_ref.md](reference/coordinate_ref.md) · [hardware-gotchas.md](reference/hardware-gotchas.md)
  - subsystems: [tasks/](reference/tasks/README.md) · [state-machine/](reference/state-machine/README.md) · [sensor-fusion/](reference/sensor-fusion/README.md) · [imu/](reference/imu/sensor_data_flow.md) · [datasheet/](reference/datasheet/)
- **[plans/](plans/README.md)** — active FC plans ([altitude-hold-and-in-air-plan.md](plans/altitude-hold-and-in-air-plan.md))
- **[journal/](journal/README.md)** — persistent record
  - [log-analysis/](journal/log-analysis/README.md) (time-ordered flight-log captures) · [deferred/](journal/deferred/) (open findings) · [memory_report.md](journal/memory_report.md) · [changelog/](journal/changelog/)
- **[scratch/](scratch/README.md)** — exploratory studies ([application-layer-sandbox.md](scratch/application-layer-sandbox.md))

> **Outside the taxonomy:** [`blog/`](blog/pid-autotuning/index.md) holds
> external-facing narrative write-ups, not internal dev docs.
