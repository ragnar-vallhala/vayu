# Navigator (GCS) — documentation

The **Navigator** is Vayu's ground-control station: a Qt6 desktop app (under
`navigator/src/`) that talks to the flight controller over the NavLink wire
protocol. It renders live telemetry (attitude, IMU, RC, motors, control loop),
drives calibration, replays logged sessions, and hosts an embedded SITL
simulator (`vsim_d`) for flying the real firmware control logic without
hardware.

These docs are organised into four lifecycle layers. The layer a doc lives in
tells you how long it lives and how it's maintained:

- **[reference/](reference/)** — *living contract.* Architecture explainers for
  newcomers plus the formal requirements. Edited in place as the code changes;
  never deleted. Start with [`reference/gcs-architecture.md`](reference/gcs-architecture.md)
  for the whole-app picture; also `requirements.md`, `feature-matrix.md`,
  `mockup-to-app.md`, and the `ui-mockup/` design prototype.
- **[plans/](plans/)** — *active plans* for in-flight work. Deleted once the
  feature ships. Empty for now.
- **[journal/](journal/)** — *persistent, time-ordered record* kept to gauge
  trajectory. Never deleted. Holds design records of shipped features
  (`shipped/`), changelogs (`changelog/`), and analysis.
- **[scratch/](scratch/)** — *pre-planning thought / studies.* Deleted once the
  feature it explores ships. Currently the live-rig-tuning study + prior-art
  research.

For the project-wide documentation taxonomy across all components (fc, navlink,
gcs, sim), see [`../../docs/README.md`](../../docs/README.md).
