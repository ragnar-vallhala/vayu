# Sim component docs

`vsim` is the physics side of the stack: the rigid-body flight dynamics, environment
(wind, atmosphere, magnetic field) and sensor models. These sources are now **linked
in-process** into the single SITL binary `vayu_sitl_rtos` (`sim/host/`), which fuses them
with the real firmware control logic and drives it headlessly (sensors + RC + environment
in, telemetry + ground truth out) — no standalone daemon. The operator-facing wire
protocol (pose/ctl FIFOs) is defined in [`../include/vsim_proto.h`](../include/vsim_proto.h).

These docs follow the four-layer lifecycle taxonomy used across the repo
(see the global index at [`../../../docs/README.md`](../../../docs/README.md)):

- **[reference/](reference/README.md)** — architecture explainers + formal
  requirements. The living contract: edited in place, never deleted. Start with
  [`reference/sim-architecture.md`](reference/sim-architecture.md) for the whole
  sim/SITL stack and the Pilot scripting API.
- **[plans/](plans/README.md)** — active plans for in-flight work. Deleted once
  the feature ships.
- **[journal/](journal/README.md)** — persistent, time-ordered record (shipped
  design records, analysis). Never deleted.
- **[scratch/](scratch/README.md)** — pre-planning thoughts and studies. Deleted
  once the idea ships or is dropped.
