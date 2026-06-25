# Sim component docs

`vsim` is the physics side of the stack: the **`vsim_d` daemon** that integrates
rigid-body flight dynamics, environment (wind, atmosphere, magnetic field) and
sensor models, plus the **SITL / headless harness** that drives the real firmware
control logic headlessly (sensors + RC + environment in, telemetry + ground
truth out). The wire protocol between `vsim_d` and its clients is defined in
[`../include/vsim_proto.h`](../include/vsim_proto.h).

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
