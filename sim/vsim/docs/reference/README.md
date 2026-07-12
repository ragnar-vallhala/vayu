# Sim reference

Architecture explainers and formal requirements for the sim. Living contract:
edited in place, not deleted.

- [`sim-architecture.md`](sim-architecture.md) — **the whole sim/SITL stack in diagrams**:
  the in-process `vayu_sitl_rtos` binary (firmware + physics fused), the pose/ctl transport,
  the SITL seam, end-to-end data flow, and the `vayu_headless` **Pilot** scripting API
  (Mermaid; verified against source).
- [`autotune-methodology.md`](autotune-methodology.md) — how the autotuner poses
  and solves the PID tuning problem, the cost surface pathologies it must handle,
  and the supporting figures in [`figures/`](figures/).

See the component overview at [`../README.md`](../README.md).
