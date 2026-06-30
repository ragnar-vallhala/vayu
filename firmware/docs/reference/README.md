# FC reference

Living contract: architecture explainers (for newcomers) plus formal requirements. Edited in place; entries are not deleted.

- [`requirements.md`](requirements.md) — firmware spec: SYS → HLR → LLR hierarchy, module prefixes, reserved ID ranges.
- [`coding-guidelines.md`](coding-guidelines.md) — the pragmatic C11 subset (R1–R12): toolchain, naming, types, memory, concurrency, error handling, CI gates.
- [`firmware-overview.md`](firmware-overview.md) — how the engineering standard fits together and how to use the requirements + guidelines docs.
- [`software-flow.md`](software-flow.md) — **the whole runtime in diagrams**: boot, every task, the IPC, and the sense→estimate→control→actuate + comms hot paths (Mermaid; verified against source).
- [`firmware-control.md`](firmware-control.md) — the cascade control path (rate/attitude loops, mixer) explained.
- [`pipeline-overview.md`](pipeline-overview.md) — end-to-end sensor → estimator → control → actuator data pipeline.
- [`trace.md`](trace.md) — generated requirement ↔ source traceability table (`tools/dev/trace.py`).
- [`coordinate_ref.md`](coordinate_ref.md) — NED axis conventions used across estimation and mixing.
- [`hardware-gotchas.md`](hardware-gotchas.md) — survival notes (e.g. no locks in high-frequency loop paths).
- [`tasks/`](tasks/README.md) — RTOS task layout and per-task contracts.
- [`state-machine/`](state-machine/README.md) — system states and transition diagrams.
- [`sensor-fusion/`](sensor-fusion/README.md) — Mahony / complementary filter derivation and tuning.
- [`imu/`](imu/sensor_data_flow.md) — IMU acquisition and sensor data-flow walkthrough.
- `datasheet/` — vendor datasheets for on-board sensors.

The runtime software flow is [`software-flow.md`](software-flow.md) (above); the old hand-drawn `soft_flow.drawio` was retired to [`../journal/legacy-soft-flow.drawio`](../journal/legacy-soft-flow.drawio) as a pre-v2 snapshot.
