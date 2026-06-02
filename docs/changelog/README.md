# Vayu Changelog

Two rolling, component-scoped changelogs track the project as a whole; the older
per-feature files each cover one discrete issue (root cause, fix, lesson).

## Rolling changelogs

| File                                                                                                   | Scope                                                              |
| ------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------- |
| [firmware-modular-refactor-and-safety-hardening.md](firmware-modular-refactor-and-safety-hardening.md) | On-target flight-control firmware (`src/`, Cortex-M).             |
| [gcs-in-app-simulator-and-world-collision.md](gcs-in-app-simulator-and-world-collision.md)             | Navigator GCS (`software/`) + the in-app simulator (`tools/vsim`).|

## Per-feature deep dives

| File                                                                                     | Summary                                                                                                  | Severity | Date       |
| ---------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- | -------- | ---------- |
| [implement-rc-telemetry-and-gcs-ui.md](implement-rc-telemetry-and-gcs-ui.md)             | Integrated iBus RC telemetry, added GCS monitoring UI, and fixed critical UART1 DMA initialization.      | **High** | 2026-03-11 |
| [implement-mahony-filter-and-quaternions.md](implement-mahony-filter-and-quaternions.md) | Implemented Mahony AHRS filter using quaternions, centralized configuration, and fixed gyro sensitivity. | **High** | 2026-03-13 |
