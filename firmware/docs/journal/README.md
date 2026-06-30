# FC journal

Persistent, time-ordered record kept to gauge trajectory. Never deleted.

- [`changelog/`](changelog/) — firmware changelogs and per-feature deep-dives (Mahony filter, modular refactor & safety hardening, [faster-than-realtime SITL + the real RTOS on host](changelog/sitl-faster-than-realtime-and-rtos-on-host.md)).
- [`log-analysis/`](log-analysis/README.md) — captured flight-log analyses (session, control-loop, motor, kernel, sensor) with plots and recommendations.
- [`deferred/`](deferred/01-attitude-estimate-underread.md) — known issues consciously deferred, kept for trajectory tracking.
- [`memory_report.md`](memory_report.md) — firmware memory (flash/RAM) footprint analysis.
- [`legacy-soft-flow.drawio`](legacy-soft-flow.drawio) — the original hand-drawn software-flow diagram (pre-NavLink-v2, pre-modular-refactor, Mahony-era). Superseded by [`../reference/software-flow.md`](../reference/software-flow.md); kept as a historical snapshot.
