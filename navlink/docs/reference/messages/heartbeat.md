# Heartbeat

> Updated for NavLink v2. Authoritative wire spec: navlink/dialect.json + docs/analysis/navlink-v2-spec.md.

Heartbeat is sent FC → GCS at 1 Hz to indicate the device is alive and to report its high-level state.

In NavLink v2 this is **`HEARTBEAT` (msgid 0)**. It is **not** header-only: the payload carries 7 fields — `type`, `autopilot`, `base_mode`, `system_status`, `nav_state`, `capabilities` (u32), and `timestamp` (u32). See `nav_state` in `../../navlink/dialect.json` for the flight-state enum. Byte layout: `../analysis/navlink-v2-spec.md`.

## Changelog

| Date       | Author               | Description                       |
| ---------- | -------------------- | --------------------------------- |
| 06/03/2026 | Ashutosh Vishwakarma | Initial version                   |
| 11/03/2026 | Antigravity          | Specified 0x0 code                |
| 06/2026    | —                    | NavLink v2: msgid 0, 7-field body |
