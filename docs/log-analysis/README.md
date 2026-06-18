# Flight-log analysis

Decoded analyses of recorded NavLink v2 telemetry sessions (`.bin`) captured
from real hardware. Each log is a `VREC` container (see
`software/src/replay/RecordFormat.h`) holding raw inbound byte chunks; every
chunk is replayed through the **generated** NavLink v2 Python codec
(`navlink/generated/python/navlink_msgs.py`) so the decode path is identical to
live/replay and stays in lock-step with `dialect.json`.

## Tooling

`parse_log.py` — decode a session and print a summary; `--csv DIR` dumps one CSV
per message type for plotting.

```sh
python3 docs/log-analysis/parse_log.py /path/to/export.bin
python3 docs/log-analysis/parse_log.py /path/to/export.bin --csv docs/log-analysis/csv
```

Generated `csv/` dumps and stray `.bin` files are git-ignored (regenerable /
bulky); the curated `*/export-*.bin` archived alongside each analysis is kept as
the source of truth so old logs stay re-analysable.

## Archives

Each capture gets its own dated folder (`YYYYMMDD-hhmmss`, from the log's own
timestamp) holding the raw `.bin` plus all analysis of it.

- [`20260617-124210/`](20260617-124210/) — **2026-06-17 12:42:10**, first
  hardware bring-up bench session: 6.25 min, 38 k frames, 0 CRC errors. Never
  reached IN_AIR (handheld/bench). Surfaced 5 telemetry data-quality bugs, a
  ControlTrace°/AttitudeEulerᵣ unit mismatch, ~2 k telemetry TX overflows,
  correct RC-loss → FAILSAFE behaviour, and uncalibrated accel (+8 %) / mag.
  - [`README`](20260617-124210/README.md) — provenance, header, message
    inventory, raw findings log
  - [`session-analysis.md`](20260617-124210/session-analysis.md) — full session
  - [`sensor-analysis.md`](20260617-124210/sensor-analysis.md) — sensor cal & reporting
