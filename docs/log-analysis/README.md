# Flight-log analysis

Decoded analyses of recorded NavLink v2 telemetry sessions (`.bin`) captured
from real hardware. Each log is a `VREC` container (see
`software/src/replay/RecordFormat.h`) holding raw inbound byte chunks; every
chunk is replayed through the **generated** NavLink v2 Python codec
(`navlink/generated/python/navlink_msgs.py`) so the decode path is identical to
live/replay and stays in lock-step with `dialect.json`.

## Tooling

- `parse_log.py` — decode a session, print a summary; `--csv DIR` dumps one CSV
  per message type.
- `make_plots.py` — generate the figures (matplotlib) into `<archive>/plots/`.
  Mermaid diagrams are inline in the markdown (render on GitHub).

```sh
python3 docs/log-analysis/parse_log.py /path/to/export.bin
python3 docs/log-analysis/parse_log.py /path/to/export.bin --csv docs/log-analysis/csv
python3 docs/log-analysis/make_plots.py docs/log-analysis/<archive-dir>
```

Generated `csv/` dumps and stray `.bin` files are git-ignored (regenerable /
bulky); the curated `*/export-*.bin` archived alongside each analysis is kept as
the source of truth so old logs stay re-analysable.

## Archives

Each capture gets its own dated folder (`YYYYMMDD-hhmmss`, from the log's own
timestamp) holding the raw `.bin` plus all analysis of it.

- [`20260617-124210/`](20260617-124210/) — **2026-06-17 12:42:10**, first
  hardware bring-up **bench-rig** session: 6.25 min, 38 k frames, 0 CRC errors.
  Never reached IN_AIR; frame hand-spun in yaw and rested tilted. Real tilt with
  low roll/pitch control authority, yaw 36× stronger, EKF roll/pitch accurate but
  yaw untrustworthy (uncalibrated mag), kernel healthy (control FIFOs 0 drops;
  telemetry-only saturation), 5 telemetry data-quality bugs.
  - [`README`](20260617-124210/README.md) — provenance, header, inventory, raw findings log
  - [`session-analysis.md`](20260617-124210/session-analysis.md) — full session overview
  - [`control-loop-analysis.md`](20260617-124210/control-loop-analysis.md) — cascade controller
  - [`motor-analysis.md`](20260617-124210/motor-analysis.md) — quad-X mixer & balance
  - [`kernel-analysis.md`](20260617-124210/kernel-analysis.md) — vaios RTOS health
  - [`sensor-analysis.md`](20260617-124210/sensor-analysis.md) — sensor cal & fusion (EKF)
