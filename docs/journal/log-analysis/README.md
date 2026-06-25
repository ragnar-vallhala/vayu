# Flight-log analysis

Decoded analyses of recorded NavLink v2 telemetry sessions (`.bin`) captured
from real hardware. Each log is a `VREC` container (see
`navigator/src/replay/RecordFormat.h`) holding raw inbound byte chunks; every
chunk is replayed through the **generated** NavLink v2 Python codec
(`navlink/generated/python/navlink_msgs.py`) so the decode path is identical to
live/replay and stays in lock-step with `dialect.json`.

## Tooling

- `parse_log.py` — decode a session, print a summary; `--csv DIR` dumps one CSV
  per message type.
- `make_plots.py` — generate the figures (matplotlib) into `<archive>/plots/`.
  Mermaid diagrams are inline in the markdown (render on GitHub).
- `build_pdf.py` — combine an archive's docs into one printable PDF (renders the
  mermaid diagrams via `mmdc`, lays out with `pandoc`+`wkhtmltopdf`).

```sh
python3 docs/journal/log-analysis/parse_log.py /path/to/export.bin
python3 docs/journal/log-analysis/parse_log.py /path/to/export.bin --csv docs/journal/log-analysis/csv
python3 docs/journal/log-analysis/make_plots.py docs/journal/log-analysis/<archive-dir>
python3 docs/journal/log-analysis/build_pdf.py docs/journal/log-analysis/<archive-dir>   # needs pandoc, wkhtmltopdf, mmdc
```

Note: `make_plots.py` (and the PDF it feeds) is **bespoke to the 2026-06-17
hardware session**. SITL captures get a session-specific `make_plots.py` inside
their own archive dir instead — see `20260620-053528/`.

Generated `csv/` dumps and stray `.bin` files are git-ignored (regenerable /
bulky); the curated `*/export-*.bin` archived alongside each analysis is kept as
the source of truth so old logs stay re-analysable.

## Archives

Each capture gets its own dated folder (`YYYYMMDD-hhmmss`, from the log's own
timestamp) holding the raw `.bin` plus all analysis of it.

- [`20260617-124210/`](20260617-124210/) — **2026-06-17 12:42:10**, first
  hardware bring-up **bench-rig** session: 6.25 min, 38 k frames, 0 CRC errors.
  Never reached IN_AIR. **Headline: the roll/pitch attitude loop is unstable —
  diverges into a ~0.3 Hz limit cycle on throttle (operator pulled throttle to
  stop it), likely rate-loop tuning (Kd=0).** Also: EKF roll/pitch accurate but
  yaw untrustworthy (uncalibrated mag), kernel healthy (control FIFOs 0 drops;
  telemetry-only saturation), 5 telemetry data-quality bugs.
  - [`README`](20260617-124210/README.md) — provenance, header, inventory, raw findings log
  - [`session-analysis.md`](20260617-124210/session-analysis.md) — full session overview
  - [`control-loop-analysis.md`](20260617-124210/control-loop-analysis.md) — cascade controller
  - [`motor-analysis.md`](20260617-124210/motor-analysis.md) — quad-X mixer & balance
  - [`kernel-analysis.md`](20260617-124210/kernel-analysis.md) — vaios RTOS health
  - [`sensor-analysis.md`](20260617-124210/sensor-analysis.md) — sensor cal & fusion (EKF)
  - [`recommendations.md`](20260617-124210/recommendations.md) — fixes + next-run capture plan (rig-only)

- [`20260620-053528/`](20260620-053528/) — **2026-06-20 05:35:28**, **SITL
  (simulator)** session: 100.9 s, 12.5 k frames, 0 CRC errors. **Headline: the
  roll/pitch inner rate loop does not track — commanded level, it answers the
  outer loop's corrective rates with near-zero output (corr 0.12 vs yaw's 0.95),
  never exceeding 13% authority even at 17.8° pitch error. Root cause: roll/pitch
  rate Kp=0.0005 (36× below yaw) with Kd=0** — the same untuned-rate-loop defect
  as the 2026-06-17 rig, reproduced deterministically. The new vertical estimator
  is healthy (fused-vs-baro RMS 0.25 m across a ±60 m/s, 547 m manual profile;
  no alt-hold loop in this build).
  - [`README`](20260620-053528/README.md) — provenance, header, inventory, raw findings log
  - [`session-analysis.md`](20260620-053528/session-analysis.md) — overview & timeline
  - [`control-loop-analysis.md`](20260620-053528/control-loop-analysis.md) — **the headline**: cascade tracking & the 36× gain gap
  - [`vertical-analysis.md`](20260620-053528/vertical-analysis.md) — vertical estimator & flight-phase health
  - [`recommendations.md`](20260620-053528/recommendations.md) — fixes + next-capture plan

- [`20260621-021352/`](20260621-021352/) — **2026-06-21 02:13**, **SITL
  (simulator)** — the **first dual-log archive**, pairing the FC telemetry
  (estimate) with the physics ground truth (`gt-*.bin`), captured *after* the
  `mag_fusion` producer fix. **Headline: against ground truth the estimator now
  matches truth on yaw and altitude; the one remaining gap is roll/pitch under
  sustained acceleration.** Yaw error **121° → 2.9°** (tracks the −134° heading
  swing — the `mag_fusion` SITL data bug is resolved); altitude RMS **0.12 m**;
  roll/pitch reads ~level while the craft is genuinely tilted up to 23° in
  free-flight drift (∝ horizontal speed) — the accelerometer gravity-vs-accel
  ambiguity, fixable only with GPS/optical-flow. Includes the status of every
  previously-detected fault.
  - [`README`](20260621-021352/README.md) — provenance, paired headers, fault-status table, raw findings
  - [`session-analysis.md`](20260621-021352/session-analysis.md) — the run, timeline, cross-run context
  - [`estimator-analysis.md`](20260621-021352/estimator-analysis.md) — **the headline**: estimate vs truth; yaw fix + roll/pitch limitation
  - [`recommendations.md`](20260621-021352/recommendations.md) — fault statuses + the velocity-aiding decision

- [`20260621-053142/`](20260621-053142/) — **2026-06-21 05:31**, **SITL
  (simulator)** dual-log, **aggressive** flight after raising `EKF_R_ACC_DIR`
  2.5e-3 → 2.5e-2 (trust the accelerometer less — the GPS-less PX4/ArduPilot
  approach). **Headline: the roll/pitch tilt under-report is essentially gone —
  with sticks centered the estimate and true tilt now agree to 0.15° (was 2.15°
  at the old accel trust), and the craft flies genuinely more level.** Yaw RMS
  0.32° (tracks a −108° swing) and altitude 0.04 m stay exact even at roll +60° /
  pitch −50° / 14.5 m/s. The residual is confined to *sustained* acceleration
  (the IMU-only floor); a separate 176 s flight confirmed no long-term drift from
  the change. The before/after to `021352`.
  - [`README`](20260621-053142/README.md) — provenance, paired headers, fault-status table, raw findings
  - [`session-analysis.md`](20260621-053142/session-analysis.md) — the run, timeline, before/after vs 021352
  - [`estimator-analysis.md`](20260621-053142/estimator-analysis.md) — **the headline**: the accel-trust effect + the residual
  - [`recommendations.md`](20260621-053142/recommendations.md) — fault statuses + the commit / real-HW decision
