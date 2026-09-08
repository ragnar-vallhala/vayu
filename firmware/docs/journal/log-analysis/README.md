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

- [`20260909-001748-imu-never-configured/`](20260909-001748-imu-never-configured/) —
  **2026-09-09 00:17**, real drone, 20 bench arms / 136.7 s, the FIRST archive
  from the high-speed IMU-to-SD recorder (`storage/imu_hs_log`) rather than
  telemetry — 248 k samples at the full sensor path. **Headline: the BMX160 has
  never been configured.** `bmx160_init()` only ever *reads* its config back, so
  the chip has run on power-on defaults forever: **~100 Hz ODR** (Nyquist 50 Hz,
  so all prop energy aliases) and **±2 g range** (gravity alone is 50% of full
  scale). Measured gravity collapses 10.2 → 4.8 m/s² with throttle on a
  stationary bench; the vertical estimator's bias clamps at −3.0 and vetoes, and
  6 of 20 arms end in FAILSAFE. Explains the 2026-09-07 flyaway's collapsing
  `|accel|`, and why the FFT notch can never have worked.
  - [`README`](20260909-001748-imu-never-configured/README.md) — provenance, inventory, headline
  - [`analysis.md`](20260909-001748-imu-never-configured/analysis.md) — eight findings, per-arm table
  - [`recommendations.md`](20260909-001748-imu-never-configured/recommendations.md) — P0 blocks flying; the enum trap in fixing it

- [`20260907-231254-vibration-flyaway/`](20260907-231254-vibration-flyaway/) —
  **2026-09-07 23:12 + 23:17**, real drone, two bench/lift captures the evening
  after the first hover: 124 s, 18 k frames. Never reached IN_AIR (the detector
  keys off `climb_rate`, which read −9.7 m/s). **Headline: with the props
  turning the accelerometer reads 2.5 m/s² where 9.81 is the answer — it loses
  three quarters of gravity.** That single mechanical fault destroys the vertical
  estimate through the accel path and, through the gyro path, saturates the rate
  loop where `MIXER_AIRMODE_RP` turns saturation into collective: stick down
  0.54 → 0.38 while mean motor command went up 0.44 → 0.64 — an **uncommanded
  climb**. The `accel_unhealthy` veto caught the half it could see and kept the
  height mode from engaging. Control FIFOs 0 drops, kernel healthy. Also: the FFT
  notch was compiled out of the flown build; the attitude body-rate fields are
  never populated; the ESP VCC jumper (not RF) was the telemetry loss.
  - [`README`](20260907-231254-vibration-flyaway/README.md) — provenance, inventory, headline
  - [`analysis.md`](20260907-231254-vibration-flyaway/analysis.md) — ten findings by severity
  - [`recommendations.md`](20260907-231254-vibration-flyaway/recommendations.md) — fix order, and what not to try

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

- [`20260625-230911-mag-spread/`](20260625-230911-mag-spread/) — **2026-06-25
  23:09**, **real FC** magnetometer spread spot-check: a 45 s dense hand rotation
  (2240 pts @50 Hz, 0 % loss) reduced to the XY/YZ/XZ spread, mean, and per-sample
  % off the mean field. **Headline: mag stays heading-grade — `‖m‖` 40.5 µT at
  CoV 4.95 %, LS-sphere hard-iron residual 0.41 µT (centred on origin).** The
  sample-mean centre lands ~19 µT off origin (coverage bias, not a fault — judge
  by the LS fit). Verifies the `20260625-032526-imu-calib-verify` calibration
  from a fresh capture; no re-cal needed.
  - [`README`](20260625-230911-mag-spread/README.md) — provenance, data, plot list
  - [`mag-analysis.md`](20260625-230911-mag-spread/mag-analysis.md) — spread/mean/%-off, LS-vs-mean centre, method

- [`20260625-231706-pitch-osc/`](20260625-231706-pitch-osc/) — **2026-06-25
  23:17**, **real FC** raw capture of the violent pitch oscillation right after
  applying the sysid-designed pitch gains. **Headline: `angle_kp = 4.14` (sysid
  `0.25·wc`) is too fast for the inner rate loop → a 1.50 Hz cascade limit cycle
  — pitch ±~50° (102° pk-pk) with stick centred, `pitch_out` railed ±1.0 for
  19 % of samples.** The same mode `freeflight_tune.json` fixed by softening the
  outer loop to 1.0; the rate gains are fine. Fix: keep sysid rate gains, drop
  `angle_kp` to ~1.0–2.0 (and cap it in `sysid_fit.design_gains`).
  - [`README`](20260625-231706-pitch-osc/README.md) — provenance, data, plot list
  - [`pitch-oscillation-analysis.md`](20260625-231706-pitch-osc/pitch-oscillation-analysis.md) — the 1.5 Hz cascade diagnosis + fix

- [`20260625-233852-pitch-verify/`](20260625-233852-pitch-indi-campaign/20260625-233852-pitch-verify/) — **2026-06-25
  23:38**, **real FC** verification after the wc-capped pitch tune v3. **Headline:
  NOT a tuning problem — across 3 gain sets (rate_kp 0.005–0.012, angle_kp
  1.75–4.14) pitch limit-cycles at 1.5–2.6 Hz while roll is rock-stable, and
  pitch_out saturates 19→37 % (gain-invariant ⇒ authority-limited).** Front motors
  run ~20 % harder than back (pitch imbalance); pitch sysid K=1381 is 2.45× roll's
  563. Points to a physical pitch-axis fault (CG forward / burned motor-ESC / prop).
  Fix the hardware, then re-sysid.
  - [`README`](20260625-233852-pitch-indi-campaign/20260625-233852-pitch-verify/README.md) — verdict, gain-sweep table, motor asymmetry, causes
