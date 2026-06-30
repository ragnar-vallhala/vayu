# 20260626-014848-pid-openair

~42 s open-air PID flight (reverted from INDI). Direct same-conditions comparator to the INDI runs.

Navigator recording, open air. Source: `~/vayu-logs/export-20260626-014848.bin`.

## Headline

PID open-air: SHARP 1.95 Hz pitch limit cycle (rel power 1.0), pitch RMS 85->155 as throttle rises. Worse than INDI at matched throttle; the reproducible PID limit cycle.

See `setup.json` for full controller config + metrics. Decode: `python3 tools/telemetry/analyze_pitch_osc.py pid-openair.bin`.
