# 20260626-020449-indi-k6-seed

~61 s open-air INDI SEED (k=6, lpf=0.010) flight — first run of the retuned seed.

Navigator recording, open air. Source: `~/vayu-logs/export-20260626-020449.bin`.

## Headline

SEED WORKS: the 3.3 Hz hunt COLLAPSED — strongest peak now 0.11 (flat spectrum), pitch RMS down ~40% vs k=20. Soft/sluggish (pitch drooped to -82 deg) but no sustained cycle, and u barely saturates (4%) -> calm enough for a CLEAN sysid b-fit. Marginally-flyable seed achieved.

See `setup.json` for full controller config + metrics. Decode: `python3 tools/telemetry/analyze_pitch_osc.py indi-k6-seed.bin`.
