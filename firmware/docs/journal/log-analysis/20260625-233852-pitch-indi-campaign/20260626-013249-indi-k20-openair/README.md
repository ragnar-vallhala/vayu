# 20260626-013249-indi-k20-openair

~10 min open-air piloted INDI (k=20) flight (angle_sp up to 100 deg; mostly grounded between brief flights, lossy long session ~26 Hz effective).

Navigator recording, open air. Source: `~/vayu-logs/export-20260626-013249.bin`.

## Headline

INDI k=20 hunts broadband at ~3.3 Hz = its own bandwidth. ~2x tighter than PID at matched throttle but the hunt is the bandwidth being too hot. Drove the k=6 seed retune.

See `setup.json` for full controller config + metrics. Decode: `python3 tools/telemetry/analyze_pitch_osc.py indi-k20-openair.bin`.
