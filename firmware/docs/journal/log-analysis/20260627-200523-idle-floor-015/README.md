# Idle-floor 0.15 — real-drone capture (2026-06-27)

Single-variable test: does a real ESC min-spin idle remove the pitch limit cycle's
trigger? `MOTOR_IDLE_FLOOR` raised `0.005 → 0.15` (`firmware/include/variables.h:122`),
isolating the ESC-stall transport delay the [plant-ID](../20260625-233852-pitch-indi-campaign/plant_id/README.md)
fingered as the root of the ~2 Hz cycle.

## Firmware under test
- Base commit `2529522` **+ the uncommitted idle-floor change** (and the integrated
  mixer with `s_airmode = MIXER_AIRMODE_DISABLED`, so allocation behaves as before —
  idle floor is the only behavioral change vs the pitch-INDI campaign).
- Build/flash: `tools/scripts/flash.sh` (rebuilds from source → picks up the working-tree
  change → objcopy → st-flash; avoids the stale `main.bin` trap).

## What is NOT changed (so the result is attributable)
- **Airmode still OFF** — the mixer's saturation behavior is the legacy uniform scaler.
- No sim-twin calibration, no gain re-tune, no hardware change.

## Capture
Default rig: ESP/UDP NavLink-v2 bridge, GCS telemetry port 14555.
```sh
python3 tools/telemetry/udp_telem_sniff.py --raw-bin idle-floor-015.bin
#   (Ctrl-C to stop, or add --seconds N)
```
Decode + characterize afterwards:
```sh
python3 tools/telemetry/analyze_pitch_osc.py idle-floor-015.bin
```

## Hypothesis / what to look for
With the stall delay removed, the pitch axis should no longer floor a motor ~95 % of
the cycle, so expect the ~1.8–2.0 Hz pitch limit cycle to **soften or shift** (lower
peak power / amplitude), with roll still the calm reference. A *fully* stable hover is
NOT expected — the P1 airmode fix and the airframe asymmetry are still outstanding.

## Safety
0.15 idle = **all four props spin the instant you arm** (~15 % throttle, real current/
heat). Arm clear of hands; keep the first runs low / tethered if possible.
