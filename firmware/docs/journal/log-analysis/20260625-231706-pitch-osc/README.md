# Pitch cascade oscillation — 2026-06-25 23:17

Live raw capture of the violent pitch oscillation that appeared right after
applying the sysid-designed pitch gains (rate + `angle_kp 4.14`) from the
0.25-throttle chirp earlier this session. Recorded off the real FC over the
ESP/UDP bridge (`10.42.0.30:14555`).

## Report (start here)

| doc | covers |
|---|---|
| [`pitch-oscillation-analysis.md`](pitch-oscillation-analysis.md) | the diagnosis — 1.5 Hz cascade from too-fast `angle_kp`, evidence, the fix |

## Headline

`angle_kp = 4.14` is too fast for the inner rate loop → **1.50 Hz cascade limit
cycle**, pitch ±~50° (102° pk-pk) with stick centred, output railed ±1.0 for
19 % of samples. Same mode `freeflight_tune.json` fixed by softening the outer
loop to 1.0. **Keep the sysid rate gains; drop `angle_kp` to ~1.0–2.0.**

## Data

| file | capture | size |
|---|---|---|
| `pitch-osc.bin` | armed, oscillating, 90 s | ~1.0 MB, 11409 datagrams, 0 % loss |

Raw VREC container (`navigator/src/replay/RecordFormat.h`) of every inbound
datagram, recorded with the sniffer's `--raw-bin`:

```sh
python3 tools/telemetry/udp_telem_sniff.py --seconds 90 \
    --raw-bin docs/journal/log-analysis/20260625-231706-pitch-osc/pitch-osc.bin
```

Regenerate the figures from the `.bin`:
```sh
python3 docs/journal/log-analysis/20260625-231706-pitch-osc/make_plots.py
```

- `plots/01_pitch_cascade.png` — limit cycle (full + zoom) + saturating `pitch_out`
- `plots/02_pitch_spectrum.png` — pitch-angle spectrum, 1.5 Hz peak
