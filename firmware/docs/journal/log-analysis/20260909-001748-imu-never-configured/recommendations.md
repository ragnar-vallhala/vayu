# Recommendations

Ordered. P0 blocks tomorrow's run.

## P0 — configure the IMU at boot

Nothing else on this list matters until the sensor is producing real data.
`bmx160_init()` must write `ACC_CONF`, `ACC_RANGE`, `GYR_CONF` and `GYR_RANGE`
instead of reading them back. The packing helpers already exist
(`bmx160.c:780-833`); they are simply never called.

Targets, matching what every other autopilot uses on a small multirotor:

| register | set to | why |
|---|---|---|
| `ACC_CONF` | odr = 1600 Hz, bwp = normal (OSR2), us = 0 | above prop + blade-pass, so the anti-alias filter has something to work with |
| `ACC_RANGE` | ±16 g (**code 12**) | gravity drops from 50% to 6.5% of full scale; leaves ~15 g of vibration headroom |
| `GYR_CONF` | odr = 1600 Hz (or 3200) | same aliasing argument; the rate loop is the consumer |
| `GYR_RANGE` | ±2000 dps (already the default) | unchanged, has ample headroom |

### The trap in doing this

`bmx160_acc_range_t` holds **g-values, not register codes**:

```c
BMX160_ACC_16G = 16
...
static uint8_t bmx160_get_acc_range(bmx160_config_t *config) {
  uint8_t val = (config->bmx160_acc_range & 15U);   // 16 & 15 == 0  -- invalid
```

while the *getter* already speaks codes (`bmx160_range_code_to_g`: 3→2 g,
5→4 g, 8→8 g, 12→16 g). Setting `BMX160_ACC_16G` today writes `0` to
`ACC_RANGE` and silently does the wrong thing. **Fix the enum to carry register
codes** (2 g→3, 4 g→5, 8 g→8, 16 g→12) so setter and getter agree, or the first
attempt at this fix will look like it worked and won't have.

Verify on hardware by reading the scale straight out of the next recording's
`FMT` frame: `scale × 32768 / 9.80665` must read 16.00, and the effective
new-data rate (`hslog.py`, repeat-run length) must stop being ~97 Hz.

## P1 — re-check everything that was tuned against aliased data

Do these *after* P0 and a fresh capture, not before.

1. **Re-run accel calibration.** The stored offsets and soft-iron in `cal.bin`
   were fitted against a ±2 g sensor at 100 Hz. They are in m/s² so they carry
   over numerically, but they describe a different instrument.
2. **Re-measure the vibration spectrum with props on.** For the first time
   there will be real content above 50 Hz. This is the capture the FFT notch
   needs, and it is the one thing this archive could not provide.
3. **Re-size `VERT_ACCEL_BIAS_MAX` (currently 3.0).** It was chosen against
   rectified data. With a sensor that reports true specific force the bias it
   must absorb should be far smaller, and a tighter clamp makes the veto a
   sharper instrument.
4. **Re-validate the FFT notch, or leave it off.** It has been force-enabled
   since 2026-09-07 and cannot have been doing anything useful. Consider
   flashing `firmware/build` (notch off) for the first post-fix run so the
   vibration is measured unfiltered.
5. **Reconsider `IMU_SAMPLE_FREQ_HZ = 2000`.** Once the sensor runs at 1600 Hz,
   polling at 1827 Hz is roughly right; today it is 19× oversampling.

## P2 — recorder and tooling

1. **Persist `s_session` in the HSL file header.** It is the one cursor field
   not recovered at boot (`head_slot`, `next_seq`, `wraps` all are), so ids
   restart at 1 every power cycle and the block session-tag aliases
   immediately instead of after 256 arms. `imu_hs_log_boot_init()` already
   reads the header; add the field beside the others.
2. **Keep Navigator attached during runs.** One `TIME_SYNC` per power cycle is
   all it takes for every `SESSION` frame to carry a real wall clock instead of
   uptime.
3. **`act` stream nominal rate is unreachable.** It declares 400 Hz but is fed
   once per 1 kHz rate-loop iteration, so it lands on 1000/3 = 333 Hz. Either
   declare 333 or decimate to 250; the decoder reports measured rates so
   nothing is wrong today, only misleading.

## Not doing

- **Chasing a mechanical vibration fix first.** There may well be a balance or
  mount problem — run 10 reached 0.98 throttle with only −2.15 of bias while
  run 3 clamped at 0.49, which does not scale with throttle and is worth
  understanding. But no mechanical measurement means anything while the sensor
  cannot see above 50 Hz.
- **Touching the vertical estimator.** It behaved correctly throughout: it
  absorbed the error, clamped, latched, and vetoed. Six FAILSAFEs are six
  successful refusals to fly on bad data.
