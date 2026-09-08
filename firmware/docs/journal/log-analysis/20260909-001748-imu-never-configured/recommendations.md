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
2. **Re-check the rate-loop D term.** With the sensor at ~100 Hz the D term is
   90% zeros and 10% impulses (analysis §6c). Once the gyro is fresh at 1600 Hz
   this fixes itself — but the current Kd was tuned *against* the impulse train,
   so it is very likely wrong for a continuous derivative. Re-tune, or at least
   re-measure, before trusting it. The same applies to `*_D_LPF_RC = 0.004`.
3. **Re-check `LPF_GYR_ALPHA` / `LPF_ACC_ALPHA`** (0.51 / 0.34). Applied once
   per poll at 1827 Hz they sit near 300 Hz and 150 Hz, which did nothing to a
   98 Hz staircase. Against a real 1600 Hz stream they become live filters with
   real phase lag in the control path — pick them deliberately rather than
   inheriting them.
4. **Re-measure the vibration spectrum with props on.** For the first time
   there will be real content above 50 Hz. This is the capture the FFT notch
   needs, and it is the one thing this archive could not provide.
5. **Re-size `VERT_ACCEL_BIAS_MAX` (currently 3.0).** It was chosen against
   rectified data. With a sensor that reports true specific force the bias it
   must absorb should be far smaller, and a tighter clamp makes the veto a
   sharper instrument.
6. **Re-validate the FFT notch, or leave it off.** It has been force-enabled
   since 2026-09-07 and cannot have been doing anything useful. Consider
   flashing `firmware/build` (notch off) for the first post-fix run so the
   vibration is measured unfiltered.
7. **Reconsider `IMU_SAMPLE_FREQ_HZ = 2000`.** Once the sensor runs at 1600 Hz,
   polling at 1827 Hz is roughly right; today it is 19× oversampling.

## P1b — the other two drivers

Found by asking whether the BMX160 was the only device left on defaults. It was
not the only one with a problem, though it is the only one left on defaults by
accident.

1. **Retry the VL53L0X probe.** `vl53l0x_init()` runs once at boot, shares I2C1
   with the IMU, and on a failed model-ID read sets `_initialized = 0` forever.
   The recording shows the rangefinder publishing nothing at all in **9 of 20
   arms** — binary per power cycle, not throttle-related. Retry the probe a few
   times with a delay, and re-probe periodically while it is absent; the ToF
   velocity aiding added 2026-09-08 is unavailable on roughly half of boots as
   things stand. Surface the state in telemetry so it is visible without a card
   read.
2. **Consider tuning the VL53L0X.** Timing budget, signal-rate limit and VCSEL
   periods are all at ST defaults (~33 ms budget, ~1.2 m usable). The height
   mode switches to baro above 1.5 m, which is beyond where the default
   configuration is reliable — the two thresholds were never reconciled.
3. **Revisit `BME280_FILTER_OFF`.** The BME280 is otherwise configured
   correctly, but its IIR filter is off and `BME280_FILTER_OFF` is the only
   filter constant that exists in the header. Bosch recommends coefficient 16
   for altimetry against exactly the kind of transient a prop makes. Measured
   effective rate today is 13.0 Hz.

## P1c — the output side

Measured in analysis §6d. None of this is a "never configured" bug — the ESC
path is set up explicitly — but all of it was sized against the aliased plant.

1. **Re-measure mixer saturation after P0.** 43% of powered ticks currently have
   a motor at a rail, and airmode is modulating collective by 1.8x at low stick
   and 0.65x above 0.5. Much of that differential demand comes from a rate PID
   chasing an aliased gyro, so it should shrink on its own — check before
   changing mixer behaviour, or a real fix gets papered over by a tuning change.
2. **Reconsider `MOTOR_IDLE_FLOOR = 0.15`.** High for a 5" airframe (0.05-0.08
   is typical). It eats authority range at the bottom, and it spins the props
   hard enough to produce aliasing at zero stick — every idle arm in this
   capture shows motors at exactly 0.150.
3. **The ESC runs at 400 Hz PWM** while the rate loop runs at 1 kHz, so ~60% of
   computed commands are never delivered. Not urgent, but once the sensor is at
   1600 Hz the actuator becomes the slowest link in the chain. OneShot/DShot is
   the eventual answer; at minimum, know that the loop rate above 400 Hz buys
   nothing today.
4. **Do not read the FAILSAFEs as an accel-health problem.** The accel latch
   vetoes height mode and stops there; the failsafes are the 70 deg tilt cutoff
   firing after the airframe tips. Restrain the airframe for the next bench run
   so a tip does not truncate the capture.

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

- **Chasing a mechanical vibration fix first.** The run-to-run spread is now
  explained without invoking a mechanical fault: at matched throttle the
  *measured* vibration is identical across runs (`acc_sd` 1.01–1.36) while the
  loss varies 4×, so what differs is where the prop tone folds, not how much
  the airframe shakes (analysis §6b). There may still be a balance problem —
  but no mechanical measurement means anything while the sensor cannot see
  above 50 Hz. Fix P0, then measure.
- **Touching the vertical estimator.** It behaved correctly throughout: it
  absorbed the error, clamped, latched, and vetoed. Six FAILSAFEs are six
  successful refusals to fly on bad data.
