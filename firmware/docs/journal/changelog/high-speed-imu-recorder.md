# High-speed IMU recorder (HSL) — and what it found on day one

Deep-dive changelog for `storage/imu_hs_log`: recording the IMU to SD at the
full sensor path rather than through the 50 Hz telemetry link, why it exists,
the on-disk format, and the sequence of things it exposed within a day of
existing. Newest first.

Format spec: [`firmware/include/storage/imu_hs_log.h`](../../../include/storage/imu_hs_log.h).
Decoder: [`tools/telemetry/hslog.py`](../../../../tools/telemetry/hslog.py).
First archive and its findings:
[`log-analysis/20260909-001748-imu-never-configured/`](https://github.com/ragnar-vallhala/vayu-logs/blob/main/20260909-001748-imu-never-configured).

---

## 2026-09-09 — What the recorder found

Written up in full in the vayu-logs session linked above; summarised here because
it is the reason the recorder was worth building.

**The BMX160 has never been configured.** `bmx160_init()` only ever *reads* its
configuration back off the chip (`bmx160.c:697-712`); the register-writing path
exists at `bmx160.c:780-833` and is called from nowhere. The sensor has run on
power-on defaults for the life of the project — **~98 Hz ODR** (measured) and
**±2 g range**. Nyquist is 50 Hz, so every prop and blade-pass frequency folds
into the control band.

Consequences, all measured rather than inferred:

- On a stationary bench, `|accel|` collapses 10.2 → 4.8 m/s² as throttle rises.
  It is **folding, not clipping**: at matched motor command `acc_sd` is flat
  (1.01–1.36) while the loss varies 4×, the loss is non-monotonic in throttle,
  and peaks reach only 38% of the int16 rail inside the collapse.
- The vertical estimator absorbs the error into `accel_bias`, clamps at −3.0,
  latches `accel_unhealthy` and vetoes height mode — correctly, in 6 of 20 arms.
  The failsafes themselves are the 70° tilt cutoff, not the veto.
- The rate loop's D term is **90% zeros and 10% impulses** (2.3× rms, 5.0× peak
  inflation) because it differentiates a 98 Hz staircase over a 1 ms `dt`.
- The FFT notch cannot work — it hunts to 450 Hz in a signal with nothing real
  above 50.
- Every gain, filter cutoff and notch band in the tree was fitted against this.

Two more, from widening the same lens:

- **The VL53L0X publishes nothing on ~half of boots.** `vl53l0x_init()` probes
  the model ID once over I2C1 shared with the IMU and, on failure, sets
  `_initialized = 0` permanently. 9 of 20 arms had no rangefinder at all.
- **43% of powered ticks have a motor at a rail**, and airmode is delivering
  1.80× the commanded collective below 0.20 stick — the 2026-09-07 flyaway's
  uncommanded-climb mechanism, measured directly.

### Fixed

- **Calibration could not start.** `task_create(calibration_task, ..., 8192, 0)`
  asked for an 8 KB stack from a 40,960 B heap whose live peak is 34,456 B, so
  the allocation failed and tripped `v_panic` in `task.c:197`. Measured the
  actual requirement with `-fstack-usage` along the real call graph —
  `calibration_task(400) → calib_engine_run(104) → run_ellipsoid(488) →
  calib_fit_ellipsoid(272)` = 1264 B, plus ~200 B of exception frame — and
  sized it 3072. Also: a NULL `cal_args` was previously handed to the task
  anyway, and a failed TCB alloc (`task_create` returns 0) was swallowed
  silently. Both now back out and report. (`src/comm/comm_processor.c`.)

  What consumed the headroom was the FFT notch: it allocates 6,832 B of FFT
  buffers per boot, and the 27,624 B heap peak recorded on 2026-09-07 was a
  `VAYU_FFT_NOTCH=OFF` build.

### Considered and rejected

- **Draining the motor queue.** `motor_task` reads one item per 2 ms tick from a
  queue the rate loop fills at 1 kHz; `SPSC_POLICY_OVERWRITE` drops the oldest
  while the consumer reads from the tail, so the ring sits permanently full and
  every ESC command is ~1–2 ms stale. Changing the read to a drain loop was
  tried and **reverted**: it makes the consumer's per-tick work depend on the
  producer's rate, coupling two tasks that are independent today, and a bounded
  fixed-cost consumer is worth more than 1–2 ms. If it is ever worth removing,
  the shape is a single-slot latest-value cell, not a loop over a FIFO.

- **Speeding `motor_task` up.** At `v_delay(2)` it already writes CCR more often
  than the 400 Hz timer latches it. Output-compare preload is enabled
  (`CCMR1 |= TIMx_CCMRy_OCxPE`, timer.c:541) so mid-period writes are
  glitch-free, but they still only take effect once per 2.5 ms. The protocol
  binds, not the task rate — and DMA changes none of it, since a motor command
  is a single write to `TIM1->CCRx`. OneShot125 is the cheap protocol step;
  bidirectional DShot would return real RPM and retire the notch problem.

### Changed

- **SPSC rings sized for how they are actually drained.** Telemetry rings are
  mailboxes — a 1 kHz producer against a consumer that pops one per gate tick —
  so depth only aged the sample that got read. Control rings are drained with
  `while (pop())`. Split the two: telemetry 9 → 2 usable, control 9 → 4 (2× the
  worst peak ever measured, 2 of 9 with zero drops). `bss 38896 → 35576`,
  −3320 B, which is what paid for the recorder's buffers.

### Added

- **`storage/imu_hs_log`** — three streams into a wrapping 32 MB ring on the
  card, armed time only: `imu` (raw i16 counts, pre-LPF/bias/remap, full sensor
  rate), `act` (4 motors + collective setpoint, ~333 Hz), `vrt` (the vertical
  estimator's own view, 20 Hz), plus sparse `EVENT` frames for arm/disarm, mode,
  accel-health and notch changes. ~27 KB/s measured, **zero dropped sectors**
  over 136.7 s on hardware.

- **The `HSL1` format.** Self-delimiting frames, one 512 B sector each, unknown
  types skipped by `len`. Ordering is by a per-frame `seq`, never by slot, so a
  wrapped ring, a partially-filled first lap and reboot gaps all decode without
  special cases. The cluster chain is committed at boot via an `f_lseek` past
  EOF (which `create_chain`s without writing) rather than `vfs_preallocate`'s
  zero-fill — ~8 FAT sector writes instead of 65,536 data writes — so no FAT
  work happens mid-stream and a power cut cannot cost the directory entry.

- **`HSL_STATUS`** (msgid 1048, 1 Hz) and a Navigator status-bar readout, so the
  recorder can be checked from the ground instead of by pulling the card.
  `dropped_sectors` is the field that matters in flight.

- **`tools/telemetry/hslog.py`** — decoder, per-arm summary, FFT and CSV export,
  with `--selftest` for the format contract and a `hslog_decode` ctest that
  decodes bytes the real C encoder wrote. `sim/host/tests/test_hslog.c` drives
  the encoder through a genuine ring wrap, an overwritten `SESSION` frame and a
  simulated reboot.

- **`tools/telemetry/hslog_pull.py`** — bounded link download using `HSL_STATUS`
  to learn the live length (a plain download would transfer all 32 MB).
  **Written but never exercised** — the card was the workflow on the day.

### Known gaps

- `s_session` is not carried across boots — it is the one cursor field missing
  from the file header beside `head_slot`/`next_seq`/`wraps` — so ids restart at
  1 each power cycle. The decoder copes by splitting on the `SESSION` frame, tag
  change *and* seq gap, but an arm whose `SESSION` frame was overwritten *and*
  which shares a tag with its predecessor cannot be separated.
- `SESSION.unix_ms` is only a real wall clock if a GCS sent `TIME_SYNC` that
  power cycle; 1 of 20 arms had one.
- The `act` stream declares 400 Hz but is fed once per 1 kHz rate-loop tick, so
  it lands on ~333 Hz. The decoder reports measured rates, so nothing downstream
  is wrong — only the declaration is misleading.
- The recording is ~19× redundant while the sensor runs at 98 Hz: 248 k samples
  hold about 13 k independent ones.
