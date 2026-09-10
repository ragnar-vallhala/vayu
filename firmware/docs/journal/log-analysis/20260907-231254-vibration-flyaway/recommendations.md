# Recommendations

Ordered by what actually unblocks flight. One root cause drives findings 1–4, so
the mechanical item is not merely first — the rest are largely wasted until it
is done.

## P0 — do not fly until these are addressed

### 1. Fix the vibration (mechanical)

Nothing in software recovers information from a sensor reading 2.5 m/s² where
9.81 is the answer. Filtering cannot undo rectification: it happens in the analog
front end and the ADC, before any digital filter sees a sample. This is why PX4
*detects* the condition and stops trusting the sensor rather than cleaning it up.

- Soft-mount the FC (gel or foam, not rigid standoffs).
- Inspect every prop for nicks; balance them.
- Spin each motor by hand and feel for bearing roughness.
- **Check the mount refitted on 2026-09-06.** That change sits between "hovered
  nicely" and this session, and is the most likely single variable.

**Acceptance test:** clamped, stationary, throttle stepped 0 → 0.2 → 0.4 → 0.6,
watching `|accel|` from `IMU_RAW`. It must stay near 9.81. `accel_bias` is a
useful secondary read-out but only while the craft is *steady* — during real
acceleration the ToF aiding transiently over-drives it.

### 2. Airmode as an interim guard

One line, `angle_rate_controller.c:146`:

```c
static mixer_airmode_t s_airmode = MIXER_AIRMODE_DISABLED;
```

With airmode off, a saturating attitude loop *loses roll/pitch authority* instead
of *gaining collective*. That is a worse handling failure and a much better
safety failure — the craft gets sloppy rather than climbing away from the pilot.
It removes the Finding 2 mechanism outright while the mechanics are fixed.

Note the [`20260627-200523-idle-floor-015`](../20260627-200523-idle-floor-015/README.md)
session ran deliberately with airmode disabled for exactly this
attributability reason; this is not a novel configuration.

## P1 — makes the next session diagnosable

### 3. Populate the attitude body-rate fields

`ATTITUDE.rollspeed/pitchspeed/yawspeed` are never written (Finding 8). Filling
them turns the gyro from 12 inferred samples into a 50 Hz stream. On a vibration
campaign this is the highest-value change in the list after the mechanics.

### 4. ToF device-status whitelist

Accept status 11 only, in `vl53l0x_decode_and_publish()`
(`vl53l0x.c:100`). Today the 20 mm failure sentinel is rejected only because it
coincidentally falls below the 30 mm value gate — a coincidence, not a check.
The driver's own comment requests this once bench data exists; Finding 4 is that
data.

### 5. Widen `PerfFifo.drops`, or report a rate

u16 saturates at 65535 and two FIFOs are pegged, so the field cannot express
what it is for.

## P2 — after the airframe is quiet

### 6. Evaluate the FFT notch on the bench, not in the air

Now compiled in and forced on (`VAYU_GYRO_NOTCH_FORCE_ON`, flashed
2026-09-07 23:41). It addresses the **gyro** half of Finding 1 only — the rate
loop chasing phantom rates — and cannot touch the accelerometer.

Props on, craft strapped down, motors stepped with the notch off then on;
compare gyro spread and `|accel|`. If `|accel|` is still ~2.5 with the notch
running, the mechanical problem is untouched and the vertical estimate stays
dead regardless.

A 74% loss of gravity suggests broadband energy rather than one clean tone, and
a notch tracks peaks — three slots per axis. Expect it to take the edge off, not
to solve this.

### 7. Re-size the accel-bias clamp

`VERT_ACCEL_BIAS_MAX` is 3.0 m/s², chosen as ~3× a props-off measurement of
−0.7. The real props-on error is ~−7. Do **not** simply raise it: a larger clamp
would only let the estimator saturate more slowly on a sensor that has stopped
measuring. Re-derive it from the P0 acceptance test, once `|accel|` is sane.

## Retired

**"The telemetry loss is the RF link."** Concluded on 2026-09-06 from
rate-independent loss (13.3% at both 1 and 50 ping/s) with 1 → 82 ms RTT jitter.
It was a **flaky ESP VCC jumper**: the two captures here bracket the fix and show
26.9% → 0.7% with no firmware or RF change. An intermittent supply produces that
same signature. Check power before
suspecting the radio.

## Not diagnosed

- `ipc_timeouts` 18,404 / 14,357 with ~670 k / 521 k blocked takes. No baseline
  exists to say whether this is normal queue-wait behaviour or a task starving.
- Vertical estimator at 19.6% CPU (782 µs peak at 250 Hz) on an F401.
