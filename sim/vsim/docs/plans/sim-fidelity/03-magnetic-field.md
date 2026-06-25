# Plan: configurable magnetic field

Status: 🔴 planned. Gap doc Tier-1 #5. The mockup lets the operator set the
**ambient field** (total intensity / declination / inclination) and shows the
resulting N/E/D vector; the sim currently models magnetometer *sensor noise* but
assumes a fixed/implicit field. This plan makes the true field a configurable world
property so the simulated mag reading is `field + noise`, and heading/yaw estimation
can be exercised under a realistic, location-specific field.

Smaller and more self-contained than wind/atmosphere — no force-model change, just
the field the mag sensor reports.

## Model

The geomagnetic field is given by three human-friendly numbers (mockup `magMag`,
`magDec`, `magInc`); convert to an NED vector (`app.js` `updateMagVec()` ~1334):

```
h = B_total · cos(inclination)        // horizontal magnitude
N = h · cos(declination)
E = h · sin(declination)
D = B_total · sin(inclination)
B_world = (N, E, D)                    // µT, NED
```

The simulated magnetometer reading is the **body-frame** projection of this field
plus the existing sensor noise:

```
B_body = R_world2body · B_world
mag_reading = B_body + noise(mag_sigma, mag_bias_clip)   // existing noise pipeline
```

This replaces whatever constant the mag sensor currently emits. `R_world2body` is the
transpose of the body→world rotation already computed in `derive()` — the attitude is
right there, so projecting the field is cheap.

Default to a representative field (e.g. total ≈ 48 µT, dec ≈ 0°, inc ≈ 60° — generic
mid-latitude) so existing behaviour is sensible if never configured.

## Protocol

`sim/vsim/include/vsim_proto.h`:

```c
VSIM_CTL_SET_MAGFIELD = 16,       // body: vsim_ctl_magfield_t
```

```c
typedef struct {
    float total_uT;       // B_total            (magMag)
    float declination;    // deg                (magDec)
    float inclination;    // deg                (magInc)
} vsim_ctl_magfield_t;    // 12 B
```

> Design choice: send the three human inputs and convert to the NED vector
> **inside the daemon** (single source of truth for the N/E/D math), rather than
> precomputing in the GCS. The GCS still shows `magVec` for the operator, computed
> the same way for display only.

## Daemon

`sim/vsim/src/main.cpp`, new `case` beside `SET_NOISE` (~line 388):

```c
case VSIM_CTL_SET_MAGFIELD: {
    vsim_ctl_magfield_t m; std::memcpy(&m, cmd.body, sizeof(m));
    ctl.setMagField(m);   // converts to B_world NED, stores on sensor model
    std::fprintf(stderr, "vsim_d: magfield %.1fuT dec=%.1f inc=%.1f\n",
                 m.total_uT, m.declination, m.inclination);
    break;
}
```

The conversion (deg→rad, the `h/N/E/D` formulas) lives in the setter; store
`B_world` on whatever component synthesises the mag reading (the same place
`vsim_ctl_noise_t.mag_*` is consumed).

## Sensor pipeline integration

Find where the magnetometer sample is produced (the consumer of `mag_sigma` /
`mag_enable` set by `SET_NOISE` — see the sensor synthesis path used by the firmware
IMU shim / `/tmp/vsim_imu`). The change:

1. Project `B_world` into body frame using current attitude.
2. Add the existing mag noise/bias.
3. When `mag_enable == 0`, keep current "zero the channel" behaviour.

No physics-force change; this only affects the reported sensor value.

## GCS — SimWorker & UI

`SimWorker`:

```cpp
void sendMagField(float total_uT, float decDeg, float incDeg);   // VSIM_CTL_SET_MAGFIELD
```

`SimulatorWidget` — add a "Magnetic Field" group to the **sensor/vehicle** panel
(near `buildSensorPanel()` ~947), mirroring the mockup:

| Mockup id | Control | Field |
|-----------|---------|-------|
| `magMag` | `QDoubleSpinBox` (µT) | `total_uT` |
| `magDec` | `QDoubleSpinBox` (deg) | `declination` |
| `magInc` | `QDoubleSpinBox` (deg) | `inclination` |
| `magVec` | read-only label | computed N/E/D (display) |

Wire to a `pushMagField()` slot (hot group); update the `magVec` label locally with
the same N/E/D math for immediate operator feedback.

## Tests

- **Unit:** the N/E/D conversion against known values — `inc=90°` → field straight
  down (`D = B_total`, `N = E = 0`); `dec=90°, inc=0°` → field due East
  (`E = B_total`). Verify the body projection at level attitude equals `B_world`, and
  that a 90° yaw rotates N↔E correctly.
- **Manual SITL:** set a strong declination, yaw the drone on the rig, confirm the
  mag reading rotates consistently with heading; toggle `mag_enable` and confirm the
  channel zeroes.

## Effort

~0.5 day. Pure data path: three numbers → NED vector → body projection in the
sensor synth, plus a small UI group. No force model touched.
