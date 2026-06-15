# Plan: advanced sensor error models

Status: 🔴 planned. Gap doc Tier-1 #7. Today the sim models only white-noise σ +
bias-clip + enable per sensor (`vsim_ctl_noise_t`, consumed in the `SET_NOISE`
handler). The mockup specifies a much richer per-sensor error stack —
**scale-factor, saturation, spikes/outliers, vibration (blade-pass ∝ throttle),
latency, temperature-dependent bias**, plus a Gaussian-vs-CSV noise **source
selector** and an editable per-sensor rate. This is the biggest **sim-to-hardware
transfer fidelity** win: a controller that survives only clean sim IMU data is not
validated against real MEMS pathologies.

Scope here covers the **accelerometer, gyroscope, magnetometer** (the sensors the
sim actually synthesises today). Barometer/GPS error models are deferred with their
sensors (see folder README).

## Error stack (applied per sensor, in order)

Each sensor's raw value `x` passes through a pipeline; every stage is independently
toggleable so it degrades gracefully to today's behaviour (all off → just σ+bias):

```
1. scale        x ← x · scale            (per-axis scale-factor error, mockup [1.000,1.000,1.000])
2. temp bias    x ← x + temp_coeff · (T − T0)     (drift vs simulated temperature)
3. vibration    x ← x + vib_amp · throttle · sin(2π f_blade t)   (blade-pass, ∝ throttle)
4. white noise  x ← x + N(0, σ)          (existing)
5. bias walk    b ← clip(b + N(0,σ_b)·dt, ±bias_clip);  x ← x + b   (existing random walk)
6. spikes       with prob p_spike per sample: x ← x + spike_mag·sign  (outliers)
7. saturation   x ← clamp(x, ±sat)       (sensor full-scale)
8. latency      push x into a ring buffer; output the sample lat_ms ago
```

Order matters: scale/temp/vibration corrupt the *physical* quantity; noise/bias add
sensor randomness; spikes and saturation are readout pathologies; latency is the
transport delay last. `f_blade` for vibration = (motor RPM × #blades) — derive from
`omega_[i]` already in the motor model, so vibration genuinely tracks throttle.

### CSV noise replay (source selector)
Per sensor, a `noise_src` flag chooses **Gaussian** (the stack above) or **CSV
replay**: a recorded real-sensor error trace is read from a file and added instead of
synthetic noise (mockup `setNoiseSrc()` ~1325). This lets a real bench capture drive
the sim. Implementation: GCS reads the CSV, streams rows to the daemon (chunked
control frames, like the mesh path transfer `sendWorldMesh` ~294–317), daemon plays
them back ring-buffered at the sensor rate.

## Protocol

The current `vsim_ctl_noise_t` is too small. Add a richer per-sensor message; keep
`SET_NOISE` working for back-compat (or migrate the existing fields into the new
struct and retire it).

`tools/vsim/include/vsim_proto.h`:

```c
VSIM_CTL_SET_SENSOR_ERR = 18,     // body: vsim_ctl_sensor_err_t

typedef struct {
    int32_t sensor;       // 0=acc 1=gyr 2=mag
    int32_t enable;       // master enable (0 = zero channel, today's behaviour)
    int32_t noise_src;    // 0=Gaussian 1=CSV
    float   scale[3];     // per-axis scale factor (1.0 = none)
    float   sigma;        // white noise RMS
    float   bias_clip;    // random-walk clip
    float   bias_walk;    // random-walk step σ
    float   sat;          // saturation full-scale (<=0 = none)
    float   spike_prob;   // per-sample outlier probability
    float   spike_mag;    // outlier magnitude
    float   vib_amp;      // vibration amplitude at full throttle
    int32_t vib_blades;   // blades per rotor (for f_blade)
    float   latency_ms;   // transport delay
    float   temp_coeff;   // bias per degree
    float   rate_hz;      // per-sensor sample rate (0 = default)
} vsim_ctl_sensor_err_t;  // ~64 B, well under 256
```

CSV replay rows use a separate streaming opcode (reuse the chunked-transfer pattern):

```c
VSIM_CTL_SENSOR_CSV = 19,   // body: { int32 sensor; int32 seq; int32 n; float rows[..] }
```

## Daemon

`tools/vsim/src/main.cpp` — replace/extend the `SET_NOISE` case (~388) with a
`SET_SENSOR_ERR` case that stores a full `SensorErrModel` per sensor on the
controller; add a `SENSOR_CSV` case that appends rows to that sensor's replay buffer.
The synthesis path (where samples are currently built with `mag_sigma` etc.) runs the
8-stage pipeline above per sample. Use the **single seeded RNG** so the optimizer
seed reproduces error realisations across runs (shares the seed with wind turbulence,
[01](01-wind-turbulence.md)).

A `SensorErrModel` per sensor owns: config, the running bias-walk state, the latency
ring buffer, the temperature (drive from the power model's heat or a simple ramp —
or expose a `sim temp` knob), and the CSV replay cursor.

## GCS — SimWorker & UI

`SimWorker`:

```cpp
void sendSensorErr(int sensor, const SensorErrConfig& c);   // VSIM_CTL_SET_SENSOR_ERR
void sendSensorCsv(int sensor, const QVector<float>& rows); // chunked VSIM_CTL_SENSOR_CSV
```

`SimulatorWidget` — this is the big UI piece: replace the flat 3-row `m_sensorRow[3]`
(σ/clip/enable, ~947–993) with a **per-sensor collapsible panel** mirroring the
mockup (`buildSensors()` ~1287, `SENS_COMPS` ~1279). For each of acc/gyr/mag:

| Mockup id | Control |
|-----------|---------|
| `sensor-{acc\|gyr\|mag}` | collapsible group (`toggleSensor`) |
| `nsrc-{key}` | Gaussian / CSV source toggle (`setNoiseSrc`) |
| `gauss-{key}` | the stack params (scale[3], σ, bias clip/walk, sat, spike p/mag, vib amp/blades, latency, temp coeff, rate) |
| `csv-{key}` | file picker → `sendSensorCsv` |

Each control → a `pushSensorErr(sensor)` slot (hot group). Keep an "advanced"
disclosure so the simple σ/clip case stays one click away (don't regress the current
quick workflow).

## Migration / back-compat

- Keep `pushNoise()` working by having it populate the new struct's `sigma`/
  `bias_clip`/`enable` and zeroing the advanced fields — i.e. the old call becomes a
  thin wrapper. Then the daemon only needs the new path.
- The existing `tst` noise behaviour (σ-only) must still pass with all advanced
  stages off.

## Tests

- **Unit (per stage, daemon-side):** scale → output scales; saturation → clamps;
  spikes → measured outlier rate ≈ `spike_prob` over a long run; vibration → spectral
  peak at `f_blade` and amplitude ∝ throttle; latency → output is input delayed by
  `latency_ms` (cross-correlation lag); temp bias → linear drift with temperature;
  all-off → identical to today's σ+bias path (regression guard).
- **CSV replay:** feed a known trace, assert the sensor output equals
  `truth + trace` row-for-row at the sensor rate.
- **Manual SITL:** crank vibration during an autotune run and confirm the cost / D-term
  behaviour reacts the way real gyro vibration would (filtered-D matters) — the whole
  point of the feature.

## Effort

~2–3 days — the largest of the five. The per-stage pipeline + CSV streaming + the
richer per-sensor UI are each non-trivial. Sequence the stages: ship scale + sat +
spikes + latency first (pure functions, easy tests), then vibration (needs motor-RPM
coupling), then temp bias (needs a temperature source), then CSV replay last.
