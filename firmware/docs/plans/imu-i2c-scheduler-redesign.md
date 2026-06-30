# I2C ownership + multi-rate, phase-offset sensor scheduler

## Context

The I2C1 bus is shared by the IMU (BMX160 @ 0x68) and the barometer (BME280 @
0x76). Today the **acquisition scheduler lives inside the IMU driver**
(`src/sensor/bmx160.c`): a hand-rolled `IMU_OP_FAST/MAG/TEMP/BARO` state machine,
driven by DMA-completion callbacks that pick `_next_op`, paced by a 2 kHz tick
semaphore. That driver also reaches across and reads the **BME280** (a different
device) inside its own loop. The `i2c_manager` owns the bus primitives (mutex,
atomic-busy, sync read/write, a one-shot async DMA read, a 9-clock `unstick`, and
`init` as the recovery entry point) but **not** the scheduling.

Two problems surfaced on hardware (2026-06-19):

1. **CPU/bus saturation.** Everything is read in one 12-byte `FAST` burst (gyro
   `0x0C` + accel `0x12`) at **2 kHz**, and `ATTITUDE_DECIM = 1` so the EKF and
   rate loop also run at 2 kHz. Measured ~**80 % CPU** on the IMU path. That flood
   of bus transactions is ~70 % bus utilisation by itself.
2. **Permanent I2C wedge (~320 s uptime).** A bus/NAK error
   (`hal_i2c_read_regs_dma` → `HAL_ERR_IO`, code 7) is hit, and the recovery
   (`i2c_manager_unstick` + `init_i2c_manager`) does **not** clear it — every
   subsequent DMA start returns `HAL_ERR_IO`, spamming `I2C DMA START FAIL: 7` /
   `RESTART FAILED` forever. More transactions ⇒ more chances to catch the error.

This plan does three things, in priority order:

- **A. Layering:** move *all* I2C hardware access **and** the acquisition
  scheduler out of `bmx160.c` into `i2c_manager`. Sensor drivers become pure:
  register a job, ingest raw bytes, convert. (User directive.)
- **B. Multi-rate, phase-offset scheduling:** read each sensor only as often as
  it needs, and phase the slots so they don't collide on the single-owner bus —
  cutting CPU and bus traffic (and thus wedge probability).
- **C. Recovery hardening:** make the bus actually recover from `HAL_ERR_IO`
  instead of latching.

Design only — no code here. File:line references are pointers to the live tree.

---

## 1. Target layering

```
            ┌─────────────────────────────────────────────┐
            │  i2c_manager  (SOLE owner of I2C1 hardware) │
            │  • bus mutex / atomic-busy / DMA            │
            │  • scheduler: tick → due jobs → one DMA     │
            │  • recovery: unstick + peripheral reset     │
            └───────────────▲───────────────┬─────────────┘
             register jobs  │               │ ingest raw bytes (ISR cb)
           ┌────────────────┴──┐     ┌──────▼───────────┐
           │ bmx160.c          │     │ bme280.c         │
           │ • reg map         │     │ • reg map        │
           │ • raw → physical  │     │ • compensation   │
           │ • publish queues  │     │ • publish        │
           └───────────────────┘     └──────────────────┘
```

**Invariant:** after this change, `bmx160.c` and `bme280.c` contain **zero**
calls to `hal_i2c_*`, `hal_gpio_*` for the bus, or `i2c_manager_read_async`. They
only (a) register an acquisition job at init and (b) provide an ingest callback
that copies/flags raw bytes. All `IMU_OP_*` state, `_next_op`, the tick pacing,
and the BME280 burst read move into `i2c_manager`.

### Job registration API (new, in `i2c_manager.h`)

```c
typedef struct {
  uint8_t  addr;          /* 7-bit device address */
  uint8_t  reg;           /* start register for the burst read */
  uint8_t  len;           /* bytes to read */
  uint16_t period_ticks;  /* run every N base ticks (1 = every tick) */
  uint16_t phase_ticks;   /* first fire at tick == phase (mod period) */
  void   (*ingest)(const uint8_t *data, uint8_t len);  /* ISR-context copy */
  bool   (*present)(void);/* optional gate; NULL = always scheduled */
} i2c_job_t;

int  i2c_manager_register_job(const i2c_job_t *job);   /* returns job id */
void i2c_manager_scheduler_start(uint32_t base_hz);    /* begins the tick loop */
```

- `present` lets the baro be skipped until detected (replaces the inline
  `bme280_is_present()` check at `bmx160.c:990`), so a missing device never NAKs
  the bus.
- `ingest` runs in DMA-completion ISR context — it must only copy + set a fresh
  flag (exactly what `bmx160_dma_callback_*` and `bme280_ingest_raw` already do).
  Heavy conversion stays in the driver tasks (`bmx160_process_data`,
  `bme280_read_task`).

The legacy one-shot `i2c_manager_read_async` and the sync `read/write/write_read`
stay for init-time register pokes (sensor config, WHO_AM_I), but **runtime
streaming goes only through the scheduler**.

---

## 2. The scheduler

A single base tick drives a cooperative, single-in-flight dispatcher:

- Base tick = **gyro rate** (decision D1 below). On each tick the scheduler picks
  the **highest-priority job whose `(tick % period) == phase`** and isn't gated
  out, issues exactly one DMA read, and on completion advances to the next due
  job for that tick, then waits for the next tick. Gyro (period 1) fires every
  tick; at most one slow job rides alongside it (see phasing).
- Single in-flight transaction (today's invariant) is preserved — the bus has one
  owner, so there is never contention, only *ordering*.

### Rates & phasing (decision D2)

| Job   | addr/reg/len   | period | rate     | phase | notes |
|-------|----------------|--------|----------|-------|-------|
| Gyro  | 0x68/0x0C/6    | 1      | base     | 0     | rate loop |
| Accel | 0x68/0x12/6    | 2      | base/2   | 0     | attitude; even ticks |
| Mag   | 0x68/0x04/8    | 11     | ~base/11 | 1     | odd phase, prime period |
| Temp  | 0x68/0x20/2    | 47     | ~base/47 | 3     | odd phase, prime period |
| Baro  | 0x76/0xF7/8    | 67     | ~base/67 | 5     | odd phase, prime period, `present`-gated |

**Phasing rationale (jitter avoidance).** On a single-owner bus a "collision" is
two jobs coming due on the same tick, which delays the gyro read and jitters the
rate loop. So: gyro every tick; accel on **even** ticks (period 2); the slow jobs
use **pairwise-coprime prime periods (11, 47, 67) on odd phases (1,3,5)** so they
land on **odd** ticks (never colliding with accel) and almost never coincide with
each other. Result: **at most one slow job per tick on top of gyro** → bounded,
near-zero added latency. (This is the user's "100 vs 45/55" idea generalised:
coprime periods + distinct phase.) Periods/phases are named constants so the
table is tunable.

At **base = 1 kHz**: gyro 1000, accel 500, mag ~91, temp ~21, baro ~15 Hz — all
adequate (BME280 maxes ~25 Hz; mag ~100 Hz is plenty). Headroom remains to add
**vertical/position** jobs later at their own coprime periods (the user's "double
rate as the need arises").

### Win vs today
- High-rate read: 12 B @ 2 kHz → **6 B @ 1 kHz** (~4× less hot-path bus/CPU).
- Accel 4× less often; mag/temp/baro unchanged-ish but explicitly scheduled.
- Total transactions drop sharply → frees the ~80 % CPU **and** cuts the
  `HAL_ERR_IO` wedge probability.

---

## 3. Estimator decoupling (gyro/accel no longer arrive together)

The EKF already separates **predict (gyro)** from the **accel update**
(`ekf.h`: "mirroring the decoupling the Mahony filter uses"), so this is feasible
without touching the filter math.

- The combined `imu_queue_attitude` sample (`bmx160_all_reading_t`) becomes a
  carrier with **per-field fresh flags** (gyro always fresh; accel fresh only on
  its slots), or two queues. Decision D3: prefer **one queue, gyro-paced, with an
  `accel_fresh` flag** — minimal churn to `attitude_task`.
- `attitude_task`: `ekf_predict()` every gyro sample (or decimated to the angle
  loop's 250 Hz — D4); `ekf_update_accel()` only when `accel_fresh`. Today it
  averages acc/gyr over `ATTITUDE_DECIM` and steps both together — that becomes
  predict-on-gyro / update-on-fresh-accel.
- Rate controller consumes gyro at base rate.
- **SITL note:** the op-chain is hardware-only (SITL feeds IMU via
  `host_imu_feeder` → `imu_queue`, not the I2C path). But the **estimator
  decoupling IS SITL-testable** — feed gyro/accel at different cadences through
  the queue and confirm attitude still tracks. Do that before flashing.

---

## 4. Recovery hardening (the permanent wedge)

`unstick` already does the right *bus* clear (9 SCL pulses + STOP). The latch is
almost certainly at the **peripheral/DMA** level: after `HAL_ERR_IO` the I2C error
flags (BERR/AF/ARLO) and/or the DMA stream aren't fully cleared, so re-init +
next DMA re-trips. Plan:

- On `HAL_ERR_IO`: disable the I2C peripheral (PE=0), abort/disable the DMA
  stream, clear its flags, clear I2C SR error bits, then `unstick`, then
  re-`init`, then a **single probe read** (e.g. WHO_AM_I) before resuming the
  scheduler. Only resume on a clean probe.
- Bound recovery attempts; on repeated failure, raise a `SYS-SAFE` degraded/
  FAILSAFE signal rather than spamming the log forever.
- This work is partly in `i2c_manager` and partly may need a NavHAL
  reset/abort entry point — audit `extern/.../i2c.c` for an existing one before
  adding.

---

## 5. Resolved / open decisions

- **D1 — Base rate: 1 kHz now.** Gyro/rate-loop at 1 kHz (user: "running it at
  1 kHz then we can decide"). Framework allows raising the base later without
  touching the phasing table. *(confirm)*
- **D2 — Phasing: coprime prime periods on odd phases** (table §2). *(confirm)*
- **D3 — Queue shape: one gyro-paced queue + `accel_fresh` flag** (vs two
  queues). *(confirm)*
- **D4 — EKF predict rate: gyro rate vs 250 Hz angle-loop decimation.** Predict
  is cheap; the covariance *update* is the cost and runs at accel/250 Hz anyway.
  Lean: predict at gyro rate, update on fresh accel. *(confirm)*
- **D5 — Scope of "all I2C in i2c_manager":** runtime streaming + recovery move
  in full; init-time register pokes keep using the sync/one-shot APIs (still in
  i2c_manager, still the only place hal_i2c_* is called). *(confirm)*

---

## 6. Phasing (build order)

1. **Recovery hardening (§4)** — standalone, highest safety value, ships the fix
   for the 320 s wedge independent of the refactor. Bench-verify it survives an
   induced bus error.
2. **Scheduler in i2c_manager (§1, §2)** — introduce the job API + tick
   dispatcher; port the IMU + baro reads to jobs; delete the `IMU_OP_*` machine
   and the BME280 read from `bmx160.c`. Behaviour-preserving first (same rates),
   then apply the multi-rate table.
3. **Estimator decoupling (§3)** — SITL-test the predict/update split, then wire
   gyro/accel as separate-cadence inputs.
4. **Hardware bring-up** — flash, watch CPU (target ≪80 %), run >>320 s to
   confirm no wedge, verify attitude/rate/baro/VERT telemetry intact.

Each step is independently flashable and reversible only by reflash — bench, not
flight, until step 4 is solid.
```
