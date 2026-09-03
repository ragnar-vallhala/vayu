# IMU calibration improvements (PX4-modeled)

Two focused upgrades to the vayu IMU calibration, each modeled on PX4-Autopilot:

1. A **runtime gyro-bias learner** (Welford + still/temp gates + persist), replacing
   the crude hot-path EMA in `bmx160.c`.
2. A **6-point closed-form accel offset+scale** fit, as an alternative to the
   free-rotation ellipsoid fit in `calib_engine`.

Reference: PX4 `src/modules/gyro_calibration/GyroCalibration.cpp` and
`src/modules/commander/accelerometer_calibration.cpp`.

---

## Part 1 — Runtime gyro-bias learner

### What vayu does today

`bmx160_process_data()` runs an inline EMA learner on **every IMU sample**
(`firmware/src/sensor/bmx160.c:1284-1304`):

```c
if (system_state_get() != SYSTEM_STATE_CALIBRATING) {
  if (acc_sq in (9.61^2, 10.01^2) && gyro_sq < 0.2^2) stable_count++;
  else stable_count = 0;
  if (stable_count > 200) {
    gyr_offset[i] = (1-ALPHA)*gyr_offset[i] + ALPHA*gyr[i];   // ALPHA = 0.0034
    if (|gyr_offset[i]| > 5) gyr_offset[i] = 0;
  }
}
gyr[i] -= gyr_offset[i];
```

It works, but it has seven concrete weaknesses versus PX4's `GyroCalibration`:

| # | Weakness | Consequence |
|---|---|---|
| 1 | Runs in the **2 kHz hot path** | wasted cycles; the learner has no reason to run per-sample |
| 2 | **Fixed single-pole EMA** (α=0.0034), no quality metric | low-passes whatever passes the gate; can chase low-freq noise |
| 3 | **No temperature gate** | a bias learned cold is applied hot (BMX160 gyro bias is strongly T-dependent) |
| 4 | **Not persisted** | RAM-only — every reboot discards the learned refinement and restarts from the static cal |
| 5 | **Chicken-and-egg still-gate** | `gyro_sq` is computed on the *pre-subtraction* rate (line 1281, before the `-= offset` at 1304). If the true bias exceeds `GYRO_STILL_DPS` (0.2 °/s) the gate can **never open** — the learner is dead exactly when the bias is large enough to matter |
| 6 | **Reset-to-zero on \|offset\|>5** | injects a step discontinuity straight into the control gyro mid-operation |
| 7 | **No commit hysteresis** | writes `gyr_offset` on every still sample → the "calibration" is perpetually drifting, non-deterministic |

### PX4's model (what we copy)

- Runs at **50 Hz while disarmed only**.
- Feeds thermal-corrected gyro into a **Welford** online mean/variance per axis.
- Gates: accel motion `<0.5 m/s²`, over-g `<1.3 g`, **temp change `>1 °C` → reset**,
  gyro variance `<0.001 (rad/s)²`.
- Commits every **5 s** (count>100) only if the mean moved **>1σ** OR variance is
  **≥10× tighter** than the saved cal OR never calibrated; `set_offset` deadband
  `0.01 rad/s`; then persists to params.

### Proposed vayu module — `gyro_bias_learner.{c,h}`

Self-contained, no heap, host-testable. The sensor task feeds it **decimated**
samples; the learner owns the still/temp gates, the Welford accumulator, and the
commit+persist decision. `bmx160.c` keeps only the one-line apply.

```c
/* firmware/include/sensor/gyro_bias_learner.h */
#ifndef GYRO_BIAS_LEARNER_H
#define GYRO_BIAS_LEARNER_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  /* Welford per-axis (mean/M2) */
  uint32_t count;
  float    mean[3];
  float    m2[3];
  /* gates */
  float    acc_ref[3];      /* motion reference (m/s^2)   */
  float    temp_ref;        /* last temperature (deg C)   */
  bool     have_temp_ref;
  /* saved-cal quality + cadence */
  float    saved_var[3];    /* variance of the last committed offset */
  uint32_t last_commit_ms;  /* quiet-period stamp */
  bool     ever_committed;
} gyro_bias_learner_t;

void gyro_bias_learner_init(gyro_bias_learner_t *L);

/* Call at ~50 Hz. gyr = RAW dps (uncorrected), acc = m/s^2, temp = deg C,
 * disarmed = (state != ARMED && state != IN_AIR && state != CALIBRATING),
 * now_ms = a millisecond clock. Returns true and fills new_offset[3] on the
 * ticks where a fresh offset should be committed (caller applies + persists);
 * returns false otherwise. */
bool gyro_bias_learner_update(gyro_bias_learner_t *L, const float gyr[3],
                              const float acc[3], float temp, bool disarmed,
                              uint32_t now_ms, float new_offset[3]);
#endif
```

```c
/* firmware/src/sensor/gyro_bias_learner.c */
#include "sensor/gyro_bias_learner.h"
#include "variables.h"
#include "maths/maths_interface.h"   /* m_fabsf, m_sqrt */
#include <string.h>

/* All thresholds map to variables.h (values below are the recommended defaults,
 * expressed in vayu's dps units — PX4's rad/s equivalents in comments). */
#ifndef GYRO_LEARN_ACC_MOTION_MS2
#define GYRO_LEARN_ACC_MOTION_MS2 0.5f       /* PX4 0.5 m/s^2                 */
#endif
#ifndef GYRO_LEARN_OVER_G
#define GYRO_LEARN_OVER_G (1.3f * 9.80665f)  /* PX4 1.3 g                     */
#endif
#ifndef GYRO_LEARN_TEMP_RESET_C
#define GYRO_LEARN_TEMP_RESET_C 1.0f         /* PX4 1.0 degC                  */
#endif
#ifndef GYRO_LEARN_VAR_MAX_DPS2
#define GYRO_LEARN_VAR_MAX_DPS2 3.3f         /* PX4 0.001 (rad/s)^2 ~= 3.28 dps^2 */
#endif
#ifndef GYRO_LEARN_MIN_COUNT
#define GYRO_LEARN_MIN_COUNT 100u            /* PX4 100                       */
#endif
#ifndef GYRO_LEARN_COMMIT_DEADBAND_DPS
#define GYRO_LEARN_COMMIT_DEADBAND_DPS 0.02f /* PX4 0.01 rad/s ~= 0.57 dps; tighter here */
#endif
#ifndef GYRO_LEARN_QUIET_MS
#define GYRO_LEARN_QUIET_MS 5000u            /* PX4 evaluates every 5 s       */
#endif
#ifndef GYRO_LEARN_SANITY_DPS
#define GYRO_LEARN_SANITY_DPS 5.0f           /* reject an absurd learned bias */
#endif

static void reset_accum(gyro_bias_learner_t *L) {
  L->count = 0u;
  for (int i = 0; i < 3; i++) { L->mean[i] = 0.0f; L->m2[i] = 0.0f; }
}

void gyro_bias_learner_init(gyro_bias_learner_t *L) {
  memset(L, 0, sizeof(*L));
  L->have_temp_ref = false;
  L->ever_committed = false;
}

/* Welford incremental update, per axis. */
static void welford_push(gyro_bias_learner_t *L, const float g[3]) {
  L->count++;
  for (int i = 0; i < 3; i++) {
    float d = g[i] - L->mean[i];
    L->mean[i] += d / (float)L->count;
    L->m2[i]   += d * (g[i] - L->mean[i]);
  }
}
static void welford_var(const gyro_bias_learner_t *L, float var[3]) {
  float inv = (L->count > 1u) ? 1.0f / (float)(L->count - 1u) : 0.0f;
  for (int i = 0; i < 3; i++) var[i] = L->m2[i] * inv;
}

bool gyro_bias_learner_update(gyro_bias_learner_t *L, const float gyr[3],
                              const float acc[3], float temp, bool disarmed,
                              uint32_t now_ms, float new_offset[3]) {
  /* (0) only learn on the ground, disarmed. */
  if (!disarmed) { reset_accum(L); return false; }

  /* (1) temperature gate — any >1 degC step invalidates the window (fix for
   *     weakness #3: keeps each learned offset in a narrow temperature band). */
  if (!L->have_temp_ref || m_fabsf(temp - L->temp_ref) > GYRO_LEARN_TEMP_RESET_C) {
    L->temp_ref = temp; L->have_temp_ref = true; reset_accum(L); return false;
  }

  /* (2) accel motion gate: not being moved, and roughly 1 g. */
  float amag2 = acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2];
  if (amag2 > GYRO_LEARN_OVER_G * GYRO_LEARN_OVER_G) { reset_accum(L); return false; }
  if (L->count == 0u) { L->acc_ref[0]=acc[0]; L->acc_ref[1]=acc[1]; L->acc_ref[2]=acc[2]; }
  float da0=acc[0]-L->acc_ref[0], da1=acc[1]-L->acc_ref[1], da2=acc[2]-L->acc_ref[2];
  if (da0*da0+da1*da1+da2*da2 > GYRO_LEARN_ACC_MOTION_MS2*GYRO_LEARN_ACC_MOTION_MS2) {
    reset_accum(L); return false;
  }

  /* (3) accumulate the RAW rate (fix for weakness #5: gate on motion via the
   *     accel, NOT on the possibly-biased gyro magnitude, so a large existing
   *     bias can never lock the learner out). */
  welford_push(L, gyr);

  /* (4) commit decision — every quiet-period, with enough samples. */
  if (L->ever_committed && (now_ms - L->last_commit_ms) < GYRO_LEARN_QUIET_MS)
    return false;
  if (L->count < GYRO_LEARN_MIN_COUNT) return false;

  float var[3]; welford_var(L, var);
  /* variance gate — window must be quiet (fix for weakness #2). */
  if (var[0] > GYRO_LEARN_VAR_MAX_DPS2 || var[1] > GYRO_LEARN_VAR_MAX_DPS2 ||
      var[2] > GYRO_LEARN_VAR_MAX_DPS2) { reset_accum(L); return false; }

  /* sanity: never commit an absurd bias (replaces the mid-flight reset-to-0). */
  for (int i = 0; i < 3; i++)
    if (m_fabsf(L->mean[i]) > GYRO_LEARN_SANITY_DPS) { reset_accum(L); return false; }

  /* PX4 significance test: moved >1 sigma OR 10x tighter OR never calibrated. */
  bool moved = false, tighter = false;
  for (int i = 0; i < 3; i++) {
    float dm = L->mean[i] - new_offset[i];   /* caller passes current offset in */
    if (dm*dm > var[i]) moved = true;
    if (var[i] < 0.1f * L->saved_var[i]) tighter = true;
  }
  bool worth =
      !L->ever_committed || moved || tighter;
  /* deadband so tiny changes don't churn the SD (fix for weakness #7). */
  float dsum2 = 0.0f;
  for (int i = 0; i < 3; i++) { float d = L->mean[i]-new_offset[i]; dsum2 += d*d; }
  if (worth && dsum2 > GYRO_LEARN_COMMIT_DEADBAND_DPS*GYRO_LEARN_COMMIT_DEADBAND_DPS) {
    for (int i = 0; i < 3; i++) { new_offset[i] = L->mean[i]; L->saved_var[i] = var[i]; }
    L->ever_committed = true;
    L->last_commit_ms = now_ms;
    reset_accum(L);        /* start a fresh window after a commit */
    return true;
  }
  return false;
}
```

#### Integration in `bmx160.c`

1. Add one static: `static gyro_bias_learner_t s_gyro_learner;` and
   `gyro_bias_learner_init(&s_gyro_learner)` in `bmx160_init`.
2. **Replace** the inline block `bmx160.c:1284-1301` (keep the `-= gyr_offset`
   at 1303-1304) with a **decimated** call (every ~40th sample ≈ 50 Hz at 2 kHz):

```c
static uint16_t s_learn_decim = 0;
if (++s_learn_decim >= LEARN_DECIM) {            /* LEARN_DECIM = IMU_SAMPLE_FREQ_HZ/50 */
  s_learn_decim = 0;
  sys_state_t st = system_state_get();
  bool disarmed = (st != SYSTEM_STATE_ARMED && st != SYSTEM_STATE_IN_AIR &&
                   st != SYSTEM_STATE_CALIBRATING);
  float off[3] = {bmx160_calib.gyr_offset[0], bmx160_calib.gyr_offset[1],
                  bmx160_calib.gyr_offset[2]};
  if (gyro_bias_learner_update(&s_gyro_learner, _bmx_data.converted.gyr_raw,
                               _bmx_data.converted.acc_raw, _bmx_data.converted.temp,
                               disarmed, v_get_ms(), off)) {
    for (int i = 0; i < 3; i++) bmx160_calib.gyr_offset[i] = off[i];   /* apply live */
    /* persist (fix for weakness #4): reuse the same enqueue the wizard uses. */
    calib_file_header_t hdr = {CALIB_FILE_MAGIC, CALIB_FILE_VERSION,
                               (uint16_t)sizeof(bmx160_calibration_t)};
    (void)fs_owner_enqueue_calib_save(&hdr, sizeof(hdr), &bmx160_calib,
                                      sizeof(bmx160_calibration_t));
  }
}
for (int i = 0; i < 3; i++) _bmx_data.converted.gyr[i] -= bmx160_calib.gyr_offset[i];
```

- **Feed `gyr_raw`, not the LPF'd/subtracted `gyr`** — the learner wants the raw
  absolute rate; it does its own gating.
- **Concurrency:** `gyr_offset[i]` is a single float read in the hot path and
  written here (same task, actually — both in the sensor path — so no cross-task
  race at all; the apply and the learner run in the same `bmx160_process_data`).
- **Persist throttling:** the quiet-period (5 s) + deadband already bound SD
  writes to at most one per 5 s and only on a real change; in practice a settled
  board commits once then goes quiet.

#### Notes / decisions

- **Scope = ground/disarmed only** (like PX4's `GyroCalibration` module). The
  richer PX4 *in-flight* path fuses the EKF's `estimator_sensor_bias`; vayu's EKF
  already estimates a gyro-bias state (`ekf.c`) so an equivalent in-flight learner
  is possible later, but it belongs in the estimator, not the driver — out of
  scope here. See [[indi-rate-loop]] / [[onhw-tune-logreport]] for why a stable
  ground zero already helps the persistent-bias hunt.
- Delete `GYRO_BIAS_ALPHA` and the `stable_count`/`ACC_STILL_*`/`GYRO_STILL_*`
  hot-path constants once the learner owns stillness.

---

## Part 2 — 6-point closed-form accel offset + scale

### What vayu does today

The accel wizard captures N averaged static poses (coverage-gated by
`pose_advances_coverage`) and runs the **ellipsoid least-squares fit**
(`calib_engine.c:run_ellipsoid`/`finalize` → `calib_fit_ellipsoid`), producing
`acc_offset[3]` + a **full 3×3** `acc_soft_iron`, then rescales so `|a|=g`.

That fit is strictly *more general* than PX4's (it recovers cross-axis
misalignment, not just diagonal scale), but it pays for it:

- needs **diverse coverage** (the `cov_done`/`pose_advances_coverage` machinery),
- can **fail on degenerate/planar** pose sets (`calib_fit_ellipsoid` returns −1 if
  Q isn't positive-definite),
- the outcome depends on *which* orientations the operator happened to hit.

### PX4's model (the closed form)

PX4 uses exactly **6 axis-aligned sides** and solves in closed form
(`accelerometer_calibration.cpp:346-379`):

```
offset[k] = (pose[+k][k] + pose[-k][k]) / 2          # midpoint of opposing sides
A         = [ pose[+x]-offset ; pose[+y]-offset ; pose[+z]-offset ]   # 3x3
accel_T   = inv(A) * g                                # exact, since b = g*I
scale     = diag(accel_T)                             # PX4 keeps only the diagonal
```

Deterministic, no coverage gating, no iterative solve, can't land on a degenerate
fit. The price: it needs all six axis-aligned poses and (as PX4 stores it) keeps
only diagonal scale.

> **Status: IMPLEMENTED** (selectable via `ACCEL_CALIB_METHOD`, default =
> `ACCEL_CALIB_ELLIPSOID` so behaviour is unchanged until flipped). Lives in
> `calib_ellipsoid.c::calib_fit_sixpoint`, dispatched by
> `calib_engine_fit_points` on `CALIB_FIT_SIXPOINT`, wired in `bmx160.c`. Host
> test: `test_calib_engine.c` cases [8]/[9]. See the transpose note below.

### Proposed vayu function — `calib_fit_sixpoint`

Drop-in for `calib_ellipsoid.{c,h}`, usable as a new `CALIB_FIT_SIXPOINT` target
or called directly from `calibration_task`. Because vayu's `acc_soft_iron` is a
**full 3×3**, we can keep the *entire* `accel_T` (misalignment for free) — better
than PX4's diagonal-only storage. Define `CALIB_SIXPOINT_DIAG_ONLY` for
PX4-faithful diagonal behavior.

> **Transpose subtlety (found during implementation).** PX4 computes
> `accel_T = inv(A)·g` and keeps only its *diagonal*, applied element-wise. vayu
> applies the **full** matrix `soft·(raw − offset)`, and the constraint is
> `accel_T · Aᵀ = g·I`, so the correct full matrix is
> `accel_T = g·(A⁻¹)ᵀ = (g/det)·cofactor(A)` — the transpose of PX4's `inv(A)`.
> The diagonal is identical either way (which is why PX4's diagonal-only storage
> never had to care). The shipped code uses the cofactor form; the host test
> verifies a non-diagonal `A_sens` is recovered exactly.

The shipped implementation is `firmware/src/calib/calib_ellipsoid.c::calib_fit_sixpoint`.
It differs from the sketch above in two deliberate ways:

- **Classify by direction, not just dominant axis.** Each side picks the pose with
  the highest cosine to that axis (`bestcos ≥ 0.80`, ~37°), so a slightly-off hold
  still slots correctly and a genuinely missing side is caught (its best cosine
  never clears the threshold). Duplicate-pose selection is rejected too.
- **Cofactor (not `inv(A)`) for the full matrix**, per the transpose note above.

Signature: `int calib_fit_sixpoint(const float (*pts)[3], int npts, float g,
float offset[3], float soft[9])` — takes the whole captured pose set (it selects
the six sides itself). It matches vayu's runtime apply exactly:
`acc_cal = soft · (acc_raw − offset)` (the same `offset + 3×3` contract
`acc_commit` already stores), so no downstream change is needed.

### Trade-off & recommendation

| | Ellipsoid LSQ (current) | 6-point closed form (proposed) |
|---|---|---|
| Poses | many, diverse (coverage-gated) | exactly 6 axis-aligned |
| Solve | iterative normal-equations | exact closed form |
| Captures misalignment | yes (full 3×3) | yes if you keep full `accel_T`; diag = PX4 |
| Noise robustness | high (over-determined LSQ) | per-pose average only (exactly determined) |
| Failure mode | −1 on degenerate coverage | −1 only if a side is missing/duplicated |
| Operator UX | "rotate freely until covered" | "lay it on each of its 6 sides" |

**Recommendation:** *add* `CALIB_FIT_SIXPOINT` as the accel path and drive it from
a **guided 6-side wizard** (the PX4 UX — clearer for users and impossible to land
degenerate), while **keeping the ellipsoid fit** for the magnetometer (free
rotation has no six natural sides) and as a fallback. Don't rip out the
ellipsoid: it's the more capable fit and the mag needs it. The six-point path is
the better *default accel* experience; the ellipsoid stays the general tool.
