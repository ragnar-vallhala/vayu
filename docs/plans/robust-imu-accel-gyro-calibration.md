# Robust, pose-tolerant accelerometer + gyro calibration (+ shared calib module)

## Context

Today the accelerometer is calibrated by a **strict 6-point routine**
(`src/sensor/bmx160.c:1718-1769`): the operator places the airframe in six named
poses (UPRIGHT, UPSIDE_DOWN, NOSE_UP, NOSE_DOWN, RIGHT_DOWN, LEFT_DOWN) and
`wait_for_orientation()` (`bmx160.c:1534-1659`) only accepts a pose when the
target axis **dominates** the gravity vector (`cross_ratio < 20%`, ~12°
tolerance, `:1574`) with the correct sign. Offset/scale then come from just the
two ±g pose values per axis: `o=(p+n)/2`, `s=2g/(p−n)` (`:1750-1756`).

This is fragile in exactly the way the user hit:
- **It demands near axis-aligned poses.** A board that can't be held flat on each
  face can never satisfy the dominance gate and the routine wedges (the NOSE_UP
  sign fix at `:1516-1520` is a scar from this).
- **Two samples per axis define the line.** One sloppy pose skews that axis's
  offset and scale with no redundancy to average it out, and cross-axis
  misalignment is ignored entirely (diagonal model).
- **The gyro path is unguarded.** Selector 2 (`bmx160.c:1771-1801`) blindly
  averages 500 `gyr_raw` samples with **no stillness check** — if the board
  drifts during the capture, the bias is silently wrong.

**Live evidence (2026-06-25, board static, `ImuRaw` over UDP):**
`|a| = 10.215 m/s²` vs `g = 9.807` (~4% off) with `aX,aY ≈ +0.43` on a roughly
level board → accel offset+scale genuinely uncalibrated; `gY,gZ ≈ +0.45/−0.46
°/s` residual gyro bias (gX centred). The runtime auto-trim
(`bmx160.c:1248-1268`) won't fix the gyro here because `‖ω‖≈0.64 > GYRO_STILL_DPS
(0.2)` keeps its stillness gate shut — so a good explicit capture matters.

**Goal (three parts):**
1. **Replace** the accel routine with a **full 3×3 ellipsoid calibration** (offset
   + shape matrix, capturing scale *and* cross-axis misalignment) that **tolerates
   inexact poses**. To make the full 9-DOF fit well-conditioned we prompt **~12
   orientations — the 6 faces plus ~6 edge/corner holds** (the cross-axis terms are
   only observable when gravity is shared between axes, i.e. on an edge/corner).
   The prompts are *advisory*: the algorithm tolerates each being inexact, so the
   operator just rests the board roughly as shown and holds still.
2. **Make the gyro capture stillness-gated** as its own explicit routine
   (selector 2), kept entirely separate from the accel routine — accel and gyro
   stay independent commands, as today.
3. **Lift the whole calibration out of the bmx160 driver into a standalone,
   sensor-agnostic module**, so a future sensor (a second IMU, etc.) reuses the
   same engine + ellipsoid math by registering a small descriptor.

**Constraints from the user:** **NavLink is unchanged** — same command msgid, same
calibration telemetry message and wire. The added edge/corner pose prompts are new
`CALIB_UPDATE_*` *status codes carried inside that existing message's buffer*
(`SYSTEM_ORIGIN_CALIBRATION`), **not** a dialect/codec change. The **GCS wizard
does change** for accel: it gains the new edge/corner pose illustrations
(user-accepted via D5). Magnetometer behaviour is **out of scope** (its ellipsoid
fit is already robust) — but it becomes a *consumer* of the new shared module (no
behaviour change).

This is a design plan only. File:line references point into the live tree and are
pointers, not contracts.

---

## 1. What we reuse (mostly re-wiring, not new math)

The machinery already exists in the file — built for the mag:

- **The ellipsoid solver** `mag_fit_ellipsoid(S[81], t[9], offset[3], soft[9])`
  (`bmx160.c:1473-1506`) and its helpers `solve9x9`/`inv3x3`/`jacobi_eig3`/
  `cbrt_pos` (`:1476/1483/1490`). It accumulates the 9-parameter quadric normal
  equations `S·p = t`, recovers the centre (offset) and the **full 3×3 shape**
  `soft` that maps the fitted ellipsoid back to a sphere. It is sensor-agnostic:
  feed it 3-vectors that *should* lie on a sphere and it returns offset + a full
  3×3 correction. **Accel static samples are exactly such vectors** (every static
  reading has true magnitude `g`), so it is reusable directly — and it already
  produces the 3×3 we now want.
- **The online-accumulation + commit-only-on-success loop** (`bmx160.c:1808-1899`)
  — accumulate `S`/`t` over motion and **keep the previous calibration if the fit
  fails** (`:1890-1899`). The template for the accel routine.
- **The stillness detector** — `ACC_STILL_LO_SQ`/`HI_SQ`/`GYRO_STILL_SQ`
  (`:30-37`) and the `stable_count` contiguous-window idea (`:1248-1255`) — reused
  to decide "this hold is a valid static capture" instead of axis-dominance.
- **Telemetry + persistence** — `calib_telemetry()` (`:1662-1670`, the
  `CALIB_UPDATE_*` pose/progress frames) and `fs_owner_enqueue_calib_save()` with
  the versioned header (`:1923-1926`).

The genuinely new code is small: pose-tolerant static capture, the full-matrix
storage/apply, the gyro stillness gate — and the module extraction (§5).

---

## 2. The new accelerometer calibration (true full 3×3, ~12 poses)

### Method — ~12 prompted holds, tolerant capture, ellipsoid fit
Step through **~12 pose prompts: the 6 faces + ~6 edge/corner holds** (D5). The
edge/corner holds are what make the off-diagonal terms observable — at a face pose
two axes read ~0 so the `2yz/2xz/2xy` columns vanish, but on an edge gravity
splits ~`g/√2` across two axes (a corner ~`g/√3` across all three). A practical
set covers each axis pair (XY, XZ, YZ) with at least two edges. Then:

1. **Relax acceptance.** Drop the strict axis-dominance gate (`:1574`). A hold is
   accepted once it is **static** (the stillness test) and `|a|` is in the wide
   `0.45g..2g` band already present at `:1572-1573` (tolerates large scale error).
   Each prompt is *advisory* — "roughly this edge" is enough; the algorithm uses
   whatever orientation is actually held.
2. **Accumulate across all holds.** Feed every accepted static `acc_raw` sample
   (coords scaled by `1/g` for float32 conditioning, the `MAG_FIT_NORM` analogue)
   into the same `S`/`t` accumulator the mag uses — hundreds of points across ~12
   well-spread directions → a well-conditioned 9-DOF system.
3. **Fit the full ellipsoid** with `mag_fit_ellipsoid()` → `offset_norm`, `soft`.
   Convert `acc_offset[i] = offset_norm[i]·g`; the dimensionless `soft` is the full
   correction matrix (so `‖soft·(a_raw − acc_offset)‖ = g` by construction). **Commit
   only on a successful fit**, else keep the previous calibration (mirror
   `:1890-1899`).
4. **Progress** rides the existing `CALIB_UPDATE_PROGRESS` frames, one step per
   prompted hold (now ~12 instead of 6).

### Why this tolerates inexact poses (the math)
The fit uses a **magnitude-only** constraint: every static reading must satisfy
`‖soft·(a_raw − o)‖ = g`, *regardless of which way the board points*. It never
references the prompted pose's nominal direction, so a pose that is off by 10–20°
introduces **zero** model error — it just moves the sample to a different point on
the same sphere. ~12 diverse-but-loose orientations, hundreds of samples, least
squares → no single placement dominates. (The current method is the brittle
special case: two exact axis points per axis, no redundancy.)

### Model + storage (full 3×3 replaces diagonal)
- Add `float acc_soft_iron[9]` (row-major 3×3, identity default) to
  `bmx160_calibration_t` (`include/sensor/bmx160.h:251-257`), exactly mirroring
  `mag_soft_iron`. `acc_scale[3]` is **retired** (or kept only as the soft-iron
  diagonal for logging).
- **Apply path:** change `(acc[i]−offset[i])·scale[i]` (`bmx160.c:1291-1292`) to the
  matrix-multiply already written for the mag (`bmx160.c:1080-1082`) — 3 extra
  mults/axis, negligible at IMU rate.
- **On-disk:** bump `CALIB_FILE_VERSION 2→3` (`bmx160.h:263`). This is FC-internal
  only. An old v2 `cal.bin` fails the magic/version/size check on load and falls
  back to compiled identity defaults (`bmx160.h:259-264`) — graceful; the operator
  re-runs calibration once. **No GCS/NavLink impact** (the file is never sent over
  the link).

> Conditioning note: the ~6 edge/corner holds are what condition the off-diagonal
> terms (face poses alone leave the `2yz/2xz/2xy` columns ~0). With the full ~12
> set the 9-DOF system is well-posed. Keep light Tikhonov regularisation (small `λ`
> on the off-diagonal rows of `S` before `solve9x9`) as a cheap backstop so a
> missed/sloppy edge degrades gracefully toward the diagonal rather than blowing up.

---

## 3. The gyro calibration (stillness-gated, separate routine)

Gyro stays its **own explicit command** (selector 2), independent of accel — same
as today, just made robust. Replace the blind 500-sample average (`:1783-1799`):
only accumulate `gyr_raw` while the **stillness gate** holds (`ACC_STILL_*` +
`GYRO_STILL_SQ`); reset the window on disturbance (reuse the `:1635-1646`
re-prompt pattern); track running variance and **report FAILED (keep the old
offset)** if the window can't complete within a timeout or variance exceeds a
sanity bound. Commit `gyr_offset = mean` only on a clean window — this directly
fixes the live `gY/gZ ≈ 0.45 °/s` residual. The continuous auto-trim
(`:1248-1268`) then maintains it. No state race: the capture runs in CALIBRATING
where the auto-trim is gated off (`:1247`).

In the shared module (§5) this is just a `CALIB_FIT_BIAS` target — same engine,
different fit — so the accel and gyro routines share the stillness-gating and
telemetry code without being coupled into one pass.

---

## 4. Wire / GCS impact (NavLink unchanged; GCS wizard updated)

- **NavLink unchanged.** Same calibration command msgid and the same calibration
  telemetry message + wire. No dialect edit, no codec regen. The new edge/corner
  prompts are extra `calib_update_type_t` **status codes carried inside the
  existing message buffer** (`include/comm/comm_types.h:74-97`,
  `SYSTEM_ORIGIN_CALIBRATION`) — an app-level enum, not a NavLink message.
- **comm_types.h gains ~6 pose codes** (e.g. `CALIB_UPDATE_EDGE_*` /
  `CALIB_UPDATE_CORNER_*`) for the new holds, alongside the existing face codes.
  Keep `bmx160.c`'s prompt sequence and the GCS interpretation in lockstep (the
  enum is the shared contract).
- **GCS wizard changes** (accepted via D5): `software/src/core/CalibrationWizard.cpp`
  + `…/widgets/CalibrationWidget.cpp` add the edge/corner pose illustrations and
  step through ~12 prompts; update `software/tests/tst_calibration_wizard.cpp`. The
  flow shape is the same (prompt → hold → next → done), just more steps.
- **`cal.bin` v3 stays firmware-internal** — the file never crosses the link, so the
  version bump is invisible to the GCS/NavLink.

---

## 5. Standalone, sensor-agnostic calibration module

Today calibration is welded into the IMU driver: `calibration_task`,
`wait_for_orientation`, `mag_fit_ellipsoid` + solvers, and the per-sensor branches
all live in `bmx160.c`. Extract it so any sensor can reuse it.

### Layout (`src/calib/`)
- **`calib_ellipsoid.{c,h}`** — pure math: `solve9x9`, `inv3x3`, `jacobi_eig3`,
  `cbrt_pos`, and `calib_fit_ellipsoid(S, t, offset, soft)` (the body of
  `bmx160.c:1473-1506`). No sensor or RTOS deps → trivially host-unit-testable.
- **`calib_engine.{c,h}`** — the orchestration (the body of `calibration_task` +
  `wait_for_orientation`): enter/exit CALIBRATING, stillness gating, per-pose
  prompting + progress telemetry, `S`/`t` accumulation, the ellipsoid or bias fit,
  commit-on-success, and the `fs_owner` persistence handoff. It operates on an
  injected descriptor and never names a sensor.

### The descriptor a sensor registers
```c
typedef enum { CALIB_FIT_ELLIPSOID, CALIB_FIT_BIAS } calib_fit_t;

typedef struct {
  const char     *name;
  calib_fit_t     fit;                 /* ellipsoid (accel/mag) or bias (gyro) */
  float           radius;              /* ellipsoid target |v|: g, |B|, …       */
  const uint8_t  *poses; uint8_t n_poses; /* prompt script; NULL = free-motion  */
  uint16_t        min_samples;            /* fit gate (accel/gyro/mag specific) */
  /* sensor-supplied I/O — the ONLY sensor coupling: */
  bool (*read_raw)(float v[3], void *ctx);   /* pop one raw 3-vector; false=none */
  bool (*sample_valid)(void *ctx);           /* optional gate (mag rhall); NULL ok */
  void (*commit)(const float offset[3], const float mat[9], void *ctx);
  void           *ctx;
} calib_target_t;

void calib_engine_run(const calib_target_t *t);  /* called by the calib task */
```
`bmx160.c` shrinks to a **consumer**: it keeps only the sample diversion
(`imu_queue_calibration`), the `bmx160_calibration_t` store, and three small
descriptors whose `read_raw` returns `acc_raw` / `gyr_raw` / `mag_compensated`
and whose `commit` writes the store + enqueues the save. **Mag is migrated to a
descriptor too** (behaviour-identical) to prove the abstraction; accel/gyro use
the new pose-tolerant ellipsoid + bias paths. A future sensor adds one descriptor.

### Build wiring
Add `src/calib/calib_ellipsoid.c` + `calib_engine.c` to `CMakeLists.txt` and
`tools/sim_host/CMakeLists.txt`; remove the moved bodies from `bmx160.c`.

---

## 6. Phasing (each step independently verifiable)

1. **Extract `calib_ellipsoid` + host test.** Move the solver/fit out verbatim;
   add `tools/sim_host/tests/test_calib_ellipsoid.c` — synth static vectors at many
   random orientations with a *known* injected offset+3×3 (and noise); assert
   recovery and corrected `‖v‖ = radius ± tol`; under-coverage → no commit. Locks
   the math with zero hardware. (Behaviour-preserving — mag still uses it.)
2. **Extract `calib_engine`; migrate mag to a descriptor.** Pure refactor, no
   behaviour change — verify mag calibration still works in SITL/bench.
3. **New accel path (true full 3×3, ~12 poses)** via the engine: relaxed capture,
   accumulate-across-holds, ellipsoid fit, `acc_soft_iron[9]`, `cal.bin` v3,
   matrix apply. Adds the edge/corner `CALIB_UPDATE_*` codes in `comm_types.h`.
4. **GCS wizard** — edge/corner pose illustrations + ~12-step flow; update
   `tst_calibration_wizard.cpp`. (Pairs with step 3's new codes.)
5. **Gyro stillness-gate (selector 2, separate routine)** as a `CALIB_FIT_BIAS`
   target on the engine.
6. **On-hardware bring-up.** Run the ~12-pose accel flow with deliberately loose
   poses; verify via the UDP harness that static `‖a‖→g` in several orientations
   and gyro axes centre near 0.

---

## 7. Verification

- **Unit (host):** `test_calib_ellipsoid` recovery — offset/3×3 recovered within
  tolerance; corrected `‖a‖=g±0.05` across 50 random orientations incl. deliberate
  ±15° pose error; degenerate/under-covered input → no commit.
- **Refactor regression:** mag calibration via the migrated descriptor is
  bit-equivalent to today (SITL + bench).
- **SITL:** ideal IMU feed → accel fit converges to ~identity (offset≈0, soft≈I),
  estimator unperturbed — a regression guard via the headless harness.
- **On hardware (the real proof):** before/after with the UDP collector — static
  `|a|` moves ~10.2 → `9.81 ± 0.1` in arbitrary orientations, gyro biases drop
  toward 0 (re-measure the live `0.45 °/s`). Confirm it completes from
  deliberately sloppy poses across the full ~12-pose flow (the whole point).
- **Acceptance (operator):** post-calibration `‖a‖=g±0.1` any orientation; gyro at
  rest `<~0.1 °/s`.

---

## 8. Risks / edge cases

- **Full-3×3 conditioning.** The ~6 edge/corner holds make the off-diagonal terms
  observable; light Tikhonov regularisation on `S` is a backstop for a missed/sloppy
  edge (degrades toward diagonal). The `‖a‖=g` acceptance test catches a bad matrix.
- **More poses = longer procedure / operator fatigue.** ~12 holds vs 6 doubles the
  time; mitigate by accepting a hold the instant stillness + min-samples are met
  (no fixed dwell) and letting the operator end early once coverage is sufficient.
- **Stillness band vs uncalibrated scale.** A badly mis-scaled accel can read `|a|`
  outside the tight `ACC_STILL` 0.2 band, so use the **wide** `0.45g..2g` band for
  *capture* qualification (already at `:1572-1573`); stillness here leans on the
  **gyro** test. Keep the tight band only for the runtime auto-trim.
- **`cal.bin` v2→v3.** Handled by the existing magic/version/size reject → identity
  defaults; operator recalibrates once. No GCS/NavLink impact.
- **Module extraction churn.** Do it behaviour-preserving first (Phases 1–2, mag
  unchanged) before adding the new accel maths (Phase 3), so a regression is
  bisectable to one phase.
- **`acc_scale` removal.** Audit all readers of `acc_scale` before deleting it
  (telemetry/logging) — keep it as the soft-iron diagonal if anything depends on it.

---

## 9. Decisions to confirm

- **D1 — Replace with full 3×3 (no diagonal interim).** `acc_soft_iron[9]`,
  `cal.bin` v3, matrix apply mirroring mag. *(user-directed)*
- **D2 — ~12 prompts (6 faces + ~6 edges/corners); relax acceptance to static +
  wide band; fit is magnitude-only so prompts are advisory.** NavLink unchanged;
  GCS wizard updated for the new prompts. *(user-directed)*
- **D3 — Keep accel and gyro as separate explicit routines** (selector 1 /
  selector 2); gyro is stillness-gated; no folding. *(user-directed)*
- **D4 — Extract a shared `src/calib/` module** (engine + ellipsoid) with a
  per-sensor descriptor; migrate mag onto it as the proof. *(user-directed)*
- **D5 — Pose count: ~12 (6 faces + ~6 edges/corners) for a well-conditioned full
  3×3** (user-chosen). Exact edge set (cover each of XY/XZ/YZ at least twice) +
  `MAG_FIT_MIN_SAMPLES`-style gates *(confirm numbers on bench)*.

---

## 10. Critical files

- **New:** `src/calib/calib_ellipsoid.{c,h}`, `src/calib/calib_engine.{c,h}`;
  `tools/sim_host/tests/test_calib_ellipsoid.c` (+ accel-recovery cases).
- `src/sensor/bmx160.c` — becomes a consumer: remove the solver/fit
  (`:1473-1506`), `calibration_task`/`wait_for_orientation` (`:1534-1659`,
  `:1689+`), accel/gyro/mag branches (`:1718-1905`) → engine + three descriptors;
  change accel apply `:1291-1292` to matrix-multiply (cf. mag `:1080-1082`); keep
  sample diversion + the calib store.
- `include/sensor/bmx160.h` — add `acc_soft_iron[9]` to `bmx160_calibration_t`
  (`:251-257`); bump `CALIB_FILE_VERSION 2→3` (`:263`).
- `include/variables.h` — accel fit gates (min holds/samples) alongside
  `:272-276`; `ACCEL_SCALE_MIN_SPAN` retired with the diagonal model.
- `include/comm/comm_types.h` — add ~6 edge/corner `CALIB_UPDATE_*` pose codes
  (`:74-97`); the FC prompt sequence + GCS interpretation share this enum.
- `software/src/core/CalibrationWizard.cpp`,
  `software/src/ui/widgets/CalibrationWidget.cpp`,
  `software/tests/tst_calibration_wizard.cpp` — edge/corner illustrations + ~12-step
  accel flow.
- `navlink/**` — **unchanged** (no dialect/codec change; pose codes ride the
  existing calibration message buffer).
- `CMakeLists.txt` + `tools/sim_host/CMakeLists.txt` — add `src/calib/*.c`.
