# On-hardware test firmware + two-tier coverage

Implementation design for **Suite B** (the on-MCU bench tests) of
[`hardware-test-suites.md`](hardware-test-suites.md), plus the **coverage**
dimension that plan does not address. This doc is the concrete build-out; the
parent doc's locked decisions (separate test firmware, NavLink test dialect in
the `0x800000+` range, auto-skip, `test onhw`/`test hw`) still hold and are not
re-litigated here.

Three things change relative to the parent plan:

1. **gcov-like coverage**, split into two tiers (host + on-target).
2. **SD-card result/coverage dump** as a first-class path, not just a fallback —
   the on-target `.gcda` blob is too big to stream reliably over WiFi UDP, so it
   rides the (now byte-perfect) file-transfer substrate instead.
3. **Single dialect, test messages flagged out of non-test builds**
   (supersedes parent decision #3, "separate `test_dialect.json`"). One
   `navlink/dialect.json` stays the source of truth; test messages carry a
   `"test": true` flag and are emitted only under a test codegen profile, so the
   production codec still contains zero test code — same bloat guarantee, no
   second file to keep in sync. See §5.

Status of the world this builds on (verified 2026-06-28):
- On-hw bench firmware: **not started** — no `firmware/tests/`, no `VAYU_HW_TEST`
  CMake option, no test dialect, no runner. Greenfield.
- Host unit tests: `sim/host/tests/*` (15 files) run on the host build.
- Host coverage: **already exists** on the `analysis/test-coverage` worktree
  (`tools/coverage_gate.py`, `docs/testing/coverage-analysis.md`, per-component
  floors via `gcovr --json-summary`). Tier 1 below adopts it; it is not rebuilt.
- Toolchain: `arm-none-eabi-gcc 13.2.1` — ships `__gcov_info_to_gcda`,
  `__gcov_filename_to_gcfn`, `__gcov_dump` in `libgcov.a` and accepts
  `-fprofile-info-section`. This is what makes freestanding gcov tractable.
- Target: STM32F401RD, FLASH 512K @ 0x08000000, **RAM 96K @ 0x20000000**
  (`firmware/linker.ld`); `.bss` ends at `_heap_start` (see
  `memory/f401-sram-heap-overflow.md`). Coverage counters land in `.bss` → the
  RAM ceiling is the governing constraint for tier 2.

---

## 1. Two-tier coverage model

| Tier | Measures | Where it runs | Mechanism | Status |
| --- | --- | --- | --- | --- |
| **1 — host** | All *portable* logic: estimators, control law, mixer, calib math, parsers, xfer state machine, FS owner logic | Host (`sim/host` + `sim/host/tests`) | Standard `gcc --coverage` + `gcovr` | exists on `analysis/test-coverage`; merge to main |
| **2 — on-target** | *Hardware-only* code that cannot run on host: I2C/DMA, BMX160/BME280 drivers, ESC/PWM, SDIO/FatFS, RTOS timing glue | The `hwtest` firmware on a real FC | freestanding gcov: `-fprofile-info-section` + `__gcov_info_to_gcda` → `.gcda` to SD → host `gcov-tool merge-stream` | new (this doc) |

**Why split.** The firmware compiles identically on host and target (the
`EKF_SELFTEST` precedent). For anything that *does* compile on host, host gcov is
free, accurate (`-O0`, no RAM limit), and already wired. On-target gcov is
expensive (64-bit counter per arc, in scarce RAM; needs `-Og`; not
timing-representative), so it earns its keep only for the driver/bus/RTOS code
that genuinely needs the metal. The two reports are stitched at the end into one
`gcovr` view, deduplicated by source path (host wins where both cover a file).

**Coverage of the *firmware under test*, not of the test harness.** Both tiers
instrument production sources only; test files and the bench runner are excluded
from the report (`gcovr --exclude`).

---

## 2. Tier 1 — host coverage (adopt, don't rebuild)

The `analysis/test-coverage` branch already does this:
- Host build compiled with `--coverage`; `sim/host/tests/*` exercised it; `gcovr
  --json-summary` feeds `tools/coverage_gate.py`, which enforces a per-component
  **floor ratchet** (e.g. `est` 70 %, `control` 17 % and rising). Floors only go
  up; a regression fails CI.

Work here is integration, not invention:
1. Merge `tools/coverage_gate.py` + `docs/testing/coverage-analysis.md` to main.
2. Add a `vayu.sh test cov` (or `--coverage` flag on `test host`) that builds the
   host suite with `--coverage`, runs it, emits `cov.json`, and runs the gate.
3. Wire the gate into CI (it already exists as a script; just call it).

This tier is the bulk of real coverage and should land **first** (it unblocks the
ratchet immediately and is zero-hardware).

---

## 3. Tier 2 — on-target gcov (the new, hard part)

### 3.1 Mechanism (freestanding gcov, GCC 13)

1. **Instrument selected TUs** with
   `-fprofile-arcs -ftest-coverage -fprofile-info-section -fprofile-update=atomic -Og`.
   - `-fprofile-info-section` puts each TU's `struct gcov_info` into a `.gcov_info`
     section instead of registering it via a startup constructor that would call
     the hosted `__gcov_init`. We walk the section ourselves.
   - `-fprofile-update=atomic` because vaios is preemptive and counters are touched
     from multiple tasks/ISRs (Cortex-M4 LDREX/STREX; libgcc provides the atomics).
     Without it, concurrent arc hits race and undercount.
   - `-Og` keeps line/arc mapping faithful while staying smaller than `-O0`.
2. **Linker script** (`firmware/linker_hwtest_cov.ld`, derived from `linker.ld`):
   bracket the section so we can iterate it —
   ```
   .gcov_info : {
     PROVIDE(__gcov_info_start = .);
     KEEP(*(.gcov_info .gcov_info.*))
     PROVIDE(__gcov_info_end = .);
   } > FLASH
   ```
   The `gcov_info` structs sit in FLASH (their counter pointers are link-time
   constants); the **counter arrays** (`__gcov0.*`, zero-init) go to `.bss` → RAM.
3. **Dump routine** (`firmware/tests/onboard/coverage_dump.c`):
   ```c
   extern const struct gcov_info *__gcov_info_start;
   extern const struct gcov_info *__gcov_info_end;
   /* libgcov, GCC>=12: stream .gcda bytes through callbacks, no fopen */
   extern void __gcov_info_to_gcda(const struct gcov_info *,
        void (*filename)(const char*, void*),
        void (*dump)(const void*, unsigned, void*),
        void *(*allocate)(unsigned, void*), void *arg);
   extern void __gcov_filename_to_gcfn(const struct gcov_info *,
        void (*dump)(const void*, unsigned, void*), void *arg);
   ```
   For each `info` between the two symbols: emit `__gcov_filename_to_gcfn(...)`
   (the per-TU **gcfn** record = source filename) immediately followed by
   `__gcov_info_to_gcda(...)` (the **gcda** record = counters). `dump` appends
   bytes to the sink; `allocate` hands back a small static scratch buffer.
4. **Sink = SD file** `0:/cov/run.gcfn_gcda` (one concatenated stream). The dump
   runs once at end-of-suite, from the FS-owner task context (so it shares the
   single-owner SD transaction — see `LOG-OWN-001`). UART streaming is supported
   too (same callback writing to the test dialect) but SD is primary for size.
5. **Host reconstruct.** `arm-none-eabi-gcov-tool merge-stream -o build_hwtest_cov
   < run.gcfn_gcda` consumes exactly the `gcfn+gcda` stream `libgcov` produced and
   writes a `.gcda` tree; then `gcovr` against `build_hwtest_cov` (which holds the
   matching `.gcno` from the instrumented compile) emits the report.

This is the GCC-intended freestanding pipeline; we write ~60 lines of device code
and one host invocation, not a custom .gcda serializer.

### 3.2 RAM budget — the governing constraint

Counters are 8 bytes × (arcs in instrumented functions), in `.bss`. The whole
firmware instrumented at once will not fit under `_heap_start + HEAP_SIZE <=
0x20018000`. So tier 2 is **module-scoped**:
- A CMake list `VAYU_HW_TEST_COV_MODULES` selects which source files compile with
  `--coverage`; everything else stays `-O2`, uncounted.
- One coverage run covers one module group (e.g. `sensor` drivers, then
  `actuator`, then `storage`). The host stitches successive runs' `.gcda` trees.
- The runner logs free-heap high-water (`HEAP_WATERMARK_ENABLE`) each run and
  refuses to start a group that would breach the ceiling — a measured gate, not a
  guess.

### 3.3 Build variants

Three images from the same `firmware/` tree, selected by CMake options:

| Image | Options | Opt | Purpose |
| --- | --- | --- | --- |
| `main` | (none) | -O2 | production firmware (unchanged) |
| `hwtest` | `-DVAYU_HW_TEST=ON` | -O2 | bench checks, NavLink results, timing-representative |
| `hwtest_cov` | `-DVAYU_HW_TEST=ON -DVAYU_HW_TEST_COV=ON -DVAYU_HW_TEST_COV_MODULES=...` | -Og + `--coverage` on the selected modules | coverage run; dumps `.gcda` to SD |

Production `main` never sees `VAYU_HW_TEST`, so zero flash/RAM bloat on the real
FC (parent-plan decision #2). The two test images are built into
`build_hwtest/` / `build_hwtest_cov/` so their `.gcno` stay matched to the binary.

---

## 4. On-hardware test firmware

### 4.1 Layout
```
firmware/tests/onboard/
  hwtest_main.c        # entry: bring up clock/HAL/RTOS, run registry, report, idle in STANDBY
  hwtest_runner.{c,h}  # check registry + result tally + reporting seam (NavLink | SD)
  coverage_dump.{c,h}  # tier-2 .gcda dump (built only with VAYU_HW_TEST_COV)
  checks/
    check_boot_os.c    # clock freq, STANDBY reached, heap/stack watermarks, sched/ISR
    check_bus_i2c.c    # blocking + DMA read, unstick, reg readback, CRC, DWT
    check_sensors.c    # BMX160/BME280 WHO_AM_I, I2C scan, stillness, baro plausibility
    check_io.c         # USART6 loopback, iBus parse, PWM duty/GPIO (props-off gated)
    check_est.c        # wraps ekf_selftest_run() as one bench item
    check_persist.c    # SD/FatFS write->readback CRC, calib save/restore
```
The check files reuse the **production** drivers/RTOS as-is (that is the point —
we test the real code), so the same sources appear in tier-2 coverage.

### 4.2 Check registry
A check is `{ const char *name; hw_result_t (*fn)(void); }`. `hwtest_runner` walks
a static array, emits `HW_TEST_BEGIN{total}`, then per check
`HW_TEST_RESULT{id,name,pass,value,units}`, then `HW_TEST_DONE{passed,failed,
skipped}`. `value`/`units` carry the measured number (clock MHz, heap bytes, gyro
σ, baro Pa) so the host records trends, not just pass/fail. Mirrors the
`EKF_SELFTEST` report shape, scaled to a registry.

### 4.3 Entry & safety
- Runs **after HAL/RTOS init but the vehicle never leaves STANDBY** and never
  arms; motors emit no PWM except the explicit, `VAYU_HW_PROPS_OFF=1`-gated
  duty/GPIO check (parent plan, Safety). Disarm→0 is itself an asserted check.
- Drops production tasks not needed for the bench (telemetry/control loops) to
  free RAM for the coverage counters (parent plan, F401 RAM note).

---

## 5. Result + coverage transport (the hybrid)

| Payload | Size | Primary path | Fallback |
| --- | --- | --- | --- |
| Pass/fail results | ~32 B/check | **NavLink test dialect** (live, `HW_TEST_*`), ESP UDP relay | also written to `0:/cov/hwtest.log` on SD |
| Coverage `.gcda` stream | KB–tens of KB | **SD file** `0:/cov/run.gcfn_gcda` | n/a (always SD) |

- **Results** stream live over the test dialect so the runner shows progress and a
  WiFi link is enough. The runner *also* finds a `hwtest.log` on SD, so a board
  with no link (or a dropped one) still yields results on the next download — this
  is the user's "dump to SD and download" path, made the guaranteed record.
- **Coverage** is never streamed live (too big for lossy UDP); it is written to SD
  and pulled with the existing **file-transfer substrate** (byte-perfect to 64 KB,
  `memory/sdio-multitask-corruption.md`), then fed to `merge-stream`.

### Test NavLink dialect — single JSON, flagged messages
The test messages live in the **same** `navlink/dialect.json`, each carrying a
`"test": true` flag, in the reserved `0x800000+` vendor/test msgid range:
`HW_TEST_BEGIN{total:u16}`, `HW_TEST_RESULT{id:u16,name:char[24],pass:u8,
value:f32,units:char[8]}`, `HW_TEST_DONE{passed:u16,failed:u16,skipped:u16}`.

`navlink/generate.py` changes (small, localized):
- **Profile flag.** `--profile prod|test` (default `prod`). `prod` filters out any
  message with `"test": true` *before* emitting C/Python/HTML, so the production
  codec (`navlink/generated/`) is byte-for-byte unchanged and contains no test
  structs, dispatch entries, or CRC-extra rows. `test` includes them; the `hwtest`
  build and `tools/hwtest/run.py` invoke that profile into their own out dirs
  (e.g. `build_hwtest/navlink_test/`), never overwriting the production codec.
- **Validator.** `validate()` today rejects any msgid outside `0x000000-0x7FFFFF`
  (spec §9 core half). Relax it so a `"test": true` message MAY sit in the
  `0x800000-0xFFFFFF` vendor/test half (and must — a test message in the core
  range is an error), while non-test messages stay confined to the core half.
  Uniqueness/field-index checks apply to both.
- **CRC-extra / dispatch.** Because filtering happens on the message list up front,
  every downstream table (CRC-extra, size, dispatch) is naturally test-free in the
  `prod` profile with no per-emitter special-casing.

Both profiles run in the codegen gate (regenerate + diff) so C and Python can't
drift and so a stray `"test"` message can never leak into the production codec.

---

## 6. Host runner — `tools/hwtest/run.py`

```
run.py [--cov MODULES] [--props-off]
  1. build       cmake -S firmware -B build_hwtest  [-DVAYU_HW_TEST_COV=ON -DVAYU_HW_TEST_COV_MODULES=...]
  2. flash       reuse tools/scripts/flash.sh flow (st-flash --connect-under-reset)
  3. link        vayu_headless.transport.live (UDP first, serial second) -- fc_available() gate
  4. collect     HW_TEST_BEGIN/RESULT/DONE -> tally; if no link, download 0:/cov/hwtest.log via xfer
  5. coverage    (if --cov) download 0:/cov/run.gcfn_gcda via xfer
                 arm-none-eabi-gcov-tool merge-stream -o build_hwtest_cov < run.gcfn_gcda
                 gcovr build_hwtest_cov --json-summary > cov_target.json
  6. report      pytest-style "N passed, M failed, K skipped" (so vayu.sh parse_pytest applies)
                 + feed cov_target.json into the SAME coverage_gate.py (target-tier floors)
```
`vayu.sh` integration (`test onhw`, `test hw`, fold into `test all` with
auto-skip) is unchanged from the parent plan.

---

## 7. Traceability hook

This directly closes the `@verifies` gap from the just-landed traceability work
(`firmware/docs/reference/trace.md` shows 0/168 requirements verified). Each bench check
and each host test carries `@verifies <MOD-SUB-NNN>`; `tools/dev/trace.py` already
parses `@verifies` and will populate the Verifiers column. Natural first targets:
the HW-only requirements that *only* a bench test can verify — `SNS-BMX-*`,
`SNS-I2C-*`, `ACT-*`, `HAL-*`, `LOG-SD-*`, `SNS-BARO-*`. Coverage (tier 1/2) and
traceability (`@verifies`) are complementary: one says "this line ran under test",
the other "this requirement has a test".

---

## 8. Phasing

Extends the parent plan's P3/P4 and adds the coverage tiers.

- **C0 — host coverage (tier 1).** Merge `coverage_gate.py` + analysis doc to main;
  add `vayu.sh test cov`; wire CI. Zero hardware, immediate ratchet. *(do first)*
- **C1 — bench skeleton (parent P3).** `firmware/tests/onboard/` + `VAYU_HW_TEST`
  CMake target → `hwtest.elf`; test dialect codegen; `run.py` flash+collect; first
  checks (boot/STANDBY, sensor `WHO_AM_I`, wrap `ekf_selftest_run`). SD `hwtest.log`
  mirror. First green/skip on real hardware.
- **C2 — bench matrix (parent P4).** I2C/DMA, IMU noise/bias, baro, UART loopback,
  RC parse, PWM-safe, scheduler/heap/stack, flash/SD, calibration. `@verifies` tags
  as each check lands.
- **C3 — on-target coverage (tier 2).** `-fprofile-info-section` build variant,
  `linker_hwtest_cov.ld`, `coverage_dump.c`, SD `.gcda`, `merge-stream` +
  `gcovr` in `run.py`, RAM-budget gate. Module-scoped, starting with `sensor`.
- **C4 — stitch + CI.** Merge host + target `gcovr` reports into one view; add
  target-tier floors to `coverage_gate.py`; docs in `build.md` + a HW-setup guide;
  optional self-hosted HW runner (default CI skips).

---

## 9. Risks / open items

- **RAM ceiling (tier 2).** Whole-firmware on-target coverage will not fit 96 K;
  module-scoping + the measured heap-watermark gate are mandatory, not optional.
  If a single module's counters still overflow, fall back to function-subset
  instrumentation (per-file `--coverage`).
- **`-Og` vs `-O2` skew.** The coverage image is not timing-representative; keep
  timing/jitter asserts in the `-O2` `hwtest` image and treat `hwtest_cov` as a
  separate, coverage-only run. Do not gate timing on the instrumented build.
- **Counter races.** `-fprofile-update=atomic` is required under the preemptive
  RTOS; verify libgcc supplies the M4 atomic helpers (it does for v7-m) or accept
  documented undercount with `single`.
- **`merge-stream` availability.** Needs `gcov-tool` from the *same* GCC major as
  the firmware compiler (13.x). Pin it; mismatched gcov versions reject the stream.
- **Flash-in-CI.** `test onhw`/coverage reflash the FC; gate behind
  `VAYU_HW_ALLOW_FLASH=1` (parent plan) so an attached dev board isn't reflashed by
  a bare `test all`.
- **Dialect drift / leakage.** Both codegen profiles (`prod`, `test`) regen in the
  gate; the `prod`-profile diff must stay empty when test messages are added,
  proving no test code leaks into the production codec.
- **Safety interlocks.** PWM/ESC checks require `VAYU_HW_PROPS_OFF=1` or interactive
  confirm before running.

---

## 10. First concrete step

C0 (host coverage merge) is hardware-free and unblocks the ratchet today, so it
leads. C1 (bench skeleton) is the first thing that runs on the metal and proves
the NavLink-results + SD-mirror loop end-to-end before the matrix or tier-2
coverage is built on top. Recommend landing C0 then C1, reviewing real output,
before committing to C2/C3 scope.
