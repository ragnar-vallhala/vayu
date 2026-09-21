# Building & testing the Vayu stack

The repo is a monorepo of independent components. **There is no root
`CMakeLists.txt`** — each component is configured from its own directory into its
own out-of-source build tree. This file is the authoritative map of *what builds
how*; the convenience wrappers in `tools/scripts/` (see [§7](#7-scripts)) drive
the same commands.

```
firmware/    ARM flight-controller firmware  (target: main ELF)
sim/host/    SITL host seam                   (libvayu_sitl, vayu_sitl_rtos, ctest suite)
sim/vsim/    physics the SITL links           (no target of its own)
navlink/     wire-protocol codec, submodule  (no build — codegen, consumed by the others)
extern/vaios git submodule (RTOS kernel + NavHAL drivers)

The ground station is NOT built here. It lives in its own repository and
consumes this one's SITL SDK release rather than any of its source:
https://github.com/ragnar-vallhala/vayu-navigator
```

---

## 0. Prerequisites

| Component | Needs |
| --- | --- |
| firmware | `arm-none-eabi-gcc` (+ `binutils`), `cmake` ≥ 3.20, `python3` + `kconfiglib`, `st-flash` (flashing only) |
| sim/host, sim/vsim | host `gcc`/`g++` (C++17), `cmake`, `python3` |
| sim/host SDK | nothing extra — `cmake --build build_sitl_rtos --target vayu_sitl_package` packages it |

The `extern/vaios` submodule must be present: `git submodule update --init --recursive`.

> **Windows:** the GCS builds via MSYS2/UCRT64 with `-DNAVIGATOR_SITL=OFF` (SITL +
> autotune are POSIX-only). See `firmware/docs/reference/windows-setup.md`.

---

## 1. Firmware (`firmware/` → ARM `main`)

```sh
cmake -S firmware -B build -DNAVHAL=ON -DEXTERNAL_LINKER=ON
cmake --build build -j                 # -> build/main (ELF), prints size
cmake --build build --target flash     # objcopy -> build/main.bin, st-flash via SWD
```
or simply `bash tools/scripts/build.sh` (build) / `bash tools/scripts/flash.sh` (build + flash).

**Board:** `nucleo_f401re` (STM32F4, Cortex-M4, hard-float). **Options:**

| Option | Default | Effect |
| --- | --- | --- |
| `USE_STANDARD_MATH` | ON | use `math.h` backend vs the firmware math lib |
| `EKF_SELFTEST` | OFF | run the EKF branch-coverage self-test at boot, report over telemetry UART |
| `VAYU_SIM` | OFF | enable host-SITL hooks (RC override path) — set by the SITL build, not for flashing |

`firmware/CMakeLists.txt` defines `VAYU_REPO_ROOT` (= `..`) and prefixes only its
**outward** refs (`extern/vaios`, `navlink/`) with it; everything firmware-internal
is `CMAKE_CURRENT_SOURCE_DIR`-relative. The NavLink C codec is regenerated at
configure time (see [§6](#6-navlink-codegen)).

---

## 2. SITL host seam (`sim/host/`)

Compiles the **real firmware control code** (`firmware/src/...`) for the host with
shim ports, so it runs without hardware.

```sh
cmake -S sim/host -B build_sitl
cmake --build build_sitl -j
ctest --test-dir build_sitl --output-on-failure    # host unit-test suite
```

**Targets:** `vayu_sitl_core` (static lib — the firmware logic; the host tests link
this), `vayu_sitl` (standalone host binary), `navlink_codec`, and the test
executables. **Options:**

| Option | Effect |
| --- | --- |
| `VAYU_SANITIZE=ON` | ASan + UBSan over the core + tests |
| `VAYU_COVERAGE=ON` | gcov instrumentation; then `cmake --build build_cov --target coverage` for a `gcovr` summary |
| `VAYU_SITL_RTOS_BUILD=ON` | also build `vsim_phys` + `vayu_sitl_rtos` — runs the real vaios scheduler on host (links `sim/vsim` physics in-process; ~57×, deterministic) |

Tests (ctest names): `safety_phase2`, `phase3_ctrl`, `phase3_comm`, `phase3_slog`,
`fs_owner`, `xfer_sm`, `xfer_e2e`, `xfer_providers`, `fs_query`, `calib_ellipsoid`,
`calib_engine`, `phase3_est_ekf`, `vertical_est`, `flight_phase`, `phase0_assert`.

---

## 3. Physics (`sim/vsim/`)

No build of its own. `sim/host` compiles `sim_controller.cpp`,
`physics_core.cpp`, `motor_model.cpp` and `sensor_models.cpp` into the SITL's
`vsim_phys` library, so the physics is built as part of §2 and nothing here
produces a separate target.

The standalone `vsim_d` daemon this section used to document was deleted in the
2026-07 consolidation, along with the FIFO pair it spoke over: one in-process
stepper runs firmware and physics together now.

---

## 4. Ground-control station — not built here

The GCS moved to its own repository and no longer builds from this tree:

    https://github.com/ragnar-vallhala/vayu-navigator

It needs none of this source. It compiles against four headers and loads the
engine at runtime from the **SITL SDK** this repo publishes as a release
asset, so the two version independently.

### 4a. Publishing the SDK the GCS consumes

```sh
cmake -S sim/host -B build_sitl_rtos -DVAYU_SITL_RTOS_BUILD=ON
cmake --build build_sitl_rtos --target vayu_sitl_package -j   # -> vayu-sitl-sdk.tar.gz
```

Contents, and the whole surface a host depends on:

| Path | What |
| --- | --- |
| `lib/libvayu_sitl.so` | the engine as a loadable module (one exported symbol, ABI-versioned) |
| `bin/vayu_sitl_rtos` | the same engine as a headless binary, driven over FIFOs |
| `include/` | `vayu_sitl_abi.h`, `vsim_proto.h`, `trimesh_bvh.h`, `vsim_math.h` |
| `BUILD_ID` | `git describe` + date — which firmware this is |

CI publishes it twice: a rolling `sitl-sdk-latest` prerelease on every push to
main, and a permanent versioned asset on a `v*` tag. Locally, `--target
vayu_sitl_package` produces the identical tarball, deliberately — packaging
that only ever runs in CI is packaging nobody can debug.

---

## 5. Build-directory conventions

All build trees are out-of-source and git-ignored. Conventional names:

| Dir | Component / mode |
| --- | --- |
| `build/` | firmware (ARM) |
| `build_sitl/` | sim/host + tests |
| `build_san/`, `build_cov/`, `build_tidy/` | sim/host sanitizer / coverage / tidy |
| `build_sitl_rtos/` | sim/host RTOS variant |
| `*-docker/` | docker builds (never clash with host-native) |

---

## 6. NavLink codegen

`navlink/dialect.json` is the single source of truth for the wire protocol. The
firmware, sim/host, and navigator CMake each run `navlink/generate.py --lang c`
into their build tree at **configure time** (nothing generated is committed, so it
can never drift). The Python codec is generated similarly for the headless SDK /
tools. No standalone build step.

---

## 7. Scripts

`tools/scripts/` holds the convenience wrappers (they locate the repo root
themselves, so run from anywhere). **`vayu.sh` is the single dispatcher** for
every component:

```sh
tools/scripts/vayu.sh build  <firmware|sitl|vsim|gcs|rtos|all>   [opts]
tools/scripts/vayu.sh test   <sitl|gcs|headless|all>            [opts]
tools/scripts/vayu.sh flash                                     # firmware build + st-flash
tools/scripts/vayu.sh clean                                     # remove all build trees
```

Options: `-j N` (jobs), `--release` / `--debug` (build type), `--no-sitl`
(gcs: `-DNAVIGATOR_SITL=OFF`), `--sanitize` / `--coverage` (sitl). Examples:
`vayu.sh build all`, `vayu.sh build gcs --no-sitl`, `vayu.sh test sitl --coverage`,
`vayu.sh test all`.

| Wrapper | Does |
| --- | --- |
| `vayu.sh` | dispatcher (above) — the main entry point |
| `build.sh` | shim → `vayu.sh build firmware` |
| `flash.sh` | firmware build + objcopy + `st-flash` |

Notes: `test sitl` (re)builds `build_sitl` incrementally, then `ctest` (the host
suite); `test gcs` builds the QtTest binaries and runs only the navigator `tst_*`
tests (the host suite leaks in via `add_subdirectory(sim/host)` — run it through
`test sitl`); the SDK's
integration tests drive the real SITL stack — without the binaries `conftest`
*silently skips* them), creates/uses `.venv`, installs the SDK editable, and runs
pytest with `VSIM_BIN_PATH`/`VAYU_SITL_BIN` pointed at those fresh binaries (so a
stale `sim/*/build` copy is never picked up); `build all` = firmware + sitl +
vsim + gcs. The `ensure_*_build` steps always do an incremental build, so a test
run can't serve a stale binary.

Other entry points: `tools/docker/build.sh {image,firmware,sitl,gcs,all,shell,clean}`
(containerised builds into `*-docker/`), and `tools/dev/trace.py --check` (the
traceability gate).

---

## 8. CI (`.github/workflows/ci.yml`)

Every push / PR runs, each job mirroring a command above:

| Job | Command |
| --- | --- |
| `build-target` | firmware `cmake -S firmware -B build` (arm-none-eabi, `-Werror`) |
| `trace-gate` | `python3 tools/dev/trace.py --check` |
| `cppcheck` | `cppcheck -I firmware/include firmware/src/` |
| `sitl-tests` | `cmake -S sim/host -B build_sitl` + `ctest` |
| `sanitizers` | `cmake -S sim/host -B build_san -DVAYU_SANITIZE=ON` + `ctest` |
| `coverage` | `cmake -S sim/host -B build_cov -DVAYU_COVERAGE=ON` + `coverage` target |
| `clang-tidy` | `build_tidy` over `git ls-files 'firmware/src/*.c'` |

The GCS is **not** yet in CI (it needs Qt6 + assimp on the runner).
