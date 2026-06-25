# Building & testing the Vayu stack

The repo is a monorepo of independent components. **There is no root
`CMakeLists.txt`** — each component is configured from its own directory into its
own out-of-source build tree. This file is the authoritative map of *what builds
how*; the convenience wrappers in `tools/scripts/` (see [§7](#7-scripts)) drive
the same commands.

```
firmware/    ARM flight-controller firmware  (target: main ELF)
navigator/   Qt6 ground-control station      (target: Navigator) + headless-sdk (Python)
sim/host/    SITL host seam                   (libvayu_sitl_core, vayu_sitl, ctest suite)
sim/vsim/    standalone physics daemon        (vsim_d)
navlink/     wire-protocol codec             (no build — codegen, consumed by the others)
extern/vaios git submodule (RTOS kernel + NavHAL drivers)
```

---

## 0. Prerequisites

| Component | Needs |
| --- | --- |
| firmware | `arm-none-eabi-gcc` (+ `binutils`), `cmake` ≥ 3.20, `python3` + `kconfiglib`, `st-flash` (flashing only) |
| sim/host, sim/vsim | host `gcc`/`g++` (C++17), `cmake`, `python3` |
| navigator | host C++ compiler, `cmake`, **Qt6** (Core/Widgets/Network/OpenGL/Concurrent/SerialPort/Test), **assimp** (only when SITL is ON) |
| headless-sdk | `python3` (pip-installable package; tests need `pytest`) |

The `extern/vaios` submodule must be present: `git submodule update --init --recursive`.

> **Windows:** the GCS builds via MSYS2/UCRT64 with `-DNAVIGATOR_SITL=OFF` (SITL +
> autotune are POSIX-only). See `docs/reference/windows-setup.md`.

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

**Targets:** `vayu_sitl_core` (static lib — the firmware logic; Navigator links
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

## 3. Physics daemon (`sim/vsim/` → `vsim_d`)

```sh
cmake -S sim/vsim -B build_vsim
cmake --build build_vsim -j           # -> build_vsim/vsim_d
```
Standalone C++17, zero external deps. Rigid-body + sensor/motor models; talks to
the host seam / GCS over `/tmp/vsim_*` FIFOs (wire: `sim/vsim/include/vsim_proto.h`).
Has its own ad-hoc tests under `sim/vsim/tests/` (g++ one-liners + python drivers).

---

## 4. Ground-control station (`navigator/` → `Navigator`)

```sh
cmake -S navigator -B navigator/build -DCMAKE_BUILD_TYPE=Release
cmake --build navigator/build -j                              # -> navigator/build/Navigator
ctest --test-dir navigator/build --output-on-failure         # QtTest suite (headless, offscreen)
```

**Options:**

| Option | Default | Effect |
| --- | --- | --- |
| `NAVIGATOR_SITL` | ON | build the in-app simulator + autotune; links `vayu_sitl_core` + `assimp` from `sim/host`. **OFF** = GCS-only (Windows / no-POSIX) |
| `NAVIGATOR_BUILD_TESTS` | ON | build the `tst_*` QtTest binaries under `navigator/tests/` |

A good configure prints `Navigator: linking against vayu_sitl_core + assimp (SITL ON)`
or `Navigator: SITL OFF (...) -- GCS-only build`. Tests run with
`QT_QPA_PLATFORM=offscreen` (no display needed). Test names: `tst_time_sync_estimator`,
`tst_command_codec`, `tst_autotune_gains`, `tst_optimizer`, `tst_cost`, `tst_sysid`, …

### 4b. Headless SDK (`navigator/headless-sdk/` — Python `vayu_headless`)

```sh
python3 -m venv .venv && ./.venv/bin/pip install -e "navigator/headless-sdk[test]"
./.venv/bin/python -m pytest navigator/headless-sdk/tests
```
Drives the SITL stack headlessly (the Pilot API). It locates the `vsim_d` /
`vayu_sitl` binaries via `paths.py` (env overrides: `VSIM_BIN_PATH`,
`VAYU_SITL_BIN`). The optional native worldmesh helper:
`cmake -B build -S navigator/headless-sdk/cpp/worldmesh` → `vsim_worldmesh`.

---

## 5. Build-directory conventions

All build trees are out-of-source and git-ignored. Conventional names:

| Dir | Component / mode |
| --- | --- |
| `build/` | firmware (ARM) |
| `build_sitl/` | sim/host + tests |
| `build_san/`, `build_cov/`, `build_tidy/` | sim/host sanitizer / coverage / tidy |
| `build_sitl_rtos/` | sim/host RTOS variant |
| `build_vsim/` | vsim_d |
| `navigator/build/`, `navigator/build-gcsonly/` | GCS |
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

`tools/scripts/` holds the convenience wrappers (they cd to the repo root
themselves, so run from anywhere):

| Script | Does |
| --- | --- |
| `build.sh` | firmware configure + build → `build/main` |
| `flash.sh` | firmware build + objcopy + `st-flash` |

*(More per-component build/test wrappers are planned — see the task that follows
this file.)*

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
