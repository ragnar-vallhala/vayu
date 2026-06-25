# Monorepo restructure — promote firmware/, navigator/, sim/ to top-level

## Context

The repo is a monorepo of ~5 distinct components, but only some are promoted to
top-level. The rest are either *naked at root* or *buried under a generic bucket*:

| Component | Lives now | Problem |
| --- | --- | --- |
| Firmware (ARM `main` ELF) | **naked at root** — `CMakeLists.txt`, `linker.ld`, `navhal.config`, `src/`, `include/` | top-level by default, not by intent; collides visually with project-level files |
| Navigator GCS (Qt6) + headless-sdk | `software/` | buried under a generic name; "software" describes the whole repo |
| Simulator (`sim_host` + `vsim` + `sim_gazebo` + `sim_renode`) | scattered inside `tools/` | a whole subsystem mislabeled as "tools" |
| NavLink protocol | `navlink/` | already a proper top-level component ✅ |
| vaios kernel | `extern/vaios` | already a proper submodule ✅ |

The goal is one consistent rule — **promote real components out of generic
buckets** — applied to all three offenders, yielding four sibling products.

This is a **design/migration plan only**. No moves are performed here. File:line
references are pointers to the live tree at time of writing (2026-06-25), not
contracts; re-grep before executing because line numbers drift.

> **Why firmware moves too.** Renaming the GCS to `navigator/` while the firmware
> stays naked at root is asymmetric — the firmware is a component too. Promoting
> it to `firmware/` makes all four products siblings. This is the largest single
> piece of the migration (the root CMakeLists enumerates ~70 source files
> explicitly for the per-module warning rollout, and `sim_host` reaches back into
> `src/` for the same files), but it is purely mechanical path editing.

---

## Target layout

```
/
├── firmware/        ← was root: CMakeLists.txt, linker.ld, navhal.config, src/, include/
├── navigator/       ← was software/ (GCS + headless-sdk + tests)
├── sim/             ← pulled out of tools/
│   ├── host/        ← tools/sim_host   (SITL firmware host seam, libvayu_sitl_core)
│   ├── vsim/        ← tools/vsim       (standalone physics daemon vsim_d)
│   ├── gazebo/      ← tools/sim_gazebo (alternative physics backend)
│   └── renode/      ← tools/sim_renode (dormant/empty)
├── navlink/         ← unchanged ✅
├── extern/vaios     ← unchanged submodule ✅
├── tools/           ← trimmed to ACTUAL tools:
│                       build.sh, flash.sh, autotune/, arduino/, docker/, windows/,
│                       and the loose *.py analysis/calib/sysid scripts
├── docs/  assets/
```

`extern/vaios` and `navlink/` do not move. `tools/autotune/` stays — it is a dev
tool, not a shipped product. The root keeps `extern/`, `navlink/`, `docs/`,
`assets/`, `.github/`, `docker-compose.yml`, `.gitignore`, `.gitmodules`,
`README.md`, `ARCHITECTURE.md`, `.clang-tidy`.

### What builds at root after firmware moves: nothing (decided)

**No root CMakeLists.** The root becomes a pure monorepo container; every
component builds from its own dir. Firmware builds via `cmake -S firmware -B build`,
and `tools/build.sh` updates its `-S .` → `-S firmware`. The root `CMakeLists.txt`
moves *into* `firmware/` (it is the firmware build) — none is left behind.

---

## Blast radius (measured 2026-06-25)

Three reference-map sweeps produced the counts below. "Hard" = build/script/runtime
breaks if not updated. "Soft" = docs/comments — correctness-neutral but should be
swept for accuracy.

| Move | Hard refs | Soft refs | Risk |
| --- | --- | --- | --- |
| `software/` → `navigator/` | 6 files + 2 runtime | ~85 files / ~90 refs | Low–Med |
| `tools/{sim_host,vsim,…}` → `sim/` | ~8 CMake + 6 CI + 3 python | ~100 docs | Med |
| root → `firmware/` | root CMake (~70 `src/` paths + GLOB), sim_host CMake (~69 `${VAYU_ROOT}/src` refs), build.sh/flash.sh/stacktrace.py, CI firmware job, vaios include wiring | many | Med–High (mechanical) |

Generated trees (`build/`, `build_sitl/`, `*/build*/`, `.venv/`, `__pycache__/`,
moc/CMake artifacts) are **not** edited — they regenerate. Delete and reconfigure
rather than fix them.

---

## Phase 0 — Pre-flight

1. Land/stash all in-flight work. A tree-wide `git mv` is hostile to open diffs.
   (Note the uncommitted SDIO work on `feat/centralised-fs-owner` per memory —
   do this on a clean dedicated branch.)
2. Create branch `chore/monorepo-restructure`.
3. Confirm clean baseline builds, so post-move breakage is unambiguous:
   - Firmware: `cmake -S . -B build && cmake --build build` (current invocation)
   - SITL: `cmake -S tools/sim_host -B build_sitl && cmake --build build_sitl && ctest --test-dir build_sitl`
   - GCS: `cmake -S software -B software/build -DNAVIGATOR_SITL=ON && cmake --build software/build`
   - GCS-only: `-DNAVIGATOR_SITL=OFF` (the Windows path)
4. Use `git mv` for every move so history follows. Reconfigure (fresh build dir)
   after each phase — never reuse a CMake cache across a path move.
5. Add the build-output dirs created by new invocations to `.gitignore` as they
   appear (`/build`, `/build_sitl`, etc. are already covered; verify).

Execute in the order below: **sim/ first** (fewest cross-refs), then
**navigator/**, then **firmware/** (touches the most, including the now-moved sim
and navigator CMake). Each phase ends green before the next starts.

---

## Phase 1 — `tools/{sim_host,vsim,sim_gazebo,sim_renode}` → `sim/`

`sim_host` has a one-way hard dependency on `vsim` (shares `vsim_proto.h`; links
its physics in the Phase-4 RTOS build). `sim_gazebo` is an alternative physics
backend for the same `sim_host`. They are one subsystem — move all four.

### Moves
```
git mv tools/sim_host   sim/host
git mv tools/vsim       sim/vsim
git mv tools/sim_gazebo sim/gazebo
git mv tools/sim_renode sim/renode
```

### Hard edits (build/CI/runtime — build breaks if skipped)

**`software/CMakeLists.txt`** (will be `navigator/CMakeLists.txt` after Phase 2 —
do these edits now, they travel with the file):
- L287 `include_directories(.../../tools/vsim/include)` → `.../../sim/vsim/include`
- L324 `set(VAYU_SITL_DIR ".../../tools/sim_host")` → `.../../sim/host`
  (L325–334 reference the variable — no change once the var is fixed)

**`sim/host/CMakeLists.txt`** (was `tools/sim_host/CMakeLists.txt`):
- L51 `${VAYU_ROOT}/tools/vsim/include` → `${VAYU_ROOT}/sim/vsim/include`
- L200–209 Phase-4 in-process physics: `${VAYU_ROOT}/tools/vsim/src/*` →
  `${VAYU_ROOT}/sim/vsim/src/*` (sim_controller, physics_core, motor_model,
  sensor_models)
- Verify its `VAYU_ROOT` derivation (`../..`) still resolves — `sim/host` is the
  same depth as `tools/sim_host`, so `../..` is unchanged ✅

**`software/tests/CMakeLists.txt`**:
- L95 `${CMAKE_SOURCE_DIR}/../tools/vsim/include` → `.../../sim/vsim/include`

**`software/headless-sdk/cpp/worldmesh/CMakeLists.txt`**:
- L23 `${_root}/tools/vsim/include` → `${_root}/sim/vsim/include`

**`.github/workflows/ci.yml`** (4 SITL jobs):
- L74 `cmake -S tools/sim_host -B build_sitl` → `-S sim/host`
- L87 `cmake -S tools/sim_host -B build_san …` → `-S sim/host`
- L103 `cmake -S tools/sim_host -B build_cov …` → `-S sim/host`
- L119 `cmake -S tools/sim_host -B build_tidy …` → `-S sim/host`

**`tools/docker/build.sh`**:
- L32 `cmake -S tools/sim_host -B build_sitl-docker` → `-S sim/host`

**`software/headless-sdk/vayu_headless/paths.py`** (runtime binary lookups):
- L22 `("tools","vsim","build","vsim_d")` → `("sim","vsim","build","vsim_d")`
- L29 `("tools","sim_host","build_sitl","vayu_sitl")` → `("sim","host","build_sitl","vayu_sitl")`

**`software/headless-sdk/tests/unit/test_wire_drift.py`**:
- L16 `("tools","vsim","include","vsim_proto.h")` → `("sim","vsim","include","vsim_proto.h")`

### Verify
```
cmake -S sim/host -B build_sitl && cmake --build build_sitl && ctest --test-dir build_sitl
cmake -S sim/vsim -B build_vsim && cmake --build build_vsim
```
Then the GCS SITL build (Phase 2 will re-verify after the rename).

### Soft sweep (~100 refs, non-breaking)
`ARCHITECTURE.md`, `AUTOTUNE-ROLLPITCH-ANALYSIS.md` (gitignored — skip), the moved
`sim/vsim/docs/**` (sim-architecture.md, journal, sim-fidelity plans),
`docs/reference/{firmware-control,trace}.md`, `docs/plans/sitl-lockstep-sim.md`,
`docs/scratch/sitl-fc-stub-inventory.md`, `software/docs/reference/requirements.md`,
`software/headless-sdk/PLAN.md`, `sim/gazebo/README.md`, plus CMake comment blocks
in the moved files. Bulk `tools/sim_host→sim/host`, `tools/vsim→sim/vsim` then
manual review.

---

## Phase 2 — `software/` → `navigator/`

### Move
```
git mv software navigator
```

### Hard edits

**`.gitignore`**:
- L13 (and the LaTeX block L28–36) `software/build-docker/` → `navigator/build-docker/`;
  sweep every `software/` occurrence.

**`tools/docker/build.sh`**:
- L8, L33, L36 `cmake -S software -B software/build-docker` → `-S navigator -B navigator/build-docker`

**`tools/windows/deploy-navigator-msys.sh`** (just edited this session):
- L13, L20 default build dir `software/build` → `navigator/build`

**`tools/loc_counter.py`**:
- L9, L34 source dirs `["software/src","software/tests"]` → `["navigator/src","navigator/tests"]`

**`navigator/headless-sdk/cpp/worldmesh/CMakeLists.txt`**:
- L17–18, L22 `${_root}/software/src/vsim/` → `${_root}/navigator/src/vsim/`
  (note: after Phase 1 the `vsim/include` ref on L23 already points at `sim/vsim`;
  this is the *GCS-side* `src/vsim/` renderer source, a different path — keep it
  under navigator)

### Runtime edits (silent failure if skipped)

**`navigator/src/ui/widgets/AboutDialog.cpp`**:
- L54 `appDir + "/../software/docs"` → `"/../navigator/docs"` (verify this path is
  actually shipped/used; may be dead — confirm before trusting)

**`navigator/headless-sdk/vayu_headless/_repo.py`**:
- L1 docstring names `software/` as the repo-root marker. Check whether
  `repo_root()` *keys on* the string `software` to locate the root — if so this is
  **hard**, update the marker to `navigator`; if it's only prose, soft.

**`navigator/headless-sdk/vayu_headless/world.py`**:
- L24 build-instruction string in an error message mentions `software/` → `navigator/`

### Verify
```
cmake -S navigator -B navigator/build -DNAVIGATOR_SITL=ON  && cmake --build navigator/build
cmake -S navigator -B navigator/build-gcsonly -DNAVIGATOR_SITL=OFF && cmake --build navigator/build-gcsonly
(cd navigator/headless-sdk && <its test runner>)
```

### Soft sweep (~85 files)
`ARCHITECTURE.md` (×9), navlink/docs, headless-sdk `*.md`, ~15 source-comment
files, python docstrings. Bulk `software/→navigator/` then review (watch for false
positives where "software" is prose, not a path).

---

## Phase 3 — root firmware → `firmware/`

The largest phase. Do it last so the sim/ and navigator/ paths it references are
already settled.

### Moves
```
git mv src include linker.ld navhal.config CMakeLists.txt firmware/
# Also any firmware-only root files: vaios_app_config.h lives in include/ (moves with it).
# extern/ stays at root (shared submodule). navlink/ stays at root (shared).
```
No root CMakeLists remains — the root `CMakeLists.txt` moves into `firmware/`.

> **`extern/vaios` stays at root**, but `firmware/CMakeLists.txt` does
> `add_subdirectory(extern/vaios)` and references `extern/vaios/include`,
> `.../NavHAL/...`. After the move these become `../extern/vaios/...` **or** keep
> them absolute via a computed repo-root var. Simplest: define
> `set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)` at the top of
> `firmware/CMakeLists.txt` and prefix the `extern/` and `navlink/` references
> with it. This also future-proofs the navlink codegen path.

### Hard edits

**`firmware/CMakeLists.txt`** (was root):
- The ~70 explicit `${CMAKE_CURRENT_SOURCE_DIR}/src/...` per-file warning-rollout
  paths: `CMAKE_CURRENT_SOURCE_DIR` now *is* `firmware/`, so `src/...` resolves
  correctly **without change** ✅ — same for the `FILE(GLOB_RECURSE … src/*.c)` on
  L100 and `include/vaios_app_config.h` (L41), `navhal.config` (L61), `linker.ld`.
  **This is the key insight: most of the root CMakeLists needs no edits because it
  uses `CMAKE_CURRENT_SOURCE_DIR`-relative paths, which move with the file.**
- The references that DO break are the ones reaching *outside* the firmware dir:
  - `extern/vaios/include`, `extern/vaios/extern/NavHAL/...` (L50, L119–122) →
    prefix with `${REPO_ROOT}/` (i.e. `../extern/vaios/...`)
  - `add_subdirectory(extern/vaios)` → `add_subdirectory(${REPO_ROOT}/extern/vaios …)`
    (out-of-tree add_subdirectory needs an explicit binary dir — supply one)
  - NavLink codegen invoking `navlink/generate.py` (the `execute_process`/`NAVLINK_GEN`
    block around L97) → `${REPO_ROOT}/navlink/...`

**`sim/host/CMakeLists.txt`**: it pulls ~69 firmware sources via
`${VAYU_ROOT}/src/...`. Now firmware lives in `firmware/src/...`:
- Replace `${VAYU_ROOT}/src/` → `${VAYU_ROOT}/firmware/src/` (and any
  `${VAYU_ROOT}/include` → `${VAYU_ROOT}/firmware/include`). This is the bulk of
  the edit — ~69 source paths + include dirs. Verify `VAYU_ROOT` itself
  (`../..` from `sim/host`) still lands on repo root ✅ (it does — `sim/host` is
  two levels deep).

**`navigator/CMakeLists.txt`**: check for any `../src` or `../include` firmware
references (the GCS may compile a few shared firmware files for SITL — grep
`\.\./src`, `\.\./include`). Re-point to `../firmware/src`, `../firmware/include`.

**`tools/build.sh`**:
- `cmake -S . -B build` → `cmake -S firmware -B build`

**`tools/flash.sh`**:
- L12–13 `arm-none-eabi-objcopy -O binary main main.bin` / `st-flash … main.bin`
  run in the build dir against the `main` ELF — confirm the build dir is still
  `build/` (unchanged) so these need no edit; if build.sh changes the output dir,
  update accordingly. (Per memory `flash-bin-staleness`: keep using objcopy/`make
  flash`, do not flash a stale `main.bin`.)

**`tools/stacktrace.py`**: derives the ELF path from repo root (per recent commit
`183e9dc`). Update its root→ELF path: `build/main` is unchanged if build dir
stays `build/`, but the *source* root for addr2line resolution may key on `src/` →
`firmware/src/`. Verify.

**`.github/workflows/ci.yml`** firmware job:
- The firmware build step `cmake -S . -B build` (or equivalent) → `-S firmware`.
  Re-grep ci.yml for `-S .` and any `src/`/`include/` path assumptions.

**`docker-compose.yml`** / `tools/docker/`: check for firmware build invocations
mounting/`-S .`.

### Verify
```
cmake -S firmware -B build && cmake --build build      # ELF builds
arm-none-eabi-size build/main                           # sanity
# full SITL + GCS re-verify (they reference firmware/src now):
cmake -S sim/host -B build_sitl && cmake --build build_sitl && ctest --test-dir build_sitl
cmake -S navigator -B navigator/build -DNAVIGATOR_SITL=ON && cmake --build navigator/build
```
Flash dry-run to a board if available (per memory: regenerate `main.bin`, don't
flash stale).

### Soft sweep
`ARCHITECTURE.md`, `README.md`, `docs/**`, root CMake comments, navhal.config
header comments, any `src/`-rooted path in docs/reference.

---

## Phase 4 — tidy `tools/` (optional, low cost)

With the subsystems gone, `tools/` is scripts + infra. Optional grouping to cut
root clutter (pure `git mv`, update only the soft docs + any cross-script refs):
- `tools/analysis/` ← sim_log_plot.py, sim_log_to_csv.py, trace.py, udp_telem_sniff.py
- `tools/calib/` ← gyro_calib_harness.py, parse_calib_hex.py
- `tools/sysid/` ← sysid_excite.py, sysid_fit.py (+ sysid_tune.json)
- leave `build.sh`, `flash.sh`, `apply_tune.py`, `stacktrace.py`, `loc_counter.py`,
  `autotune/`, `arduino/`, `docker/`, `windows/` at `tools/` top.

Skip if it adds churn without clear benefit — the component promotions are the
substance; this is cosmetics.

---

## Post-migration checklist

- [ ] All four builds green (firmware ELF, SITL + ctest, GCS SITL-on, GCS SITL-off)
- [ ] `git grep -n 'tools/sim_host\|tools/vsim\|tools/sim_gazebo\|tools/sim_renode'`
      returns only intended/historical doc hits
- [ ] `git grep -nw software` reviewed (prose "software" OK; path `software/` not)
- [ ] `git grep -n 'cmake -S \.'` — no stale root-firmware invocations
- [ ] CI passes on the branch (the SITL + firmware + tidy jobs all re-point)
- [ ] `compile_commands.json` regenerated; clangd resolves
- [ ] Update memory notes that name old paths: `gcs-windows-build`,
      `sitl-architecture`, `sitl-test-harness`, `mixer-frame-vs-physics-frame`,
      `headless-sdk`, `i2c-bus-sharing`/`uart-port-roles` (if they cite `src/...`).
- [ ] One commit per phase (sim/, navigator/, firmware/, tidy) so a regression
      bisects to a single move. No Claude trailer (repo convention).

## Risk notes

- **Biggest trap:** `git mv` then editing CMake caches in place. Always delete the
  build dir and reconfigure after a move.
- **`extern/vaios` out-of-tree add_subdirectory** from `firmware/` needs an
  explicit binary dir argument — the most likely thing to get wrong in Phase 3.
- **Submodule unaffected:** `.gitmodules` points at `extern/vaios`, which does not
  move — no submodule surgery.
- **Mechanical, not architectural:** every edit is a path string. No code logic,
  no API, no link-time symbol changes. The `CMAKE_CURRENT_SOURCE_DIR`-relative
  style of the root CMakeLists means the firmware move is far cheaper than the raw
  reference count suggests — only the *outward* (extern/, navlink/) refs break.
```

