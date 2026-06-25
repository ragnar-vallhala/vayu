# Dockerized build environment

Reproducible builds of the whole Vayu stack — firmware, host SITL, and the Qt6
Navigator GCS — on **Linux, macOS, or Windows**, without installing the ARM
cross-toolchain, Qt6, or assimp natively. The image carries only the toolchains;
your checkout is bind-mounted, so edits on the host build immediately and the
artifacts land back in your tree.

## Prerequisites

- **Docker** — [Docker Desktop](https://www.docker.com/products/docker-desktop/)
  on Windows/macOS (enable the WSL2 backend on Windows), or Docker Engine +
  the compose plugin on Linux.
- The **submodules** must be checked out (the firmware needs `extern/vaios` +
  NavHAL):
  ```
  git submodule update --init --recursive
  ```

## Quick start

```bash
tools/docker/build.sh image      # build the toolchain image once (~1–2 GB)
tools/docker/build.sh all        # firmware + SITL (with tests) + Navigator GCS
```

Or a single target:

```bash
tools/docker/build.sh firmware   # STM32 build      -> build-docker/main(.bin via objcopy)
tools/docker/build.sh sitl       # host SITL + ctest -> build_sitl-docker/
tools/docker/build.sh gcs        # Qt6 Navigator     -> navigator/build-docker/Navigator
tools/docker/build.sh shell      # interactive shell in the container
tools/docker/build.sh clean      # remove the *-docker build dirs
```

Outputs use `*-docker` build directories so they never clash with your
host-native CMake caches (different compilers and absolute paths).

## Windows

`build.sh` is a bash script — run it from **WSL2** or **Git Bash**. From native
PowerShell, drive compose directly instead:

```powershell
docker compose build
docker compose run --rm vayu cmake -B build-docker
docker compose run --rm vayu cmake --build build-docker -j
```

(The same `cmake -S … -B …` invocations from `build.sh` work for SITL and the
GCS.)

## Flashing is done on the host, not in the container

USB/SWD passthrough is not portable across Docker Desktop backends, so the image
deliberately omits flashing. Build in the container, then flash the host-visible
artifact:

- **Linux:** `st-flash --reset write build-docker/main.bin 0x08000000`
  (or `tools/scripts/flash.sh`, which builds + flashes natively).
- **Windows/macOS:** flash `build-docker/main` / `main.bin` with
  **STM32CubeProgrammer** (ST-LINK over USB).

## Running the GCS

The image builds `Navigator` but doesn't run a display. To *run* the GUI you
need an X server (Linux: bind `/tmp/.X11-unix` + `DISPLAY`; Windows/macOS: an
X/Wayland bridge). For day-to-day GCS use, run the natively-built `Navigator` on
the host — Docker here is for **reproducible building** across machines.

## What's inside

Ubuntu 24.04 + `gcc-arm-none-eabi` (+ newlib), CMake/Ninja, Python3 +
`kconfiglib`, Qt6 (base/serialport/opengl) + `libassimp-dev`, and the CI
analysis tools (`cppcheck`, `clang-tidy`, `gcovr`). The package set mirrors
`.github/workflows/ci.yml` plus the GCS dependencies.
