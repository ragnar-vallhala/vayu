#!/usr/bin/env bash
# Cross-platform builds for the Vayu stack inside the Docker toolchain image.
# Run from anywhere; it locates the repo root itself.
#
#   tools/docker/build.sh image      # build (or rebuild) the toolchain image
#   tools/docker/build.sh firmware   # arm-none-eabi STM32 build -> build-docker/
#   tools/docker/build.sh sitl       # host SITL build + ctest    -> build_sitl-docker/
#   tools/docker/build.sh gcs        # Qt6 Navigator GCS          -> software/build-docker/
#   tools/docker/build.sh all        # firmware + sitl + gcs
#   tools/docker/build.sh shell      # interactive shell in the container
#   tools/docker/build.sh clean      # remove the *-docker build dirs
#
# Windows: run this under WSL2 or Git Bash, or use the `docker compose run`
# commands from docker-compose.yml directly in PowerShell.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

# Build image as the host user so bind-mounted outputs aren't root-owned.
export USER_UID="$(id -u)" USER_GID="$(id -g)"

DC="docker compose"
$DC version >/dev/null 2>&1 || DC="docker-compose"

run() { $DC run --rm vayu bash -lc "$1"; }
J='-j"$(nproc)"'

case "${1:-all}" in
  image)    $DC build ;;
  firmware) run "cmake -B build-docker && cmake --build build-docker $J" ;;
  sitl)     run "cmake -S sim/host -B build_sitl-docker && cmake --build build_sitl-docker $J && ctest --test-dir build_sitl-docker --output-on-failure" ;;
  gcs)      run "cmake -S software -B software/build-docker && cmake --build software/build-docker $J" ;;
  all)      "$0" firmware && "$0" sitl && "$0" gcs ;;
  shell)    $DC run --rm vayu bash ;;
  clean)    rm -rf build-docker build_sitl-docker software/build-docker ;;
  *)        echo "usage: $0 [image|firmware|sitl|gcs|all|shell|clean]"; exit 1 ;;
esac
