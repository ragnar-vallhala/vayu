#!/usr/bin/env bash
# Copyright (C) 2026 NAVRobotec Pvt Ltd
# Author: Ragnar Vallhala
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Cross-platform builds for the Vayu stack inside the Docker toolchain image.
# Run from anywhere; it locates the repo root itself.
#
#   tools/docker/build.sh image      # build (or rebuild) the toolchain image
#   tools/docker/build.sh firmware   # arm-none-eabi STM32 build -> build-docker/
#   tools/docker/build.sh sitl       # host SITL build + ctest    -> build_sitl-docker/
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
  firmware) run "cmake -S firmware -B build-docker && cmake --build build-docker $J" ;;
  sitl)     run "cmake -S sim/host -B build_sitl-docker && cmake --build build_sitl-docker $J && ctest --test-dir build_sitl-docker --output-on-failure" ;;
  all)      "$0" firmware && "$0" sitl && "$0" gcs ;;
  shell)    $DC run --rm vayu bash ;;
  *)        echo "usage: $0 [image|firmware|sitl|gcs|all|shell|clean]"; exit 1 ;;
esac
