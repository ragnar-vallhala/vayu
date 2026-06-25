#!/usr/bin/env bash
# Firmware build + flash. Run from anywhere; it locates the repo root itself.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

rm -rf build
mkdir build
cd build
cmake .. -DNAVHAL=ON -DEXTERNAL_LINKER=ON
cmake --build .
arm-none-eabi-objcopy -O binary main main.bin
st-flash --connect-under-reset write main.bin 0x8000000
