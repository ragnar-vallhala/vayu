#!/usr/bin/env bash
# Run clang-tidy over the firmware against the ARM compile database.
#
# The firmware is cross-compiled, so three things have to be handled or the
# analysis reports noise instead of defects. Each flag below is here because it
# was measured to remove a whole class of false positives:
#
#   --target=arm-none-eabi      without it clang assumes the host ABI
#   -isystem <gcc>/include      compiler intrinsics (stdint, stdarg)
#   -isystem <arm-none-eabi>/include   newlib headers -- WITHOUT this you get
#                               "'string.h' file not found" on 5 files and the
#                               analysis of them is abandoned entirely
#   -Wno-error                  the build uses -Werror; clang has diagnostics
#                               gcc does not, so its extras would abort analysis
#   -Wno-ignored-optimization-argument   clang does not know
#                               -fsingle-precision-constant, which the build sets
#   -Wno-newline-eof            three vendored headers lack a trailing newline
#   -Wno-macro-redefined        NavHAL and vaios both define VERSION*; that is
#                               a known upstream collision in extern/, not ours
#
# Using the SITL database instead (as CI did until 2026-09-09) leaves 11 of the
# 67 firmware sources with no compile command at all, so clang-tidy analyses
# them with default flags and invents undeclared-function and parse errors.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD="${1:-$ROOT/firmware/build}"

if [ ! -f "$BUILD/compile_commands.json" ]; then
  echo "no compile_commands.json in $BUILD -- run: cmake -S firmware -B $BUILD" >&2
  exit 2
fi

GCC_INC="$(arm-none-eabi-gcc -print-file-name=include)"
LIBC_INC="$(dirname "$GCC_INC")/../../../arm-none-eabi/include"

cd "$ROOT"
git ls-files 'firmware/src/*.c' \
  | xargs -P"$(nproc)" -I{} clang-tidy -p "$BUILD" --quiet \
      --extra-arg=--target=arm-none-eabi \
      --extra-arg=-isystem"$GCC_INC" \
      --extra-arg=-isystem"$LIBC_INC" \
      --extra-arg=-Wno-error \
      --extra-arg=-Wno-ignored-optimization-argument \
      --extra-arg=-Wno-newline-eof \
      --extra-arg=-Wno-macro-redefined \
      {}
