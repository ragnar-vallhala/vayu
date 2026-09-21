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
# Vayu monorepo build/test dispatcher. Run from anywhere — it locates the repo
# root itself. One entry point for every component (see build.md for the full map).
#
# Usage:
#   vayu.sh build <firmware|sitl|vsim|rtos|vtest|all>  [opts]
#   vayu.sh test  [sitl|all|vtest]                   [opts]   # no target -> vtest TUI
#   vayu.sh flash                                     [opts]   # firmware build + st-flash
#   vayu.sh clean                                              # remove all build trees
#
# Options:
#   -j N          parallel jobs (default: nproc)
#   --release     CMAKE_BUILD_TYPE=Release (sitl/vsim; firmware flags are fixed)
#   --debug       CMAKE_BUILD_TYPE=Debug
#   --sanitize    sitl only: -DVAYU_SANITIZE=ON
#   --coverage    sitl only: -DVAYU_COVERAGE=ON, then runs the coverage-gate
#                 target (per-component floor ratchet; fails on a regression)
#
# Examples:
#   vayu.sh build all
#   vayu.sh test sitl --sanitize
#   vayu.sh test all
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

JOBS="$(nproc 2>/dev/null || echo 4)"
BUILD_TYPE=""
NO_SITL=0
SANITIZE=0
COVERAGE=0

die() { echo "vayu.sh: $*" >&2; exit 2; }
say() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }   # bold step banner

# ---- parse global options (anywhere after the subcommand) ------------------
CMD="${1:-}"; shift || true
TARGET="${1:-}"; [ $# -gt 0 ] && shift || true
case "$TARGET" in -*) TARGET="";; esac   # no target given (it was an option)

while [ $# -gt 0 ]; do
  case "$1" in
    -j) JOBS="${2:?-j needs a number}"; shift 2;;
    -j*) JOBS="${1#-j}"; shift;;
    --release) BUILD_TYPE=Release; shift;;
    --debug)   BUILD_TYPE=Debug; shift;;
    --no-sitl) NO_SITL=1; shift;;
    --sanitize) SANITIZE=1; shift;;
    --coverage) COVERAGE=1; shift;;
    -h|--help) sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0;;
    *) die "unknown option: $1";;
  esac
done

# Echo the build-type flag (or nothing). ALWAYS returns 0 — a non-zero return
# here would trip `set -e` when captured in a bare `x="$(bt_flag)"` assignment.
bt_flag() { if [ -n "$BUILD_TYPE" ]; then echo "-DCMAKE_BUILD_TYPE=$BUILD_TYPE"; fi; }

# ---- per-component build steps ---------------------------------------------
build_firmware() {
  say "firmware -> build/main (ARM)"
  cmake -S firmware -B build -DNAVHAL=ON -DEXTERNAL_LINKER=ON
  cmake --build build -j"$JOBS"
}
build_sitl() {
  local extra=""
  [ "$SANITIZE" = 1 ] && extra="$extra -DVAYU_SANITIZE=ON"
  [ "$COVERAGE" = 1 ] && extra="$extra -DVAYU_COVERAGE=ON"
  say "sim/host -> build_sitl${extra:+ ($extra )}"
  # shellcheck disable=SC2086
  cmake -S sim/host -B build_sitl $(bt_flag) $extra
  cmake --build build_sitl -j"$JOBS"
}
build_rtos() {
  say "sim/host RTOS -> build_sitl_rtos/vayu_sitl_rtos"
  cmake -S sim/host -B build_sitl_rtos -DVAYU_SITL_RTOS_BUILD=ON
  cmake --build build_sitl_rtos -j"$JOBS" --target vayu_sitl_rtos
}
build_vtest() {
  say "vtest -> build_vtest/vtest (host C orchestrator)"
  if [ ! -f vtest/vtest.c ]; then
    die "vtest/ is empty -- run: git submodule update --init vtest"
  fi
  mkdir -p build_vtest
  cc -std=c11 -O2 -Wall -Wextra vtest/vtest.c -o build_vtest/vtest
}

# ---- test steps: split into "ensure built" (aborts on failure) and "run"
#      (tolerant, returns status) so `test all` can build everything first and
#      then run every suite, keeping the results together at the end. -----------

# configure-if-needed, then ALWAYS incrementally build, so a test run can never
# pick up a stale binary (cheap when nothing changed).
ensure_sitl_build() {
  if [ -d build_sitl ]; then cmake --build build_sitl -j"$JOBS"; else build_sitl; fi
}
ensure_rtos_build() {
  if [ -d build_sitl_rtos ]; then
    cmake --build build_sitl_rtos -j"$JOBS" --target vayu_sitl_rtos
  else build_rtos; fi
}

# Per-suite result tallies (test cases run), filled by the run_* steps and read
# by the summary table. SKIP is dynamic (pytest); ctest suites here all run.
SITL_PASS=0 SITL_FAIL=0 SITL_SKIP=0
LOGDIR=""
_ensure_logdir() { [ -n "$LOGDIR" ] || LOGDIR="$(mktemp -d "${TMPDIR:-/tmp}/vayu-test.XXXXXX")"; }

# Parse a captured ctest log -> "pass fail skip" (each ctest test == one case).
# `|| true` on every grep: a no-match exits non-zero, which under `set -e` would
# otherwise abort the parser mid-way and yield empty counts.
parse_ctest() {
  local total failed
  total=$(grep -oE 'out of [0-9]+' "$1" 2>/dev/null | tail -1 | grep -oE '[0-9]+' || true)
  failed=$(grep -oE '[0-9]+ tests failed' "$1" 2>/dev/null | tail -1 | grep -oE '[0-9]+' || true)
  total=${total:-0}; failed=${failed:-0}
  echo "$((total - failed)) $failed 0"
}
# Parse a captured pytest log -> "pass fail skip" (errors fold into fail). pytest
# omits a category entirely when its count is 0 (e.g. no "failed" on a clean run),
# so each grep must tolerate no-match — hence `|| true`.

run_sitl_tests() {
  _ensure_logdir
  say "ctest: sim/host"
  local log="$LOGDIR/sitl.log" rc=0
  if ctest --test-dir build_sitl --output-on-failure 2>&1 | tee "$log"; then rc=0; else rc=1; fi
  read -r SITL_PASS SITL_FAIL SITL_SKIP < <(parse_ctest "$log")
  if [ "$COVERAGE" = 1 ] && [ "$rc" = 0 ]; then
    say "coverage ratchet gate"
    cmake --build build_sitl --target coverage-gate || rc=1
  fi
  return "$rc"
}

# single-component: build then run (set -e aborts on a build failure)
test_sitl()     { ensure_sitl_build;     run_sitl_tests; }

# The vtest orchestrator (vtest/) is a host C binary that discovers + drives the
# ctest suites itself. `vayu.sh test` with no target builds it and launches it:
# the interactive TUI on a terminal, or `--run` (batch + exit code) under CI.
#
# We only CONFIGURE the ctest suites here (cheap) so discovery (`ctest -N`)
# works — vtest builds each test binary on demand when it is selected for run,
# streaming the build log into its log pane. A suite that can't be configured is
# flagged in-TUI rather than aborting the launch.
run_vtest() {
  say "vtest orchestrator: configuring suites (lazy build on run), launching"
  build_vtest
  [ -f build_sitl/CTestTestfile.cmake ] || cmake -S sim/host -B build_sitl \
    $(bt_flag) >/dev/null 2>&1 || say "sitl configure failed — vtest will flag it"
  [ -f build_fwtest/CTestTestfile.cmake ] || cmake -S firmware/tests/host \
    -B build_fwtest >/dev/null 2>&1 || say "fw-host configure failed — vtest will flag it"
  # build_sitl_rtos: the SITL integration cases drive the in-process
  # vayu_sitl_rtos; vtest builds it on demand, so just ensure the dir is configured.
  [ -f build_sitl_rtos/CMakeCache.txt ] || cmake -S sim/host -B build_sitl_rtos \
    -DVAYU_SITL_RTOS_BUILD=ON >/dev/null 2>&1 \
    || say "rtos configure failed — SITL integration may skip"
  if [ -t 1 ]; then ./build_vtest/vtest; else ./build_vtest/vtest --run; fi
}

# Consolidated table: static source counts (Files/Asserts) + dynamic case
# tallies (Pass/Fail/Skip) collected by the run_* steps.
test_table() {
  local f_sitl a_sitl
  f_sitl=$(find sim/host/tests -maxdepth 1 -name 'test_*.c' 2>/dev/null | wc -l | tr -d ' ')
  a_sitl=$(grep -rhoE '\bCHECK\b' sim/host/tests/*.c 2>/dev/null | wc -l | tr -d ' ')

  local tp=$SITL_PASS
  local tf=$SITL_FAIL
  local ts=$SITL_SKIP
  local tfiles=$f_sitl
  local tas=$a_sitl

  printf '\n\033[1m==== test all — summary ====\033[0m\n'
  printf '  \033[1m%-24s %6s %8s %6s %6s %6s\033[0m\n' "Category" "Files" "Asserts" "Pass" "Fail" "Skip"
  printf '  %-24s %6s %8s %6s %6s %6s\n' "sim/host (ctest)"      "$f_sitl" "$a_sitl" "$SITL_PASS" "$SITL_FAIL" "$SITL_SKIP"
  printf '  %s\n' "------------------------------------------------------------------"
  printf '  \033[1m%-24s %6s %8s %6s %6s %6s\033[0m\n' "TOTAL" "$tfiles" "$tas" "$tp" "$tf" "$ts"
  printf '  \033[2mFiles/Asserts = source counts; Pass/Fail/Skip = test cases run.\033[0m\n'
}

# everything: build all targets first, THEN run all suites, THEN one table.
test_all() {
  say "PHASE 1/2 — building all test targets"
  ensure_sitl_build

  say "PHASE 2/2 — running all suites"
  local fail=0
  run_sitl_tests     || fail=1

  test_table
  if [ "$fail" = 0 ]; then printf '  \033[1;32mALL GREEN\033[0m\n'; else printf '  \033[1;31mSOME FAILED\033[0m\n'; fi
  return "$fail"
}

# ---- dispatch --------------------------------------------------------------
case "$CMD" in
  build)
    case "$TARGET" in
      firmware) build_firmware;;
      sitl)     build_sitl;;
      rtos)     build_rtos;;
      vtest)    build_vtest;;
      all)      build_firmware; build_sitl; build_rtos;;
      ""|*)     die "build: target must be one of firmware|sitl|rtos|vtest|all";;
    esac;;
  test)
    case "$TARGET" in
      sitl)        test_sitl;;
      all)         test_all;;
      vtest|ui|"") run_vtest;;
      *)           die "test: target must be one of sitl|all|vtest (none = vtest TUI)";;
    esac;;
  flash)
    build_firmware
    say "flash via st-flash"
    arm-none-eabi-objcopy -O binary build/main build/main.bin
    st-flash --connect-under-reset write build/main.bin 0x8000000;;
  clean)
    say "removing build trees"
    rm -rf build build_sitl build_san build_cov build_tidy build_sitl_rtos \
           build_vsim build_vtest build_fwtest
    echo "clean.";;
  ""|-h|--help)
    sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//';;
  *) die "unknown command: $CMD (try: build | test | flash | clean)";;
esac
