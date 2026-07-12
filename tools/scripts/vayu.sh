#!/usr/bin/env bash
# Vayu monorepo build/test dispatcher. Run from anywhere — it locates the repo
# root itself. One entry point for every component (see build.md for the full map).
#
# Usage:
#   vayu.sh build <firmware|sitl|vsim|gcs|rtos|vtest|all>  [opts]
#   vayu.sh test  [sitl|gcs|headless|all|vtest]       [opts]   # no target -> vtest TUI
#   vayu.sh flash                                     [opts]   # firmware build + st-flash
#   vayu.sh clean                                              # remove all build trees
#
# Options:
#   -j N          parallel jobs (default: nproc)
#   --release     CMAKE_BUILD_TYPE=Release (gcs/sitl/vsim; firmware flags are fixed)
#   --debug       CMAKE_BUILD_TYPE=Debug
#   --no-sitl     gcs only: configure -DNAVIGATOR_SITL=OFF (GCS-only, no sim/autotune)
#   --sanitize    sitl only: -DVAYU_SANITIZE=ON
#   --coverage    sitl only: -DVAYU_COVERAGE=ON, then runs the coverage-gate
#                 target (per-component floor ratchet; fails on a regression)
#
# Examples:
#   vayu.sh build all
#   vayu.sh build gcs --no-sitl
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
build_gcs() {
  local sitl=ON; [ "$NO_SITL" = 1 ] && sitl=OFF
  local type; type="$(bt_flag)"; [ -z "$type" ] && type="-DCMAKE_BUILD_TYPE=Release"
  say "navigator -> navigator/build (SITL $sitl)"
  cmake -S navigator -B navigator/build "$type" -DNAVIGATOR_SITL=$sitl
  cmake --build navigator/build -j"$JOBS"
}
build_vtest() {
  say "vtest -> build_vtest/vtest (host C orchestrator)"
  mkdir -p build_vtest
  cc -std=c11 -O2 -Wall -Wextra vtest/vtest.c -o build_vtest/vtest
}

# ---- test steps: split into "ensure built" (aborts on failure) and "run"
#      (tolerant, returns status) so `test all` can build everything first and
#      then run every suite, keeping the results together at the end. -----------
HEADLESS_PY=python3   # resolved by ensure_headless_build, used by run_headless_tests

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
ensure_gcs_build() {
  local type; type="$(bt_flag)"; [ -z "$type" ] && type="-DCMAKE_BUILD_TYPE=Release"
  say "navigator: configure + build with tests"
  cmake -S navigator -B navigator/build "$type" -DNAVIGATOR_BUILD_TESTS=ON
  cmake --build navigator/build -j"$JOBS"
}
ensure_headless_build() {
  # The integration tests drive the REAL SITL stack, so they need a fresh
  # vayu_sitl_rtos; without it conftest SILENTLY skips (false green). Build it
  # (run_headless_tests points the SDK at it via VAYU_SITL_RTOS_BIN).
  ensure_rtos_build
  # System Python is often externally managed (PEP 668), so install into a venv.
  # Use $REPO_ROOT/.venv if present, otherwise create it.
  if [ ! -x "$REPO_ROOT/.venv/bin/python" ]; then
    say "headless-sdk: creating venv at .venv"
    python3 -m venv "$REPO_ROOT/.venv"
  fi
  HEADLESS_PY="$REPO_ROOT/.venv/bin/python"
  say "headless-sdk: install (editable, into .venv)"
  "$HEADLESS_PY" -m pip install -q --upgrade pip
  "$HEADLESS_PY" -m pip install -q -e "navigator/headless-sdk[test]"
}

# Per-suite result tallies (test cases run), filled by the run_* steps and read
# by the summary table. SKIP is dynamic (pytest); ctest suites here all run.
SITL_PASS=0 SITL_FAIL=0 SITL_SKIP=0
GCS_PASS=0  GCS_FAIL=0  GCS_SKIP=0
HL_PASS=0   HL_FAIL=0   HL_SKIP=0
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
parse_pytest() {
  local line p f e s
  line=$(grep -E '^=+.*(passed|failed|error|skipped|no tests).*=+$' "$1" 2>/dev/null | tail -1 || true)
  p=$(printf '%s' "$line" | grep -oE '[0-9]+ passed'  | grep -oE '[0-9]+' || true); p=${p:-0}
  f=$(printf '%s' "$line" | grep -oE '[0-9]+ failed'  | grep -oE '[0-9]+' || true); f=${f:-0}
  e=$(printf '%s' "$line" | grep -oE '[0-9]+ error'   | grep -oE '[0-9]+' || true); e=${e:-0}
  s=$(printf '%s' "$line" | grep -oE '[0-9]+ skipped' | grep -oE '[0-9]+' || true); s=${s:-0}
  echo "$p $((f + e)) $s"
}

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
run_gcs_tests() {
  _ensure_logdir
  # Only navigator's own QtTest binaries (tst_*); navigator add_subdirectory's
  # sim/host, which registers the host ctest suite too — run that via `test sitl`.
  say "ctest: navigator (offscreen, tst_*)"
  local log="$LOGDIR/gcs.log" rc=0
  if ctest --test-dir navigator/build -R '^tst_' --output-on-failure 2>&1 | tee "$log"; then rc=0; else rc=1; fi
  read -r GCS_PASS GCS_FAIL GCS_SKIP < <(parse_ctest "$log")
  return "$rc"
}
run_headless_tests() {
  _ensure_logdir
  # Point the SDK at the binary vayu.sh just built (its root-convention build
  # dir), so the integration tests never resolve to a stale sim/*/build copy.
  say "pytest: navigator/headless-sdk (binary: build_sitl_rtos)"
  local log="$LOGDIR/headless.log" rc=0
  if VAYU_SITL_RTOS_BIN="$REPO_ROOT/build_sitl_rtos/vayu_sitl_rtos" \
     "$HEADLESS_PY" -m pytest navigator/headless-sdk/tests 2>&1 | tee "$log"; then rc=0; else rc=1; fi
  read -r HL_PASS HL_FAIL HL_SKIP < <(parse_pytest "$log")
  return "$rc"
}

# single-component: build then run (set -e aborts on a build failure)
test_sitl()     { ensure_sitl_build;     run_sitl_tests; }
test_gcs()      { ensure_gcs_build;      run_gcs_tests; }
test_headless() { ensure_headless_build; run_headless_tests; }

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
  [ -f navigator/build/CTestTestfile.cmake ] || cmake -S navigator \
    -B navigator/build -DNAVIGATOR_BUILD_TESTS=ON >/dev/null 2>&1 \
    || say "gcs configure failed — vtest will flag it"
  [ -f build_fwtest/CTestTestfile.cmake ] || cmake -S firmware/tests/host \
    -B build_fwtest >/dev/null 2>&1 || say "fw-host configure failed — vtest will flag it"
  # build_sitl_rtos: the headless integration tests drive the in-process
  # vayu_sitl_rtos; vtest builds it on demand, so just ensure the dir is configured.
  [ -f build_sitl_rtos/CMakeCache.txt ] || cmake -S sim/host -B build_sitl_rtos \
    -DVAYU_SITL_RTOS_BUILD=ON >/dev/null 2>&1 \
    || say "rtos configure failed — headless integration may skip"
  if [ -t 1 ]; then ./build_vtest/vtest; else ./build_vtest/vtest --run; fi
}

# Consolidated table: static source counts (Files/Asserts) + dynamic case
# tallies (Pass/Fail/Skip) collected by the run_* steps.
test_table() {
  local f_sitl a_sitl f_gcs a_gcs f_hl a_hl
  f_sitl=$(find sim/host/tests -maxdepth 1 -name 'test_*.c' 2>/dev/null | wc -l | tr -d ' ')
  a_sitl=$(grep -rhoE '\bCHECK\b' sim/host/tests/*.c 2>/dev/null | wc -l | tr -d ' ')
  f_gcs=$(find navigator/tests -maxdepth 1 -name 'tst_*.cpp' 2>/dev/null | wc -l | tr -d ' ')
  a_gcs=$(grep -rhoE '\bQVERIFY2?\b|\bQCOMPARE\b|\bQTRY_[A-Z]+\b|\bQFAIL\b|\bQVERIFY_EXCEPTION_THROWN\b' navigator/tests/tst_*.cpp 2>/dev/null | wc -l | tr -d ' ')
  f_hl=$(find navigator/headless-sdk/tests -name 'test_*.py' 2>/dev/null | wc -l | tr -d ' ')
  a_hl=$(grep -rhE '^[[:space:]]*assert\b' navigator/headless-sdk/tests 2>/dev/null | wc -l | tr -d ' ')

  local tp=$((SITL_PASS + GCS_PASS + HL_PASS))
  local tf=$((SITL_FAIL + GCS_FAIL + HL_FAIL))
  local ts=$((SITL_SKIP + GCS_SKIP + HL_SKIP))
  local tfiles=$((f_sitl + f_gcs + f_hl))
  local tas=$((a_sitl + a_gcs + a_hl))

  printf '\n\033[1m==== test all — summary ====\033[0m\n'
  printf '  \033[1m%-24s %6s %8s %6s %6s %6s\033[0m\n' "Category" "Files" "Asserts" "Pass" "Fail" "Skip"
  printf '  %-24s %6s %8s %6s %6s %6s\n' "sim/host (ctest)"      "$f_sitl" "$a_sitl" "$SITL_PASS" "$SITL_FAIL" "$SITL_SKIP"
  printf '  %-24s %6s %8s %6s %6s %6s\n' "navigator (tst_*)"     "$f_gcs"  "$a_gcs"  "$GCS_PASS"  "$GCS_FAIL"  "$GCS_SKIP"
  printf '  %-24s %6s %8s %6s %6s %6s\n' "headless-sdk (pytest)" "$f_hl"   "$a_hl"   "$HL_PASS"   "$HL_FAIL"   "$HL_SKIP"
  printf '  %s\n' "------------------------------------------------------------------"
  printf '  \033[1m%-24s %6s %8s %6s %6s %6s\033[0m\n' "TOTAL" "$tfiles" "$tas" "$tp" "$tf" "$ts"
  printf '  \033[2mFiles/Asserts = source counts; Pass/Fail/Skip = test cases run.\033[0m\n'
}

# everything: build all targets first, THEN run all suites, THEN one table.
test_all() {
  say "PHASE 1/2 — building all test targets"
  ensure_sitl_build
  ensure_gcs_build
  ensure_headless_build

  say "PHASE 2/2 — running all suites"
  local fail=0
  run_sitl_tests     || fail=1
  run_gcs_tests      || fail=1
  run_headless_tests || fail=1

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
      gcs)      build_gcs;;
      vtest)    build_vtest;;
      all)      build_firmware; build_sitl; build_rtos; build_gcs;;
      ""|*)     die "build: target must be one of firmware|sitl|gcs|rtos|vtest|all";;
    esac;;
  test)
    case "$TARGET" in
      sitl)        test_sitl;;
      gcs)         test_gcs;;
      headless)    test_headless;;
      all)         test_all;;
      vtest|ui|"") run_vtest;;
      *)           die "test: target must be one of sitl|gcs|headless|all|vtest (none = vtest TUI)";;
    esac;;
  flash)
    build_firmware
    say "flash via st-flash"
    arm-none-eabi-objcopy -O binary build/main build/main.bin
    st-flash --connect-under-reset write build/main.bin 0x8000000;;
  clean)
    say "removing build trees"
    rm -rf build build_sitl build_san build_cov build_tidy build_sitl_rtos \
           build_vsim build_vtest build_fwtest navigator/build navigator/build-gcsonly
    echo "clean.";;
  ""|-h|--help)
    sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//';;
  *) die "unknown command: $CMD (try: build | test | flash | clean)";;
esac
