#!/usr/bin/env bash
# Vayu monorepo build/test dispatcher. Run from anywhere — it locates the repo
# root itself. One entry point for every component (see build.md for the full map).
#
# Usage:
#   vayu.sh build <firmware|sitl|vsim|gcs|rtos|all>   [opts]
#   vayu.sh test  <sitl|gcs|headless|all>             [opts]
#   vayu.sh flash                                     [opts]   # firmware build + st-flash
#   vayu.sh clean                                              # remove all build trees
#
# Options:
#   -j N          parallel jobs (default: nproc)
#   --release     CMAKE_BUILD_TYPE=Release (gcs/sitl/vsim; firmware flags are fixed)
#   --debug       CMAKE_BUILD_TYPE=Debug
#   --no-sitl     gcs only: configure -DNAVIGATOR_SITL=OFF (GCS-only, no sim/autotune)
#   --sanitize    sitl only: -DVAYU_SANITIZE=ON
#   --coverage    sitl only: -DVAYU_COVERAGE=ON (then runs the coverage target)
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
build_vsim() {
  say "sim/vsim -> build_vsim/vsim_d"
  cmake -S sim/vsim -B build_vsim $(bt_flag)
  cmake --build build_vsim -j"$JOBS"
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

# ---- test steps: split into "ensure built" (aborts on failure) and "run"
#      (tolerant, returns status) so `test all` can build everything first and
#      then run every suite, keeping the results together at the end. -----------
HEADLESS_PY=python3   # resolved by ensure_headless_build, used by run_headless_tests

ensure_sitl_build() { [ -d build_sitl ] || build_sitl; }
ensure_gcs_build() {
  local type; type="$(bt_flag)"; [ -z "$type" ] && type="-DCMAKE_BUILD_TYPE=Release"
  say "navigator: configure + build with tests"
  cmake -S navigator -B navigator/build "$type" -DNAVIGATOR_BUILD_TESTS=ON
  cmake --build navigator/build -j"$JOBS"
}
ensure_headless_build() {
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

run_sitl_tests() {
  say "ctest: sim/host"
  ctest --test-dir build_sitl --output-on-failure || return 1
  if [ "$COVERAGE" = 1 ]; then say "coverage summary"; cmake --build build_sitl --target coverage; fi
}
run_gcs_tests() {
  # Only navigator's own QtTest binaries (tst_*); navigator add_subdirectory's
  # sim/host, which registers the host ctest suite too — run that via `test sitl`.
  say "ctest: navigator (offscreen, tst_*)"
  ctest --test-dir navigator/build -R '^tst_' --output-on-failure
}
run_headless_tests() {
  say "pytest: navigator/headless-sdk"
  "$HEADLESS_PY" -m pytest navigator/headless-sdk/tests
}

# single-component: build then run (set -e aborts on a build failure)
test_sitl()     { ensure_sitl_build;     run_sitl_tests; }
test_gcs()      { ensure_gcs_build;      run_gcs_tests; }
test_headless() { ensure_headless_build; run_headless_tests; }

# everything: build all targets first, THEN run all suites, THEN one summary.
test_all() {
  say "PHASE 1/2 — building all test targets"
  ensure_sitl_build
  ensure_gcs_build
  ensure_headless_build

  say "PHASE 2/2 — running all suites"
  local fail=0 r_sitl r_gcs r_hl
  run_sitl_tests     && r_sitl=PASS || { r_sitl=FAIL; fail=1; }
  run_gcs_tests      && r_gcs=PASS  || { r_gcs=FAIL;  fail=1; }
  run_headless_tests && r_hl=PASS   || { r_hl=FAIL;   fail=1; }

  printf '\n\033[1m==== test all — summary ====\033[0m\n'
  printf '  sim/host (ctest)       %s\n' "$r_sitl"
  printf '  navigator (tst_*)      %s\n' "$r_gcs"
  printf '  headless-sdk (pytest)  %s\n' "$r_hl"
  if [ "$fail" = 0 ]; then printf '  \033[1mALL GREEN\033[0m\n'; else printf '  \033[1mSOME FAILED\033[0m\n'; fi
  return "$fail"
}

# ---- dispatch --------------------------------------------------------------
case "$CMD" in
  build)
    case "$TARGET" in
      firmware) build_firmware;;
      sitl)     build_sitl;;
      vsim)     build_vsim;;
      rtos)     build_rtos;;
      gcs)      build_gcs;;
      all)      build_firmware; build_sitl; build_vsim; build_gcs;;
      ""|*)     die "build: target must be one of firmware|sitl|vsim|gcs|rtos|all";;
    esac;;
  test)
    case "$TARGET" in
      sitl)     test_sitl;;
      gcs)      test_gcs;;
      headless) test_headless;;
      all)      test_all;;
      ""|*)     die "test: target must be one of sitl|gcs|headless|all";;
    esac;;
  flash)
    build_firmware
    say "flash via st-flash"
    arm-none-eabi-objcopy -O binary build/main build/main.bin
    st-flash --connect-under-reset write build/main.bin 0x8000000;;
  clean)
    say "removing build trees"
    rm -rf build build_sitl build_san build_cov build_tidy build_sitl_rtos \
           build_vsim navigator/build navigator/build-gcsonly
    echo "clean.";;
  ""|-h|--help)
    sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//';;
  *) die "unknown command: $CMD (try: build | test | flash | clean)";;
esac
