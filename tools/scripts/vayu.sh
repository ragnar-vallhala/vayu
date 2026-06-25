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

# ---- per-component test steps ----------------------------------------------
test_sitl() {
  [ -d build_sitl ] || build_sitl
  say "ctest: sim/host"
  ctest --test-dir build_sitl --output-on-failure
  if [ "$COVERAGE" = 1 ]; then
    say "coverage summary"
    cmake --build build_sitl --target coverage
  fi
}
test_gcs() {
  # tests are ON by default; ensure a tests-enabled build exists
  local type; type="$(bt_flag)"; [ -z "$type" ] && type="-DCMAKE_BUILD_TYPE=Release"
  say "navigator: configure with tests"
  cmake -S navigator -B navigator/build "$type" -DNAVIGATOR_BUILD_TESTS=ON
  cmake --build navigator/build -j"$JOBS"
  say "ctest: navigator (offscreen)"
  # Select only navigator's own QtTest binaries (tst_*); navigator add_subdirectory's
  # sim/host, which registers the host ctest suite too — run that via `test sitl`.
  ctest --test-dir navigator/build -R '^tst_' --output-on-failure
}
test_headless() {
  say "pytest: navigator/headless-sdk"
  local py=python3
  if [ -x "$REPO_ROOT/.venv/bin/python" ]; then py="$REPO_ROOT/.venv/bin/python"; fi
  "$py" -m pip install -q -e "navigator/headless-sdk[test]"
  "$py" -m pytest navigator/headless-sdk/tests
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
      all)      test_sitl; test_gcs; test_headless;;
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
