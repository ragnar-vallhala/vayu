#!/usr/bin/env bash
# Run clang-tidy over every source in the repo that some build actually
# compiles, and name the ones nothing does. Exits non-zero on any finding --
# .clang-tidy sets WarningsAsErrors, so this is a gate, not a report.
#
# Scope is taken from compile databases, not from a hand-maintained file list,
# so it grows with the builds. The last thing the script prints is the set of
# tracked sources no listed database covers -- that is the gap report, and it
# is the only reason this stays honest as the tree moves.
#
# A file in two databases is analysed under both, deliberately: on ARM
# `int`, `unsigned` and `size_t` are all 32-bit, so the widening checks that
# found most of the real defects here (bugprone-implicit-widening-of-
# multiplication-result) cannot fire. The host pass over the same firmware
# sources is what catches them.
#
# CROSS-COMPILATION. An ARM database needs three things or clang-tidy reports
# noise instead of defects. Each flag is here because it was measured to
# remove a whole class of false positives:
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
# Databases are ARM or host by inspection (does the compiler name contain
# arm-none-eabi), so adding a build dir needs no extra configuration here.
#
# Usage: run_clang_tidy.sh [build-dir ...]   (default: the list below)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

# .clang-tidy is a gate (WarningsAsErrors), so a version difference between a
# developer and CI turns a toolchain bump into a red build on someone else's
# PR. CI pins this; locally it is whatever is on PATH.
TIDY="${CLANG_TIDY:-clang-tidy}"

# Configure with cmake -S <src> -B <dir> -DCMAKE_EXPORT_COMPILE_COMMANDS=ON.
# A database only has to be CONFIGURED, not built -- clang-tidy reads the
# compile commands, not the objects.
#
#   cmake -S firmware          -B firmware/build
#   cmake -S firmware          -B firmware/build_hwtest -DVAYU_HW_TEST=ON
#   cmake -S firmware/tests/host -B firmware/tests/host/build
#         ...and BUILD this one: AUTOMOC generates the tst_*.moc that
#   cmake -S sim/host          -B sim/host/build -DVAYU_SITL_RTOS_BUILD=ON
#         The flag matters: without it the RTOS engine, the loadable module and
#         sim/vsim's physics are in no database at all. They used to ride in via
#
# an off-by-default option, and with it off GpuGrass.cpp is in no database.
DEFAULT_BUILDS=(
  firmware/build             # ARM   firmware/src
  firmware/build_hwtest      # ARM   firmware/tests/onboard
  firmware/tests/host/build  # host  firmware/tests/host
  sim/host/build             # host  sim/host + sim/vsim (configure with RTOS on)
)

# Standalone tests and tools built by a single hand-written compiler line
# recorded in their own header comment, so there is no database to read.
# Pathspec, then the flags that line uses.
NODB=(
  "vtest/vtest.c          -std=c11 -D_POSIX_C_SOURCE=200809L"
  "navlink/tests/test_c.c -std=c11 -Inavlink/generated/c"
)
# navlink's C codec is generated from the tracked dialect.json and is NOT in
# the index, so a fresh checkout has no navlink_msgs.h and test_c.c cannot be
# parsed. Generate it the same way navlink/tests/run_tests.py step 1 does --
# idempotent, and it is the only reason CI saw a finding a developer with a
# warm tree never would.
if [ ! -f navlink/generated/c/navlink_msgs.h ]; then
  echo "generating navlink C codec (absent, and not tracked)" >&2
  python3 navlink/generate.py --lang c >/dev/null
fi

BUILDS=("$@")
if [ ${#BUILDS[@]} -eq 0 ]; then BUILDS=("${DEFAULT_BUILDS[@]}"); fi

GCC_INC="$(arm-none-eabi-gcc -print-file-name=include 2>/dev/null || true)"
LIBC_INC="${GCC_INC:+$(dirname "$GCC_INC")/../../../arm-none-eabi/include}"
ARM_ARGS=(
  --extra-arg=--target=arm-none-eabi
  --extra-arg=-isystem"$GCC_INC"
  --extra-arg=-isystem"$LIBC_INC"
  --extra-arg=-Wno-error
  --extra-arg=-Wno-ignored-optimization-argument
  --extra-arg=-Wno-newline-eof
  --extra-arg=-Wno-macro-redefined
)

analysed="$(mktemp)"; errbuf="$(mktemp)"
trap 'rm -f "$analysed" "$errbuf"' EXIT
rc=0

# clang-tidy raises diagnostics inside system and Qt headers, then drops them
# because they are not in HeaderFilterRegex -- but it still prints the count it
# raised, "291 warnings generated.", once per file. --quiet hides the companion
# "Suppressed N warnings" note and not that line, so a clean run reads as
# hundreds of warnings. Nothing there is actionable: a real finding prints as
# "path:line:col: error:" on stdout. Drop the counts, keep everything else.
run_tidy() { # args...: passed straight to xargs
  xargs "$@" 2>"$errbuf" || return 1
} 
report_err() {
  grep -vE '^[0-9]+ (warning|error)s? generated\.$' "$errbuf" >&2 || true
}

for build in "${BUILDS[@]}"; do
  db="$build/compile_commands.json"
  if [ ! -f "$db" ]; then
    echo "skip $build -- no compile_commands.json" >&2
    continue
  fi
  # Tracked, non-vendored, not a generated file inside a build tree.
  files="$(jq -r '.[].file' "$db" | sed "s|^$ROOT/||" | sort -u \
           | grep -v '^extern/' | grep -vE '(^|/)build[^/]*/' \
           | git check-ignore -nv --stdin 2>/dev/null | sed -n 's/^::\t//p' || true)"
  if [ -z "$files" ]; then continue; fi

  args=()
  if grep -q 'arm-none-eabi' "$db"; then args=("${ARM_ARGS[@]}"); fi
  echo "== $build ($(wc -l <<<"$files") files, $([ ${#args[@]} -gt 0 ] && echo arm || echo host))" >&2
  run_tidy -P"$(nproc)" -I{} "$TIDY" -p "$build" --quiet "${args[@]}" {} <<<"$files" || rc=1
  report_err
  cat >>"$analysed" <<<"$files"
done

for entry in "${NODB[@]}"; do
  spec="${entry%% *}"; flags="${entry#* }"
  files="$(git ls-files "$spec")"
  if [ -z "$files" ]; then continue; fi
  echo "== $spec ($(wc -l <<<"$files") files, no database)" >&2
  # shellcheck disable=SC2086 -- flags are a deliberate word list
  run_tidy -P"$(nproc)" -I{} "$TIDY" --quiet {} -- $flags <<<"$files" || rc=1
  report_err
  cat >>"$analysed" <<<"$files"
done

# Anything left is unanalysed: wire up its build, add it to NODB, or delete it
# if nothing builds it any more. firmware/docs carries scratch sources attached
# to log-analysis journals; those are archive, not tree.
missed="$(git ls-files '*.c' '*.cpp' | grep -vE '^(extern|firmware/docs)/' | sort -u \
           | comm -13 <(sort -u "$analysed") -)"
if [ -z "$missed" ]; then
  echo "== not analysed by anything above: none" >&2
else
  echo "== not analysed by anything above ($(wc -l <<<"$missed") files)" >&2
  echo "$missed" >&2
fi

exit $rc
