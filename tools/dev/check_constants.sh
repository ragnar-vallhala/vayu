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
#
# One home per physical constant (fault line F13).
#
# F13 is the same formula or constant written twice across a boundary, free to
# diverge, agreeing only by coincidence. It is not hypothetical here: bme280.c
# carried its own ISA altitude curve and its own sea-level datum beside the
# hub's, and they agreed only because the driver's sea-level setter had no
# callers. Gravity was written two ways, 9.80665f where it defines a unit and
# 9.81f where it sets a gate.
#
# A duplicate of this kind cannot be caught by a compiler or a unit test: both
# copies are correct in isolation, and the bug is that a later edit will move
# one of them. So the check is textual -- the literal may appear in exactly one
# file, the one that names it.
#
# This does NOT gate tuning (control/tuning.h), board facts (vayu_board.h) or
# loop rates. Those are properties of this aircraft and this board, and having
# a local one is often right. It gates values that would be the same on any
# vehicle, where two of them disagreeing is never intended.
#
# Test code is excluded: a test that hard-codes 9.80665 to check a conversion
# is stating an expected value, which is its job.
#
# Usage: check_constants.sh   (exit 1 if a constant has more than one home)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

SEARCH="firmware/src firmware/include"

# name | home file | regex matching the bare literal
CONSTANTS=(
  "gravity|firmware/include/physics.h|9\.80665|9\.81[^0-9]"
  "sea-level pressure|firmware/include/hub/sample.h|101325"
  "ISA altitude coefficient|firmware/src/hub/hub.c|44330"
)

rc=0
for entry in "${CONSTANTS[@]}"; do
  IFS='|' read -r name home pat1 pat2 <<<"$entry"
  pat="$pat1"
  [ -n "${pat2:-}" ] && pat="$pat1|$pat2"

  hits="$(grep -rnE "$pat" $SEARCH 2>/dev/null | grep -v "^${home}:" || true)"
  n=0
  [ -n "$hits" ] && n="$(printf '%s\n' "$hits" | wc -l | tr -d ' ')"

  if [ "$n" -gt 0 ]; then
    printf '  %-26s %3d outside %s  SECOND HOME\n' "$name" "$n" "$home"
    printf '%s\n' "$hits" | sed 's/^/      /' >&2
    rc=1
  else
    printf '  %-26s     single home: %s\n' "$name" "$home"
  fi
done

if [ "$rc" != 0 ]; then
  cat >&2 <<'EOF'

A physical constant is spelled in more than one place.

Both copies are correct today; the fault is that nothing keeps them equal, and
the compiler cannot see the difference. Include the header that owns it:
  - gravity, and anything else true of any vehicle -> physics.h
  - something true of THIS airframe                -> control/tuning.h
  - something true of THIS board                   -> vayu_board.h
EOF
  exit 1
fi

echo "constants: one home each"
