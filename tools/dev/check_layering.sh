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
# Keep silicon out of the logic layers.
#
# The rule (docs/analysis/navhal-hardware-logic-separation.md §1): no
# translation unit under control/, est/ or maths/ may name a pin, a peripheral
# instance, a timer, or a NavHAL function. Those layers are pure maths over SI
# quantities; a hardware change must not recompile them, and they must stay
# host-testable without a HAL.
#
# This is TEXTUAL and per-file, not an include-graph check. variables.h still
# pulls navhal.h transitively (fault line F1), so an include-graph rule would
# fail everything and gate nothing. Textual is what is enforceable today, and it
# is what actually stopped the growth: between the June 2026 audit and September
# the raw-DWT call sites went 4 -> 7 and SYS_CLOCK_FREQ gained consumers,
# because nothing was watching.
#
# Time is an OS concept and may spill; pins, buses, chips, timers and registers
# are silicon and may not. Logic gets cycle stamps from sys/clock.h.
#
# SECTIONS AND THE RATCHET
#
# The rule above is the same everywhere; what differs is how far each part of
# the tree has got. Each section carries the count it is ALLOWED, and the gate
# fails when a section goes ABOVE it. The number may only come down.
#
# That is what makes a staged sweep survivable: a section cleaned this week
# cannot quietly regress while the next one is being worked on.
#
# FLOORS -- a nonzero allowance is not a backlog
#
# The sweep is finished: every section below is at its FLOOR, and the reason
# is written next to it. A floor is what the section legitimately names
# because that is the section's job -- sys/clock.c is the DWT seam the logic
# layers exist to avoid naming, and heartbeat.c drives an annunciator the same
# way actuator/motor.c drives an ESC. Driving a floor to zero does not remove
# the hardware dependency, it hides it behind another indirection and makes
# the gate report clean while the coupling is still there.
#
# So: a number going UP is a regression and fails. A number going DOWN is only
# progress if a dependency actually went away -- check the reason first.
#
# driver/ is deliberately absent. Silicon is what a driver is FOR -- gating it
# would be gating the thing that exists to hold the hardware.
#
# sensor/ is the adapter that brings drivers up without naming them, so its
# allowance is 0 and starts there: it holds a descriptor and walks a linker
# section, and the day it needs to know which chip it is talking to, the
# descriptor is missing a field. That is the whole design, and this is the
# check that keeps it true.
#
# To tighten: run the gate, it prints the new number when a section comes in
# under. Lower it here in the same commit that did the work.
#
# Usage:
#   check_layering.sh            every section
#   check_layering.sh comm       just that one
# Exit 1 if any checked section exceeds its allowance.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

# name | allowed (== its floor) | directories
SECTIONS=(
  "core|0|firmware/src/control firmware/src/est firmware/src/maths firmware/include/control firmware/include/est firmware/include/maths"
  "hub|0|firmware/src/hub firmware/include/hub"
  "sensor|0|firmware/src/sensor firmware/include/sensor"
  "dsp|0|firmware/src/dsp firmware/src/calib firmware/include/dsp firmware/include/calib"
  "storage|0|firmware/src/storage firmware/include/storage"
  # actuator: motor.c includes driver/esc.h. Driving an ESC is what it is for.
  "actuator|1|firmware/src/actuator firmware/include/actuator"
  # internal: clock.c 3 + boot.c 2 are the DWT/clock seam -- the one place the
  # CPU rate and the cycle counter are named, so control/ est/ maths/ never
  # have to. heartbeat.c names driver/indicator.h and sys_utils.c names
  # driver/crc.h; each drives exactly the device it names. logger/ is at 0.
  "internal|7|firmware/src/sys firmware/src/logger firmware/include/sys"
  # comm: one driver/uart.h per transport -- a transport owns a port.
  "comm|4|firmware/src/comm firmware/include/comm"
)

# Each pattern is a class of silicon fact, not a blocklist of names -- a new
# NavHAL call or a new pin macro is caught without touching this file.
#
# driver/ is in here because a device header reaches silicon on the caller's
# behalf: driver/bmx160.h includes navhal.h, so a TU that includes it gets the
# whole register map without ever typing "hal_". An umbrella header made that
# invisible -- this check reported clean while control/ transitively included
# navhal.h through driver/driver.h. Naming a driver is itself the violation;
# logic takes SI samples from hub/, not devices.
PATTERN='#include[[:space:]]*"(navhal|hal_|driver/)[^"]*"|\bhal_[a-z0-9_]+[[:space:]]*\(|\bGPIO_P[A-Z][0-9]|\bHAL_(I2C|UART|SPI|GPIO|PWM|TIM|DMA)|\bTIM[0-9]+\b'

WANT="${1:-}"
rc=0
checked=0

for entry in "${SECTIONS[@]}"; do
  name="${entry%%|*}"
  rest="${entry#*|}"
  allowed="${rest%%|*}"
  dirs="${rest#*|}"

  [ -n "$WANT" ] && [ "$WANT" != "$name" ] && continue
  checked=$((checked + 1))

  present=""
  for d in $dirs; do
    [ -d "$d" ] && present="$present $d"
  done
  if [ -z "$present" ]; then
    printf '  %-9s  (no such directory, skipped)\n' "$name"
    continue
  fi

  hits="$(grep -rnE "$PATTERN" $present 2>/dev/null || true)"
  n=0
  [ -n "$hits" ] && n="$(printf '%s\n' "$hits" | wc -l | tr -d ' ')"

  if [ "$n" -gt "$allowed" ]; then
    printf '  %-9s  %3d > %-3d  OVER\n' "$name" "$n" "$allowed"
    printf '%s\n' "$hits" | sed 's/^/      /' >&2
    rc=1
  elif [ "$n" -lt "$allowed" ]; then
    printf '  %-9s  %3d < %-3d  ratchet: lower it to %d if a dependency went away\n' \
      "$name" "$n" "$allowed" "$n"
  elif [ "$allowed" = 0 ]; then
    printf '  %-9s  %3d        clean\n' "$name" "$n"
  else
    printf '  %-9s  %3d        at its floor\n' "$name" "$n"
  fi
done

if [ "$checked" = 0 ]; then
  echo "no section named '${WANT}'" >&2
  exit 2
fi

if [ "$rc" != 0 ]; then
  cat >&2 <<'EOF'

A section named more silicon than it is allowed.

These layers take SI quantities and dt, and must compile without a HAL.
  - need a cycle stamp or the CPU rate?  -> sys/clock.h
  - need a pin, bus, timer or hal_ call? -> it belongs in a driver, not here
  - need sensor data?                    -> take it as an SI sample, not a
                                            driver header (driver/*.h pulls
                                            navhal.h in behind you)

The allowance in tools/dev/check_layering.sh may only go DOWN, and every
section is already at its floor -- so this is a new dependency, not a
leftover. Raising the number is not the fix.
EOF
  exit 1
fi

echo "layering: every section within its allowance"
