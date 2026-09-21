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
# The rule (docs/analysis/navhal-hardware-logic-separation.md §5.1): no
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
# Usage: check_layering.sh   (exit 1 on any violation)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

DIRS=(
  firmware/src/control firmware/src/est firmware/src/maths
  firmware/include/control firmware/include/est firmware/include/maths
)

# Each pattern is a class of silicon fact, not a blocklist of names -- a new
# NavHAL call or a new pin macro is caught without touching this file.
#
# driver/ is in here because a device header reaches silicon on the logic
# layer's behalf: driver/bmx160.h includes navhal.h, so a control TU that
# includes it gets the whole register map without ever typing "hal_". An
# umbrella header made that invisible -- this check reported clean while
# control/ transitively included navhal.h through driver/driver.h. Naming a
# driver is itself the violation; logic takes SI samples from hub/, not
# devices. This ran with a list of known exceptions while those consumers were
# migrated; the list reached zero and was deleted, which is what a migration
# ledger is for.
PATTERN='#include[[:space:]]*"(navhal|hal_|driver/)[^"]*"|\bhal_[a-z0-9_]+[[:space:]]*\(|\bGPIO_P[A-Z][0-9]|\bHAL_(I2C|UART|SPI|GPIO|PWM|TIM|DMA)|\bTIM[0-9]+\b'

hits="$(grep -rnE "$PATTERN" "${DIRS[@]}" 2>/dev/null || true)"

if [ -n "$hits" ]; then
  cat >&2 <<'EOF'
Layering violation: silicon named inside control/ est/ maths/.

These layers take SI quantities and dt, and must compile without a HAL.
  - need a cycle stamp or the CPU rate?  -> sys/clock.h
  - need a pin, bus, timer or hal_ call? -> it belongs in a driver, not here
  - need sensor data?                    -> take it as an SI sample, not a
                                            driver header (driver/*.h pulls
                                            navhal.h in behind you)

EOF
  echo "$hits" >&2
  exit 1
fi

echo "layering: control/ est/ maths/ name no silicon"
