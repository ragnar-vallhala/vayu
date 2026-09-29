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
# Board selection.
#
# One directory per flight-controller board under firmware/board/<name>/, each
# shipping a vayu_board.h with the same macro names (see board/README.md). The
# selected directory goes on the include path, so every consumer writes
#
#     #include "vayu_board.h"
#
# and never names a board. Switching boards is this one variable.
#
#     cmake -S firmware -B build -DVAYU_BOARD=navixsm_f401re
#
# This mirrors NavHAL's own src/board/<name>/board.h layout on purpose: when a
# board graduates into NavHAL it is a git mv plus a Kconfig entry, and vayu's
# copy is deleted. Until then the two live side by side, which is why the
# header is vayu_board.h and not board.h -- NavHAL's selected board directory is
# ALREADY on our include path, so a second board.h would be a coin toss decided
# by -I order, and the loser is a board built with the wrong pins.

# Paths resolve against THIS file, not the includer, so sim/host can include
# it and get the same board and the same consistency check as the firmware.
set(VAYU_BOARD "navixdev" CACHE STRING "Flight-controller board (firmware/board/<name>)")

set(_VAYU_BOARD_DIR "${CMAKE_CURRENT_LIST_DIR}/${VAYU_BOARD}")
if(NOT EXISTS "${_VAYU_BOARD_DIR}/vayu_board.h")
  file(GLOB _known RELATIVE "${CMAKE_CURRENT_LIST_DIR}"
       "${CMAKE_CURRENT_LIST_DIR}/*/vayu_board.h")
  string(REPLACE "/vayu_board.h" "" _known "${_known}")
  message(FATAL_ERROR
    "VAYU_BOARD='${VAYU_BOARD}' has no firmware/board/${VAYU_BOARD}/vayu_board.h.\n"
    "Known boards: ${_known}\n"
    "Adding one: see firmware/board/README.md.")
endif()

# Each board states the NavHAL board it must be built against, because the two
# selections are independent and a mismatch is silent: NavHAL's choice drives
# the linker script and the MCU family, vayu's drives the pins. Disagreeing
# means flashing F4 pin numbers onto an H7, or a linker script for the wrong
# part. The board header owns the expected value; we read it back and check.
file(STRINGS "${_VAYU_BOARD_DIR}/vayu_board.h" _want
     REGEX "^#define[ \t]+VAYU_BOARD_NAVHAL_BOARD[ \t]+")
string(REGEX REPLACE ".*VAYU_BOARD_NAVHAL_BOARD[ \t]+\"([^\"]+)\".*" "\\1" _want "${_want}")

file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/../navhal.config" _have
     REGEX "^CONFIG_BOARD=")
string(REGEX REPLACE ".*CONFIG_BOARD=\"([^\"]+)\".*" "\\1" _have "${_have}")

if(NOT _want STREQUAL _have)
  message(FATAL_ERROR
    "Board selection mismatch.\n"
    "  VAYU_BOARD=${VAYU_BOARD} expects NavHAL board '${_want}'\n"
    "  firmware/navhal.config selects   '${_have}'\n"
    "Set CONFIG_BOARD (and its CONFIG_BOARD_* / CONFIG_FAMILY_* lines) in "
    "firmware/navhal.config to match, or pick a different VAYU_BOARD.")
endif()

# Which sensor backends this board carries. Everything else under
# src/driver/sensor/ is left out of the build entirely -- see board.cmake.
include("${_VAYU_BOARD_DIR}/board.cmake")
if(NOT DEFINED BOARD_SENSOR_DRIVERS)
  message(FATAL_ERROR
    "firmware/board/${VAYU_BOARD}/board.cmake must set BOARD_SENSOR_DRIVERS.")
endif()

# Turn the names into paths, and fail on one that does not exist rather than
# silently booting a board with no IMU.
set(VAYU_SENSOR_SOURCES "")
foreach(_drv ${BOARD_SENSOR_DRIVERS})
  set(_src "${CMAKE_CURRENT_LIST_DIR}/../src/driver/sensor/${_drv}.c")
  if(NOT EXISTS "${_src}")
    file(GLOB _avail RELATIVE "${CMAKE_CURRENT_LIST_DIR}/../src/driver/sensor"
         "${CMAKE_CURRENT_LIST_DIR}/../src/driver/sensor/*.c")
    string(REPLACE ".c" "" _avail "${_avail}")
    message(FATAL_ERROR
      "Board ${VAYU_BOARD} wants sensor driver '${_drv}', which has no "
      "firmware/src/driver/sensor/${_drv}.c.\nAvailable: ${_avail}")
  endif()
  list(APPEND VAYU_SENSOR_SOURCES "${_src}")
endforeach()

message(STATUS "vayu board: ${VAYU_BOARD} (on NavHAL board ${_have})")
message(STATUS "vayu sensors: ${BOARD_SENSOR_DRIVERS}")
include_directories(${_VAYU_BOARD_DIR})
