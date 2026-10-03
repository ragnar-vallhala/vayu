cmake_minimum_required(VERSION 3.20)
project(vayu C ASM)

# This firmware project lives in firmware/; the shared deps it reaches outward to
# (extern/vaios submodule, navlink/ codec) stay at the repo root one level up.
# Firmware-internal paths use ${CMAKE_CURRENT_SOURCE_DIR} (= firmware/) and need
# no change; only the outward refs below use ${VAYU_REPO_ROOT}.
get_filename_component(VAYU_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

#-------------------------
# Toolchain / basic config
#-------------------------
set(CMAKE_SYSTEM_NAME Generic)
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_BINARY_SIZE ${TOOLCHAIN_PREFIX}size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_COMPILE_COMMANDS_OUTPUT "${CMAKE_SOURCE_DIR}/compile_commands.json")

#-------------------------
# Board / MCU defaults
#-------------------------
set(CMAKE_SYSTEM_PROCESSOR cortex-m4 CACHE STRING "Target processor" FORCE)
set(BOARD nucleo_f401re CACHE STRING "Target board")
set(VENDOR stm32 CACHE STRING "Vendor")

#-------------------------
# External deps
#-------------------------
add_compile_definitions(VAIOS)

# FFT-driven dynamic gyro notch (dsp/gyro_notch). Needs the FPU (float-heavy
# biquad + FFT), so it is gated behind this switch — ON by default for the
# cortex-m4F target. Turning it OFF stubs gyro_notch to passthrough and lets
# --gc-sections drop the notch/FFT code, reclaiming the flash on a size-tight
# build. Runtime is still off until gyro_notch_set_enabled(true).
# Deliberately overflow a task stack into its MPU guard region at boot, to prove
# the guard faults on the access. Halts the FC by design -- a diagnostic, never a
# flight build. See mpu_guard_selftest() in src/sys/heartbeat.c.
option(VAYU_MPU_GUARD_SELFTEST "Fault-inject a stack overflow into the MPU guard" OFF)
if(VAYU_MPU_GUARD_SELFTEST)
  add_compile_definitions(VAYU_MPU_GUARD_SELFTEST)
endif()
option(VAYU_FFT_NOTCH "Compile the FFT dynamic gyro notch" ON)
if(VAYU_FFT_NOTCH)
  add_compile_definitions(VAYU_FFT_NOTCH)
endif()
# vaios defaults the VAIOS_MODULE_VFS option off (opt-in) since VFS couples the
# build to NavHAL's FatFs/SDIO layer. vayu uses VFS (logger, pid_config, bmx160
# calibration), so force the cache option ON before add_subdirectory(extern/vaios)
# below — vaios derives the -DVAIOS_MODULE_VFS compile-def from this variable, so
# this is the single source of truth (don't also -D it, or it redefines).
set(VAIOS_MODULE_VFS ON CACHE BOOL "vayu needs the vaios VFS" FORCE)
set(NAVHAL ON)
# The kernel configuration vayu builds with. vaios 0.2.0 replaced the
# hand-written VAIOS_CONFIG_FILE header with Kconfig: VAIOS_DEFCONFIG seeds
# <build>/.config, and tools/kconfig.py turns that into vaios_autoconf.h, which is
# force-included ahead of every TU (ours too — see below). Must be a FORCE cache
# entry so vaios's own `set(VAIOS_DEFCONFIG ... CACHE)` cannot clobber it; if it
# did, the build would silently take vaios's defaults, where HEAP_SIZE is 0x16000
# and overruns the 96KB SRAM at boot.
set(VAIOS_DEFCONFIG "${CMAKE_CURRENT_SOURCE_DIR}/vaios.defconfig"
    CACHE FILEPATH "vaios Kconfig defconfig for the vayu firmware" FORCE)
# vaios seeds <build>/.config from VAIOS_DEFCONFIG (plus navhal.config) only when
# that file does not yet exist, so in an existing build dir an edit to either
# source would be silently ignored — the firmware would keep building with the old
# values and nothing would say so. Drop a stale seed so the two files stay
# authoritative. The cost is that a `menuconfig` edit does not survive a defconfig
# change; for vayu that is the right way round, since the defconfig is what gets
# reviewed and the build .config is a gitignored artifact.
if(EXISTS "${CMAKE_BINARY_DIR}/.config"
   AND ("${VAIOS_DEFCONFIG}" IS_NEWER_THAN "${CMAKE_BINARY_DIR}/.config"
        OR "${CMAKE_CURRENT_SOURCE_DIR}/navhal.config" IS_NEWER_THAN "${CMAKE_BINARY_DIR}/.config"))
  message(STATUS "vayu: defconfig/navhal.config newer than ${CMAKE_BINARY_DIR}/.config — re-seeding")
  file(REMOVE "${CMAKE_BINARY_DIR}/.config")
endif()
# NavHAL's driver set is owned by vayu (./navhal.config), NOT the vaios submodule.
# vaios exposes NAVHAL_CONFIG_FILE (defaulting to its own navhal.config); we
# FORCE-set it to our copy so NavHAL is configured straight from here WITHOUT ever
# writing into the vaios tree — the same ownership model as VAIOS_CONFIG_FILE
# above (vaios stays genuinely pristine, no `ignore = dirty` mask needed). Must be
# a FORCE cache entry for the same reason as VAIOS_CONFIG_FILE (see note above).
# Edit ./navhal.config to change the firmware driver set.
set(NAVHAL_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/navhal.config"
    CACHE STRING "NavHAL Kconfig .config (driver set) for the vayu firmware" FORCE)

# NavHAL's modular Kconfig (post-M7) uses relative `source "samples/Kconfig"`
# directives, which kconfiglib resolves against $srctree. NavHAL's CMake runs
# kconfig.py without setting it, so point srctree at the NavHAL source root
# here (inherited by the nested execute_process). Avoids editing the vaios /
# NavHAL submodules. Without this, configure fails: "'samples/Kconfig' not
# found ... $srctree ... is unset".
set(ENV{srctree} "${VAYU_REPO_ROOT}/extern/vaios/extern/NavHAL")

add_subdirectory(${VAYU_REPO_ROOT}/extern/vaios ${CMAKE_CURRENT_BINARY_DIR}/vaios)

# vaios's own headers, for vayu's TUs. AFTER add_subdirectory for two reasons:
# VAIOS_PORT is the port directory vaios picked (armv7e-m, derived from the ISA in
# navhal.config — it was cortex-m4 before 0.2.0, and naming it by hand is how that
# rename became a build failure); and keeping this out of vaios's inherited scope
# means vayu's include/sys/types.h (our err_t enum) cannot shadow the toolchain's
# <sys/types.h> when vaios's kernel/syscalls.c includes it, which breaks newlib's
# <unistd.h>. Do NOT add ${CMAKE_CURRENT_SOURCE_DIR}/include above this line.
include_directories(
    ${VAYU_REPO_ROOT}/extern/vaios/include
    ${VAYU_REPO_ROOT}/extern/vaios/portable/${VAIOS_PORT}
)

# Force-include the generated kernel config, exactly as vaios does for its own
# sources. vaios's add_compile_options() is directory-scoped, so it does not reach
# us, and without this every vayu TU that pulls in a vaios header fails on a bare
# config macro (LOG_MSG_MAX_LEN, TICK_PERIOD_US, STACK_ALIGN_SIZE...). The macros
# are #ifndef-guarded, so a -D on the command line still wins.
add_compile_options("SHELL:-include ${CMAKE_BINARY_DIR}/vaios_autoconf.h")

# NavHAL ships src/utils/freestanding.c -- its own memcpy/memmove/memset/memcmp
# -- because a -nostdlib build has no libc to satisfy the calls GCC emits for
# aggregate copies. vayu is not such a build: it links newlib (-lc -lm), so both
# definitions reach the linker and it fails on "multiple definition of memset".
# Drop NavHAL's copies for this build rather than reordering -lc, which leaves
# newlib's assembly implementations in place -- they matter for the 512-byte SD
# block copies, where a naive byte loop does not.
# It rides in via NavHAL's `common` OBJECT library, which libhal absorbs.
get_target_property(_navhal_common_srcs common SOURCES)
list(FILTER _navhal_common_srcs EXCLUDE REGEX "utils/freestanding\\.c$")
set_target_properties(common PROPERTIES SOURCES "${_navhal_common_srcs}")

#-------------------------
# Project sources
#-------------------------
set(CMAKE_C_STANDARD 11)

#-------------------------
# NavLink v2 generated codec (single source of truth: navlink/dialect.json)
#-------------------------
# Phase 0 of the v2 migration (navlink/INTEGRATION.md): generate the C codec at
# configure time into the build tree and compile it into the firmware. Nothing
# generated is committed; it is regenerated on every configure so it can never
# drift from the dialect. Marked SYSTEM so the generated header never trips a
# comm module's -Wconversion/-Werror build (the .c builds under the baseline
# -Wall, not the strict set).
set(NAVLINK_DIR ${VAYU_REPO_ROOT}/navlink)
set(NAVLINK_GEN ${CMAKE_CURRENT_BINARY_DIR}/navlink_gen)
# On-hardware bench firmware (Suite B / C1): the hwtest image gets the
# test-profile codec (adds the HW_TEST_* dialect messages); production builds use
# the prod profile so the real FC carries zero test code. See
# firmware/docs/plans/on-hardware-test-and-coverage.md.
option(VAYU_HW_TEST "Build the on-hardware bench firmware (hwtest) in place of production main" OFF)
if(VAYU_HW_TEST)
  set(NAVLINK_PROFILE test)
else()
  set(NAVLINK_PROFILE prod)
endif()
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${NAVLINK_GEN})
execute_process(
  COMMAND python3 ${NAVLINK_DIR}/generate.py --lang c --profile ${NAVLINK_PROFILE} --out ${NAVLINK_GEN}
  RESULT_VARIABLE NAVLINK_GEN_RC)
if(NOT NAVLINK_GEN_RC EQUAL 0)
  message(FATAL_ERROR "navlink codegen failed (rc=${NAVLINK_GEN_RC})")
endif()
include_directories(SYSTEM ${NAVLINK_GEN}/c)
set(NAVLINK_SOURCES ${NAVLINK_GEN}/c/navlink_msgs.c)

# Board facts (pins, buses, timers) and the board's sensor-backend list. Must
# run before the source glob below, which filters on it. The board directory
# holds only vayu_board.h, so putting it on the include path here does not
# shadow anything the way include/ would (see the note above).
include(${CMAKE_CURRENT_SOURCE_DIR}/board/boards.cmake)

FILE(GLOB_RECURSE VAYU_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
# Sensor backends are selected by the board, not by being on disk: drop every
# file under src/driver/sensor/ and add back only the ones BOARD_SENSOR_DRIVERS
# names (firmware/board/boards.cmake). An unselected driver is never compiled,
# so it costs nothing and registers nothing.
list(FILTER VAYU_SOURCES EXCLUDE REGEX "/src/driver/sensor/")
list(APPEND VAYU_SOURCES ${VAYU_SENSOR_SOURCES})
if(VAYU_HW_TEST)
  # Swap the production entry (src/main.c) for the bench harness
  # (tests/onboard/*.c, which provides its own main()).
  list(REMOVE_ITEM VAYU_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c)
  FILE(GLOB_RECURSE HWTEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/tests/onboard/*.c)
  list(APPEND VAYU_SOURCES ${HWTEST_SOURCES})
  add_compile_definitions(VAYU_HW_TEST)
  message(STATUS "VAYU_HW_TEST: building on-hardware bench firmware (hwtest)")
endif()
add_executable(main ${STARTUP_FILE} ${VAYU_SOURCES} ${NAVLINK_SOURCES})

# Force the FFT gyro notch enabled at boot, overriding whatever pid.bin holds.
# OFF ships: the enable is normally owned by CMD_SET_GYRO_NOTCH and persisted,
# and a build that silently overrides persisted state is a trap. ON is for a
# test airframe where the notch must be known-on without a working uplink to
# send the command -- it logs "notch: FORCED ON by build" at boot so the
# override is visible in telemetry rather than assumed.
#   cmake -S firmware -B firmware/build-notch -DVAYU_GYRO_NOTCH_FORCE_ON=ON
# [TEMP BLOCK TODO: REMOVE]
option(VAYU_GYRO_NOTCH_FORCE_ON
       "Force the gyro notch on at boot, ignoring the persisted enable" OFF)
if(VAYU_GYRO_NOTCH_FORCE_ON)
  target_compile_definitions(main PRIVATE GYRO_NOTCH_FORCE_ON=1)
  message(STATUS "gyro notch: FORCED ON at boot (overrides pid.bin)")
endif()
target_compile_options(main PRIVATE ${BASE_C_FLAGS})
set_target_properties(main PROPERTIES COMPILE_FLAGS "${BASE_C_FLAGS}")
if(VAYU_HW_TEST)
  set_target_properties(main PROPERTIES OUTPUT_NAME hwtest)
  target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests/onboard)
endif()

#-------------------------
# Phase 4 — per-module warning rollout (CONV-06 / R1.2)
#-------------------------
# Sources of modules brought to structural conformance build with the
# full R1.2 warning set and -Werror; the rest of the tree stays at the
# baseline -Wall until its module's turn. Progress tracked in
# docs/firmware/plan/warning-rollout.md. Add a module's TUs here once
# they compile clean under these flags.
set(VAYU_R12_WARN_FLAGS
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wcast-align
    -Wpointer-arith -Wstrict-prototypes -Wmissing-prototypes -Wundef
    -Wfloat-equal -Werror
    # §5.3 vendor exemption: treat vaios/NavHAL headers as system so their
    # warnings never fail a vayu module's -Werror build.
    -isystem ${VAYU_REPO_ROOT}/extern/vaios/include
    -isystem ${VAYU_REPO_ROOT}/extern/vaios/portable/${VAIOS_PORT}
    -isystem ${VAYU_REPO_ROOT}/extern/vaios/extern/NavHAL/include
    -isystem ${VAYU_REPO_ROOT}/extern/vaios/extern/NavHAL/include/port/cortex-m4
    -isystem ${VAYU_REPO_ROOT}/extern/vaios/extern/NavHAL/src/vendor/stm32/family/stm32f4/include)
set(VAYU_R12_CLEAN_SOURCES
    # CALIB module (shared sensor-agnostic ellipsoid fit + engine)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/calib/calib_ellipsoid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/calib/calib_engine.c
    # ACT module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/driver/esc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/actuator/motor.c
    # EST module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/est/sensor_fusion.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/est/ekf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/est/lpf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/est/hover_estimate.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/storage/battery_calib.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/storage/hover_store.c
    # SNS module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/driver/bmx160.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/driver/bme280.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/driver/vl53l0x.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/driver/ride_along.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hub/hub.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/driver/i2c_manager.c
    # CTRL module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/angle_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/angle_rate_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/mixer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/sysid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/pid_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/throttle_curve.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/height_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/control_buffer.c
    # COMM module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/channel.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/serializer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/comm_processor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/telemetry_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/ibus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/rc_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/rc_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/rc_safety.c
    # xfer (FTP) substrate: codec-blind core SM + service task hold to the strict
    # bar; navlink_xfer_tx.c is excluded (codec-touching, like navlink_tx.c).
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/xfer/navlink_xfer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/xfer/xfer_service_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/xfer/fs_query.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/xfer/providers/file_provider.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/xfer/providers/log_provider.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/xfer/providers/stream_provider.c
    # LOG / STORAGE module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/storage/fs_owner.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/logger/log_text.c
    # SYS module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sys/state.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sys/boot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sys/assert.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sys/heartbeat.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sys/clock.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sys/sys_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sys/math_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sys/irq_registry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/driver/timer_callbacks.c)
set_source_files_properties(${VAYU_R12_CLEAN_SOURCES}
    PROPERTIES COMPILE_OPTIONS "${VAYU_R12_WARN_FLAGS}")

#-------------------------
# On-target gcov (C3) — tier 2 coverage
#-------------------------
# Instrument the selected firmware modules with freestanding gcov: each TU's
# gcov_info goes into the .gcov_info linker section (firmware/linker.ld), counters
# into .bss; coverage_dump.c streams the .gcda to SD. -Og keeps line mapping
# faithful; atomic counters because the IMU read task samples concurrently.
# MUST run AFTER the VAYU_R12 set_source_files_properties so these flags win on
# bmx160.c / bme280.c (which are also in VAYU_R12_CLEAN_SOURCES).
#   cmake -S firmware -B build_hwtest_cov -DVAYU_HW_TEST=ON -DVAYU_HW_TEST_COV=ON
option(VAYU_HW_TEST_COV "Instrument selected modules with on-target gcov (needs VAYU_HW_TEST)" OFF)
set(VAYU_HW_TEST_COV_MODULES "driver/bmx160.c;driver/bme280.c;driver/i2c_manager.c"
    CACHE STRING "Sources (relative to src/) compiled with --coverage in the hwtest image")
if(VAYU_HW_TEST AND VAYU_HW_TEST_COV)
  add_compile_definitions(VAYU_HW_TEST_COV)
  set(COV_FLAGS -fprofile-arcs -ftest-coverage -fprofile-info-section
                -fprofile-update=atomic -Og)
  foreach(m ${VAYU_HW_TEST_COV_MODULES})
    set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/${m}
        PROPERTIES COMPILE_OPTIONS "${COV_FLAGS}")
  endforeach()
  message(STATUS "VAYU_HW_TEST_COV: instrumenting ${VAYU_HW_TEST_COV_MODULES}")
endif()


#-------------------------
# Math backend options
#-------------------------
option(USE_STANDARD_MATH "Use standard math.h backend" ON)

#-------------------------
# EKF on-target self-test
#-------------------------
# Compiles src/est/ekf_selftest.c's branch-coverage scenarios into the firmware
# and runs them at boot (src/main.c), logging each check + the total over the
# telemetry UART. Off by default so production firmware carries none of it:
#   cmake -B build_ekftest -DEKF_SELFTEST=ON && cmake --build build_ekftest
option(EKF_SELFTEST "Run the EKF correctness self-test at boot (UART report)" OFF)
if(EKF_SELFTEST)
  add_compile_definitions(EKF_SELFTEST)
  message(STATUS "EKF_SELFTEST enabled: self-test runs at boot")
endif()

#-------------------------
# Simulation build
#-------------------------
# Set -DVAYU_SIM=ON for the host SITL build. Enables the RC override
# path and other sim hooks that bypass real hardware.
option(VAYU_SIM "Build for host SITL simulation" OFF)
if(VAYU_SIM)
  target_compile_definitions(main PRIVATE VAYU_SIM)
  message(STATUS "VAYU_SIM enabled: SITL build")
endif()

#-------------------------
# Compiler flags (embedded friendly)
#-------------------------
# soft-float ABI (no FPU). Keep -fsingle-precision-constant to force float constants.
set(BASE_C_FLAGS
    "-mcpu=${CMAKE_SYSTEM_PROCESSOR} -mthumb -Wall -O2 -g -std=c11 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -D_FPU_ENABLED"
)
set(BASE_ASM_FLAGS "-mcpu=${CMAKE_SYSTEM_PROCESSOR} -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16")

set(CMAKE_C_FLAGS "${BASE_C_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${BASE_ASM_FLAGS}" CACHE STRING "" FORCE)

set(EXTERNAL_LINKER YES)
if(EXTERNAL_LINKER)
  set(CMAKE_EXE_LINKER_FLAGS "-T ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld -lnosys -lc -mfloat-abi=hard -mfpu=fpv4-sp-d16") # can be
else()
set(CMAKE_EXE_LINKER_FLAGS "-T ${LINKER_FILE} -lnosys -lc -mfloat-abi=hard -mfpu=fpv4-sp-d16") # can be
endif()
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include  ${CMAKE_CURRENT_SOURCE_DIR}/drivers ${CMAKE_CURRENT_SOURCE_DIR}/maths)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "cortex-m4")
  add_compile_definitions(CORTEX_M4)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# add_subdirectory(drivers) # implementations for specific architectures (context switching, interrupt handling)
# add_subdirectory(maths) # Maths library (vectors, matrices, quaternions, etc.)
# add_subdirectory(kernel) # The heart of your RTOS (scheduling, tasks, synchronization, etc.) — totally architecture independent
if(SENSOR)
add_subdirectory(sensor) # Small test apps showing how to use the RTOS
else()
    # Linker flags already set above
endif()

#-------------------------
# Link libraries (keep minimal)
#-------------------------
# Do NOT link -lm globally (it pulls in bulky libm implementations).
target_link_libraries(main PRIVATE hal vaios -lgcc -lm)
# libgcov provides __gcov_info_to_gcda / __gcov_filename_to_gcfn + the counter
# merge helpers the instrumented TUs reference (on-target coverage, C3).
if(VAYU_HW_TEST AND VAYU_HW_TEST_COV)
  target_link_libraries(main PRIVATE -lgcov)
endif()


#-------------------------
# Post build: binary + size
#-------------------------
# Which programmer, and it has to be the PHYSICAL PORT, not the serial.
#
# The ST-Links on this bench are clones: every one of them reports the serial
# 303030303030303030303031 (ASCII "000000000001") over the ST-Link protocol,
# whatever the USB descriptor says. So st-flash --serial cannot tell two of
# them apart -- it matches the first that claims that serial -- and with the FC
# and a second board both attached, `flash` silently goes to whichever
# enumerated first. The only symptom is that the board you meant to flash keeps
# behaving like the old firmware.
#
# OpenOCD can pin the adapter to a USB port path, which a clone cannot spoof.
# Find the port for the board you want:
#
#   for d in /sys/bus/usb/devices/*/; do \
#     [ "$(cat $d/idVendor 2>/dev/null)" = 0483 ] && echo "$(basename $d)"; done
#
#   cmake -S firmware -B build -DSTLINK_USB_PORT=3-2
#
# With the port set, `flash` uses OpenOCD and goes to that port and no other.
# Left empty it falls back to st-flash's first-enumerated behaviour and warns,
# which is fine only while exactly one adapter is attached.
set(STLINK_USB_PORT "" CACHE STRING
    "USB port path of the ST-Link to flash, e.g. 3-2 (see the note above)")

if(STLINK_USB_PORT STREQUAL "")
  message(WARNING
    "STLINK_USB_PORT is not set: 'flash' falls back to st-flash and will use "
    "whichever ST-Link enumerates first. These adapters all report the same "
    "serial, so with two attached that is a coin toss -- set "
    "-DSTLINK_USB_PORT=<port> (e.g. 3-2).")
  add_custom_target(flash
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:main> ${CMAKE_CURRENT_BINARY_DIR}/main.bin
    COMMAND ${FLASHER} --connect-under-reset write ${CMAKE_CURRENT_BINARY_DIR}/main.bin ${FLASH_ADDRESS}
    DEPENDS main
    COMMENT "Convert ELF to BIN and flash (UNPINNED: first ST-Link found)"
  )
else()
  find_program(OPENOCD_BIN openocd REQUIRED)
  # `verify` reads the image back and compares; `reset` leaves the board
  # running rather than halted. Both matter here -- a verified write to the
  # wrong board is still the wrong board, so the port pin above is what makes
  # the verify meaningful.
  add_custom_target(flash
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:main> ${CMAKE_CURRENT_BINARY_DIR}/main.bin
    COMMAND ${OPENOCD_BIN}
            -f interface/stlink.cfg
            -c "adapter usb location ${STLINK_USB_PORT}"
            -c "transport select hla_swd"
            -f target/stm32f4x.cfg
            -c "program ${CMAKE_CURRENT_BINARY_DIR}/main.bin ${FLASH_ADDRESS} verify reset exit"
    DEPENDS main
    COMMENT "Convert ELF to BIN and flash via OpenOCD on USB port ${STLINK_USB_PORT}"
  )
endif()

add_custom_command(
  TARGET main POST_BUILD
  COMMAND ${CMAKE_BINARY_SIZE} $<TARGET_FILE:main>
  COMMENT "Calculating size of elf file"
)

message(STATUS "C FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "Linker FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")
