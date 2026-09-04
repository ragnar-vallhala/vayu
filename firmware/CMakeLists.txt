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
# Must be a FORCE cache entry: vaios's own `set(VAIOS_CONFIG_FILE "" CACHE ...)`
# would otherwise clobber a plain variable to empty (CMP0126 OLD), leaving
# if(VAIOS_CONFIG_FILE) false so vaios never emits -DVAIOS_CONFIG_FILE — then
# the aggregator can't find this app config and kernel TUs (e.g. memory.c) fall
# back to the default HEAP_SIZE=0x16000, overrunning the 96KB SRAM at boot.
set(VAIOS_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/include/vaios_app_config.h"
    CACHE STRING "vaios config header for the vayu firmware" FORCE)
# NOTE: do NOT add ${CMAKE_CURRENT_SOURCE_DIR}/include here. This block is
# inherited by add_subdirectory(extern/vaios) below, and vayu's include/sys/
# types.h (our err_t enum) would shadow the toolchain's standard <sys/types.h>
# when vaios's kernel/syscalls.c includes it — breaking newlib's <unistd.h>
# (mode_t/pid_t/off_t undefined). vayu's own targets pick up include/ from the
# include_directories() call after add_subdirectory(extern/vaios).
include_directories(
    ${VAYU_REPO_ROOT}/extern/vaios/include
    ${VAYU_REPO_ROOT}/extern/vaios/portable/cortex-m4
)

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

FILE(GLOB_RECURSE VAYU_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
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
    -isystem ${VAYU_REPO_ROOT}/extern/vaios/extern/NavHAL/include
    -isystem ${VAYU_REPO_ROOT}/extern/vaios/extern/NavHAL/include/port/cortex-m4
    -isystem ${VAYU_REPO_ROOT}/extern/vaios/extern/NavHAL/src/vendor/stm32/family/stm32f4/include)
set(VAYU_R12_CLEAN_SOURCES
    # CALIB module (shared sensor-agnostic ellipsoid fit + engine)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/calib/calib_ellipsoid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/calib/calib_engine.c
    # ACT module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/actuator/esc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/actuator/motor.c
    # EST module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/est/sensor_fusion.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/est/ekf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/est/lpf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/est/hover_estimate.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/storage/hover_store.c
    # SNS module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor/bmx160.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor/bme280.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor/vl53l0x.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor/imu_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor/i2c_manager.c
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
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sys/sys_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sys/math_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sys/timer_callbacks.c)
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
set(VAYU_HW_TEST_COV_MODULES "sensor/bmx160.c;sensor/bme280.c;sensor/i2c_manager.c"
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
add_custom_target(flash
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:main> ${CMAKE_CURRENT_BINARY_DIR}/main.bin
  COMMAND ${FLASHER} --connect-under-reset write ${CMAKE_CURRENT_BINARY_DIR}/main.bin ${FLASH_ADDRESS}
  DEPENDS main
  COMMENT "Convert ELF to BIN and flash"
)

add_custom_command(
  TARGET main POST_BUILD
  COMMAND ${CMAKE_BINARY_SIZE} $<TARGET_FILE:main>
  COMMENT "Calculating size of elf file"
)

message(STATUS "C FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "Linker FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")
