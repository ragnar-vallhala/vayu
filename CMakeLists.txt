cmake_minimum_required(VERSION 3.20)
project(vayu C ASM)

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
    extern/vaios/include
    extern/vaios/portable/cortex-m4
)

# NavHAL Kconfig is owned by vayu (./navhal.config), NOT the vaios submodule,
# so vaios stays pristine for clean vendor updates. Stage our copy into the
# path vaios's CMake feeds to NavHAL's kconfig.py, before add_subdirectory
# runs that step. The vaios working-tree file is gitignored-as-dirty via
# .gitmodules (ignore = dirty), so this override never shows the submodule
# as modified. Edit ./navhal.config to change the firmware driver set.
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/navhal.config
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/vaios/navhal.config
    COPYONLY)

# NavHAL's modular Kconfig (post-M7) uses relative `source "samples/Kconfig"`
# directives, which kconfiglib resolves against $srctree. NavHAL's CMake runs
# kconfig.py without setting it, so point srctree at the NavHAL source root
# here (inherited by the nested execute_process). Avoids editing the vaios /
# NavHAL submodules. Without this, configure fails: "'samples/Kconfig' not
# found ... $srctree ... is unset".
set(ENV{srctree} "${CMAKE_CURRENT_SOURCE_DIR}/extern/vaios/extern/NavHAL")

add_subdirectory(extern/vaios)

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
set(NAVLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/navlink)
set(NAVLINK_GEN ${CMAKE_CURRENT_BINARY_DIR}/navlink_gen)
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${NAVLINK_GEN})
execute_process(
  COMMAND python3 ${NAVLINK_DIR}/generate.py --lang c --out ${NAVLINK_GEN}
  RESULT_VARIABLE NAVLINK_GEN_RC)
if(NOT NAVLINK_GEN_RC EQUAL 0)
  message(FATAL_ERROR "navlink codegen failed (rc=${NAVLINK_GEN_RC})")
endif()
include_directories(SYSTEM ${NAVLINK_GEN}/c)
set(NAVLINK_SOURCES ${NAVLINK_GEN}/c/navlink_msgs.c)

FILE(GLOB_RECURSE VAYU_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
add_executable(main ${STARTUP_FILE} ${VAYU_SOURCES} ${NAVLINK_SOURCES})
target_compile_options(main PRIVATE ${BASE_C_FLAGS})
set_target_properties(main PROPERTIES COMPILE_FLAGS "${BASE_C_FLAGS}")

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
    -isystem ${CMAKE_CURRENT_SOURCE_DIR}/extern/vaios/include
    -isystem ${CMAKE_CURRENT_SOURCE_DIR}/extern/vaios/extern/NavHAL/include
    -isystem ${CMAKE_CURRENT_SOURCE_DIR}/extern/vaios/extern/NavHAL/include/port/cortex-m4
    -isystem ${CMAKE_CURRENT_SOURCE_DIR}/extern/vaios/extern/NavHAL/src/vendor/stm32/family/stm32f4/include)
set(VAYU_R12_CLEAN_SOURCES
    # ACT module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/actuator/esc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/actuator/motor.c
    # EST module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/est/sensor_fusion.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/est/ekf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/est/lpf.c
    # SNS module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor/bmx160.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor/bme280.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor/imu_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor/i2c_manager.c
    # CTRL module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/angle_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/angle_rate_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/sysid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/pid_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/pid.c
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
    # LOG module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/logger/logger.c
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
