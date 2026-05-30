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
set(NAVHAL ON)
set(VAIOS_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/include/vaios_app_config.h")
include_directories(
    extern/vaios/include
    extern/vaios/portable/cortex-m4
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
add_subdirectory(extern/vaios)

#-------------------------
# Project sources
#-------------------------
set(CMAKE_C_STANDARD 11)
FILE(GLOB_RECURSE VAYU_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
add_executable(main ${STARTUP_FILE} ${VAYU_SOURCES})
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
    ${CMAKE_CURRENT_SOURCE_DIR}/src/est/lpf.c
    # SNS module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor/bmx160.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor/imu_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor/i2c_manager.c
    # CTRL module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/angle_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/angle_rate_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/pid_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/control_buffer.c
    # COMM module (Phase 4)
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/channel.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/serializer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/deserializer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/comm_processor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/telemetry_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/ibus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/rc_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/rc_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comm/rc_safety.c)
set_source_files_properties(${VAYU_R12_CLEAN_SOURCES}
    PROPERTIES COMPILE_OPTIONS "${VAYU_R12_WARN_FLAGS}")


#-------------------------
# Math backend options
#-------------------------
option(USE_STANDARD_MATH "Use standard math.h backend" ON)

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
