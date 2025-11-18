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


#-------------------------
# Math backend options
#-------------------------
option(USE_STANDARD_MATH "Use standard math.h backend" OFF)
option(USE_ARM_DSP "Use ARM CMSIS DSP math backend (small fast-math)" ON)
option(USE_ARM_DSP_TABLES "Include fast-math lookup tables (bigger flash)" OFF)

#-------------------------
# Compiler flags (embedded friendly)
#-------------------------
# soft-float ABI (no FPU). Keep -fsingle-precision-constant to force float constants.
set(BASE_C_FLAGS
    "-mcpu=${CMAKE_SYSTEM_PROCESSOR} -mthumb -Wall -O2 -g -std=c11 -mfloat-abi=soft -fsingle-precision-constant"
)
set(BASE_ASM_FLAGS "-mcpu=${CMAKE_SYSTEM_PROCESSOR} -mthumb -mfloat-abi=soft")

# Put functions/data in sections so linker can gc unused code
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${BASE_C_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${BASE_ASM_FLAGS}")
# Linker flags: use nano.specs to reduce newlib overhead, enable gc-sections
message(STATUS "Linker file is: ${LINKER_FILE}")
message(STATUS "Startup file is: ${STARTUP_FILE}")
if(EXTERNAL_LINKER)
    set(CMAKE_EXE_LINKER_FLAGS "-T ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld -specs=nano.specs -Wl,--gc-sections -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/main.map -mfloat-abi=soft -lc -lgcc ")
else()
    set(CMAKE_EXE_LINKER_FLAGS "-T ${LINKER_FILE} -specs=nano.specs -Wl,--gc-sections -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/main.map -mfloat-abi=soft -lc -lgcc ")
endif()

#-------------------------
# Link libraries (keep minimal)
#-------------------------
# Do NOT link -lm globally (it pulls in bulky libm implementations).
target_link_libraries(main PRIVATE hal vaios -lgcc -lm)

#-------------------------
# CMSIS fast-math minimal lib
#-------------------------
if(USE_ARM_DSP)
    # ensure we define CPU for CMSIS
    add_compile_definitions(ARM_MATH_CM4)
    # include directories used by the small dsp lib
    set(CMSIS_CORE_INC extern/CMSIS_5/CMSIS/Core/Include)
    set(CMSIS_DSP_INC  extern/CMSIS-DSP/Include)
    set(CMSIS_DSP_PRI_INC  extern/CMSIS-DSP/PrivateInclude)
    include_directories(${CMSIS_CORE_INC} ${CMSIS_DSP_INC} ${CMSIS_DSP_PRI_INC})

    # Build a tiny library containing ONLY the fast sin/cos (and tables if enabled).
    # NOTE: arm_common_tables.c needs core include path (arm_compiler_specific.h, etc)
    set(CMSIS_FAST_SOURCES
        ${CMAKE_SOURCE_DIR}/extern/CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.c
        ${CMAKE_SOURCE_DIR}/extern/CMSIS-DSP/Source/FastMathFunctions/arm_cos_f32.c
        ${CMAKE_SOURCE_DIR}/extern/CMSIS-DSP/Source/CommonTables/arm_common_tables.c
    )

    if(USE_ARM_DSP_TABLES)
        # include the fast math tables implementation (this is the large table file)
        list(APPEND CMSIS_FAST_SOURCES
            ${CMAKE_SOURCE_DIR}/extern/CMSIS-DSP/Source/FastMathFunctions/arm_fast_math_tables.c
        )
        add_compile_definitions(ARM_FAST_MATH_TABLES)
    endif()

    add_library(cmsis_fast_math STATIC ${CMSIS_FAST_SOURCES})
    target_include_directories(cmsis_fast_math PUBLIC ${CMSIS_CORE_INC} ${CMSIS_DSP_INC})
    # Link only the small library
    target_link_libraries(main PRIVATE cmsis_fast_math)
    # Also expose USE_ARM_DSP macro to project sources
    target_compile_definitions(main PRIVATE USE_ARM_DSP)
endif()

#-------------------------
# Post build: binary + size
#-------------------------
add_custom_target(flash
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:main> ${CMAKE_CURRENT_BINARY_DIR}/main.bin
  COMMAND ${FLASHER} write ${CMAKE_CURRENT_BINARY_DIR}/main.bin ${FLASH_ADDRESS}
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
message(STATUS "USE_ARM_DSP = ${USE_ARM_DSP}")
message(STATUS "USE_ARM_DSP_TABLES = ${USE_ARM_DSP_TABLES}")
