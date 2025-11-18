cmake_minimum_required(VERSION 3.20)

# Prevent macOS-specific flags
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_OSX_ARCHITECTURES "" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_INIT "" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "" CACHE STRING "" FORCE)

# Toolchain settings
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_BINARY_SIZE ${TOOLCHAIN_PREFIX}size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

project(vayu C ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_COMPILE_COMMANDS_OUTPUT "${CMAKE_SOURCE_DIR}/compile_commands.json")
# Defaults
set(CMAKE_SYSTEM_PROCESSOR
    cortex-m4
    CACHE STRING "Target processor" FORCE) # Processor on board
set(BOARD
    nucleo_f401re
    CACHE STRING "Target board")
set(VENDOR
    stm32
    CACHE STRING "Vendor")


set(CMAKE_C_STANDARD 11)

add_compile_definitions(VAIOS)
set(NAVHAL ON)
include_directories(extern/vaios/include extern/vaios/portable/cortex-m4 ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_subdirectory(extern/vaios)

message("Linker File for Vayu: ${LINKER_FILE}")
message("Startup File for Vayu: ${STARTUP_FILE}")

set(CMAKE_C_FLAGS
"-mcpu=${CMAKE_SYSTEM_PROCESSOR} -mthumb -Wall -O0 -g -std=c11 -mfloat-abi=soft  -fsingle-precision-constant") # C++ standard 11

set(CMAKE_ASM_FLAGS "-mcpu=${CMAKE_SYSTEM_PROCESSOR} -mthumb -mfloat-abi=soft")


if(EXTERNAL_LINKER)
  set(CMAKE_EXE_LINKER_FLAGS "-T ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld -lnosys -lc -mfloat-abi=soft") # can be
else()
set(CMAKE_EXE_LINKER_FLAGS "-T ${LINKER_FILE} -lnosys -lc -mfloat-abi=soft") # can be
endif()

FILE(GLOB_RECURSE VAYU_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)

add_executable(main ${STARTUP_FILE} ${VAYU_SOURCES})

target_link_libraries(main PRIVATE
    hal
   -lgcc
   -lm
    vaios
)

add_custom_target(flash
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:main> ${CMAKE_CURRENT_BINARY_DIR}/main.bin
  COMMAND ${FLASHER} write ${CMAKE_CURRENT_BINARY_DIR}/main.bin ${FLASH_ADDRESS}
  DEPENDS main
  COMMENT "Converting ELF to BIN and flashing to board"
)

message(STATUS "TARGET File ${CMAKE_CURRENT_BINARY_DIR}/main.bin")
# elf file size calculation
add_custom_command(
  TARGET main
  POST_BUILD
  COMMAND ${CMAKE_BINARY_SIZE} $<TARGET_FILE:main>
  COMMENT "Calculating size of elf file")


