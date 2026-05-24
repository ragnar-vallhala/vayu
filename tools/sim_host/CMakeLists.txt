cmake_minimum_required(VERSION 3.20)
project(vayu_sitl C)

# Native host SITL build for vayu. Reuses the firmware controller sources
# (src/control, src/maths, src/utils, src/actuator/motor.c) and links them
# against host shims for the vaios kernel + NavHAL.
#
# Configure:  cmake -S tools/sim_host -B build_sitl
# Build:      cmake --build build_sitl
# Run:        ./build_sitl/vayu_sitl

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

get_filename_component(VAYU_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(VAIOS_ROOT  "${VAYU_ROOT}/extern/vaios")
set(NAVHAL_ROOT "${VAIOS_ROOT}/extern/NavHAL")

add_compile_options(-Wall -Wextra -O2 -g -fno-strict-aliasing)
add_compile_definitions(
    VAYU_SIM=1
    VAYU_SITL=1
    CORTEX_M4=1   # bypass the architecture #error in vaios/task.h; we shim
                  # the ARM-specific bits via tools/sim_host/include/port.h
    NAVHAL=1
    VAIOS_CONFIG_FILE="${VAYU_ROOT}/include/vaios_app_config.h"
)

# Shadow + firmware + vaios + NavHAL headers, in priority order.
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VAYU_ROOT}/include
    ${VAIOS_ROOT}/include
    ${NAVHAL_ROOT}/include
    ${NAVHAL_ROOT}/include/port/cortex-m4   # utility type-only headers
)

# Host shim sources.
set(HOST_SOURCES
    src/host_main.c
    src/host_port.c
    src/host_vaios.c
    src/host_navhal.c
    src/host_imu_feeder.c
)

# Firmware sources the host binary reuses verbatim.
set(VAYU_SOURCES
    ${VAYU_ROOT}/src/actuator/esc.c
    ${VAYU_ROOT}/src/actuator/motor.c
    ${VAYU_ROOT}/src/control/angle_controller.c
    ${VAYU_ROOT}/src/control/angle_rate_controller.c
    ${VAYU_ROOT}/src/maths/control_buffer.c
    ${VAYU_ROOT}/src/maths/lpf.c
    ${VAYU_ROOT}/src/maths/maths_interface.c
    ${VAYU_ROOT}/src/maths/pid.c
    ${VAYU_ROOT}/src/utils/math_utils.c
    ${VAYU_ROOT}/src/maths/sensor_fusion.c
    ${VAYU_ROOT}/src/sensor/imu_buffer.c
    ${VAYU_ROOT}/src/sys/state.c
    ${VAYU_ROOT}/src/comm/rc_buffer.c
)

# vaios kernel pieces that are pure C and portable.
set(VAIOS_HOST_SOURCES
    ${VAIOS_ROOT}/kernel/structure.c
)

add_executable(vayu_sitl
    ${HOST_SOURCES}
    ${VAYU_SOURCES}
    ${VAIOS_HOST_SOURCES}
)

find_package(Threads REQUIRED)
target_link_libraries(vayu_sitl PRIVATE Threads::Threads m rt)

message(STATUS "vayu_sitl: native host SITL build configured")
