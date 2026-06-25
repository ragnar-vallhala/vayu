cmake_minimum_required(VERSION 3.20)
project(vsim_d CXX)

# Standalone physics daemon. Zero external deps -- pure C++17 plus
# POSIX (FIFOs, signals, threads). Talks to Navigator + the firmware
# host via the four FIFOs defined in include/vsim_proto.h.
#
# Configure:  cmake -S sim/vsim -B build_vsim
# Build:      cmake --build build_vsim
# Run:        ./build_vsim/vsim_d

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Mirror the workspace-wide compile_commands.json drop point so clangd
# picks up the daemon target alongside everything else.
get_filename_component(VAYU_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(CMAKE_COMPILE_COMMANDS_OUTPUT "${VAYU_ROOT}/compile_commands.json")

add_compile_options(-Wall -Wextra -O2 -g)

add_executable(vsim_d
    src/physics_core.cpp
    src/motor_model.cpp
    src/sensor_models.cpp
    src/sim_controller.cpp
    src/fifo_transport.cpp
    src/main.cpp
)
target_include_directories(vsim_d PRIVATE include)

find_package(Threads REQUIRED)
target_link_libraries(vsim_d PRIVATE Threads::Threads)

message(STATUS "vsim_d: standalone physics daemon configured")
