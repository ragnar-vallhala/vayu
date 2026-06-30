# Host unit tests for firmware logic — a standalone CTest project built with the
# HOST compiler (NOT the firmware ARM toolchain). Each test is a self-contained
# main() that links the few firmware sources it exercises.
#
#   cmake -S firmware/tests/host -B build_fwtest && ctest --test-dir build_fwtest
#
# vtest (the repo-root orchestrator) drives this via its ctest adapter, the same
# way it drives sim/host and navigator. Add a new test by dropping a *_test.c
# here and registering it below.
cmake_minimum_required(VERSION 3.16)
project(vayu_fw_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF) # strict -std=c11, matching the host recipes
enable_testing()

get_filename_component(FW "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
# -iquote (NOT -I): firmware/include resolves only "..." includes, so its
# sys/types.h never shadows the system <sys/types.h> (a repo gotcha).
add_compile_options(-Wall -Wextra -iquote ${FW}/include)

# FFT engine (firmware/src/maths/fft.c) vs a naive DFT.
add_executable(test_fft fft_unit_test.c ${FW}/src/maths/fft.c)
target_link_libraries(test_fft m)
add_test(NAME fft COMMAND test_fft)

# Control-allocation mixer + airmode.
add_executable(test_mixer mixer_unit_test.c
    ${FW}/src/control/mixer.c
    ${FW}/src/maths/maths_interface.c)
target_link_libraries(test_mixer m)
add_test(NAME mixer COMMAND test_mixer)
