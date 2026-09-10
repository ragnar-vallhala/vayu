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

# Rate/attitude PID core (P/I/D/FF, anti-windup, D-on-measurement, LPF, clamp).
add_executable(test_pid pid_unit_test.c
    ${FW}/src/control/pid.c
    ${FW}/src/maths/maths_interface.c)
target_link_libraries(test_pid m)
add_test(NAME pid COMMAND test_pid)

# Notch (band-stop) biquad: center-tone kill, passband unity, fail-safe bypass.
add_executable(test_biquad biquad_unit_test.c ${FW}/src/maths/biquad.c)
target_link_libraries(test_biquad m)
add_test(NAME biquad COMMAND test_biquad)

# maths_interface: quaternion / vector / scalar helpers.
add_executable(test_maths maths_unit_test.c ${FW}/src/maths/maths_interface.c)
target_link_libraries(test_maths m)
add_test(NAME maths COMMAND test_maths)

# float32 -> float16 telemetry packing (sys/math_utils.c).
add_executable(test_math_utils math_utils_unit_test.c ${FW}/src/sys/math_utils.c)
add_test(NAME math_utils COMMAND test_math_utils)

# FFT notch analysis front-end: tone recovery, peak-pick, band gating (dsp/).
add_executable(test_notch_fft notch_fft_unit_test.c
    ${FW}/src/dsp/notch_fft.c
    ${FW}/src/maths/fft.c)
target_link_libraries(test_notch_fft m)
add_test(NAME notch_fft COMMAND test_notch_fft)

# Dynamic notch bank: tune onto a tone, attenuate it, pass the passband (dsp/).
add_executable(test_notch_bank notch_bank_unit_test.c
    ${FW}/src/dsp/notch_bank.c
    ${FW}/src/dsp/notch_fft.c
    ${FW}/src/maths/fft.c
    ${FW}/src/maths/biquad.c)
target_link_libraries(test_notch_bank m)
add_test(NAME notch_bank COMMAND test_notch_bank)

# In-flight hover-collective estimate: learns hover, refuses to learn from
# climbing / banked / accelerating / implausible samples.
add_executable(test_hover_estimate hover_estimate_unit_test.c
    ${FW}/src/est/hover_estimate.c
    ${FW}/src/maths/maths_interface.c)
target_link_libraries(test_hover_estimate m)
add_test(NAME hover_estimate COMMAND test_hover_estimate)

# Collective stick shaping: hover at mid-stick, linear in thrust, expo.
add_executable(test_throttle_curve throttle_curve_unit_test.c
    ${FW}/src/control/throttle_curve.c
    ${FW}/src/maths/maths_interface.c)
target_link_libraries(test_throttle_curve m)
add_test(NAME throttle_curve COMMAND test_throttle_curve)

# Height hold: the authority-floor clamp, setpoint handling, source handoff.
add_executable(test_height_ctrl height_ctrl_unit_test.c
    ${FW}/src/control/height_controller.c)
target_link_libraries(test_height_ctrl m)
add_test(NAME height_ctrl COMMAND test_height_ctrl)

# On-hardware system-ID chirp: excite / capture / self-abort (control/sysid.c).
add_executable(test_sysid sysid_unit_test.c
    ${FW}/src/control/sysid.c
    ${FW}/src/maths/maths_interface.c)
target_link_libraries(test_sysid m)
add_test(NAME sysid COMMAND test_sysid)
