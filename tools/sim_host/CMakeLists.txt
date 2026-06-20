cmake_minimum_required(VERSION 3.20)
project(vayu_sitl C CXX)   # CXX for the in-process vsim physics (Phase 4 RTOS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# Phase 5 — sanitizers (R12.2) and coverage (CONV-05 / R12.4).
option(VAYU_SANITIZE "Build host SITL + tests with ASan + UBSan" OFF)
option(VAYU_COVERAGE "Build host SITL + tests with gcov instrumentation" OFF)
if(VAYU_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()
if(VAYU_COVERAGE)
    add_compile_options(--coverage -O0)
    add_link_options(--coverage)
endif()
add_compile_definitions(
    VAYU_SIM=1
    VAYU_SITL=1
    CORTEX_M4=1   # bypass the architecture #error in vaios/task.h; we shim
                  # the ARM-specific bits via tools/sim_host/include/port.h
    NAVHAL=1
    VAIOS_CONFIG_FILE="${VAYU_ROOT}/include/vaios_app_config.h"
)

# Shadow + firmware + vaios + NavHAL + vsim_proto headers, in priority order.
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VAYU_ROOT}/include
    ${VAIOS_ROOT}/include
    ${NAVHAL_ROOT}/include
    ${NAVHAL_ROOT}/include/port/cortex-m4   # utility type-only headers
    ${VAYU_ROOT}/tools/vsim/include         # vsim_proto.h (shared with vsim_d)
)

# Host shim sources -- everything that goes into the library.
# main() lives separately so only the binary picks it up.
set(HOST_SOURCES
    src/host_port.c
    src/host_vaios.c
    src/host_navhal.c
    src/host_imu_feeder.c
    src/host_baro.c
    src/host_rc_feeder.c
    src/host_lifecycle.c
    src/host_vfs.c
    src/vsim_iface.c
)
set(HOST_MAIN_SOURCES
    src/host_main.c
)

# Firmware sources the host binary reuses verbatim.
set(VAYU_SOURCES
    ${VAYU_ROOT}/src/actuator/esc.c
    ${VAYU_ROOT}/src/actuator/motor.c
    ${VAYU_ROOT}/src/control/angle_controller.c
    ${VAYU_ROOT}/src/control/angle_rate_controller.c
    ${VAYU_ROOT}/src/control/pid_config.c   # CMD_SET_PID apply + SD persistence
    ${VAYU_ROOT}/src/control/flight_mode.c  # stabilise/acro arbitration (RC + GCS)
    ${VAYU_ROOT}/src/control/control_buffer.c
    ${VAYU_ROOT}/src/est/lpf.c
    ${VAYU_ROOT}/src/maths/maths_interface.c
    ${VAYU_ROOT}/src/control/pid.c
    ${VAYU_ROOT}/src/sys/math_utils.c
    ${VAYU_ROOT}/src/sys/sys_utils.c
    ${VAYU_ROOT}/src/est/sensor_fusion.c
    ${VAYU_ROOT}/src/est/ekf.c
    ${VAYU_ROOT}/src/est/attitude_task.c   # the REAL firmware estimator task (now run in SITL)
    ${VAYU_ROOT}/src/est/vertical_estimator.c  # 2-state baro+accel vertical estimator (VERT)
    ${VAYU_ROOT}/src/est/vertical_task.c       # VERT task I/O wrapper (sibling of attitude_task)
    ${VAYU_ROOT}/src/est/flight_phase.c        # takeoff/landing detector + FC-owned AGL (IN_AIR)
    ${VAYU_ROOT}/src/sensor/imu_buffer.c
    ${VAYU_ROOT}/src/sensor/bme280.c   # real FC baro driver (altitude/publish/getters); host_baro.c feeds it
    ${VAYU_ROOT}/src/logger/logger.c    # wrap counters; telemetry reads them
    ${VAYU_ROOT}/src/logger/log_text.c  # vayu_log + queue (moved from utils.c)
    ${VAYU_ROOT}/src/sys/state.c
    ${VAYU_ROOT}/src/sys/assert.c
    ${VAYU_ROOT}/src/comm/rc_buffer.c
    ${VAYU_ROOT}/src/comm/rc_safety.c   # RC watchdog + arm gate (host-buildable)
    # Telemetry chain: produces DroneProtocol packets the GCS reads.
    # UART2 in host_navhal pipes them to a pty (for live GCS attach) and
    # tees a raw copy to /tmp/vayu_uart2.log (for post-sim analysis).
    ${VAYU_ROOT}/src/comm/channel.c
    ${VAYU_ROOT}/src/comm/perf_packet.c  # perf_fifo_fill_row (buffers link it)
    ${VAYU_ROOT}/src/comm/serializer.c
    ${VAYU_ROOT}/src/comm/comm_processor.c
    ${VAYU_ROOT}/src/comm/navlink_router.c  # v2 RX handler table (uplink)
    ${VAYU_ROOT}/src/comm/navlink_tx.c      # TX seam (telemetry/cmd-resp framing)
    ${VAYU_ROOT}/src/comm/telemetry_task.c
)

# vaios kernel pieces that are pure C and portable.
set(VAIOS_HOST_SOURCES
    ${VAIOS_ROOT}/kernel/structure.c
)

# ----------------------------------------------------------------------------
# Targets
#
# libvayu_sitl_core.a (static): firmware sources + host shims + vsim_iface +
# the vayu_sitl_start / vayu_sitl_stop API. The Navigator GCS links this in
# directly so it can spawn the firmware threads inside its own process.
#
# vayu_sitl (executable): thin wrapper around the library that calls
# vayu_sitl_start(NULL) so the standalone binary still works for offline
# tests with the legacy /tmp FIFOs.
# ----------------------------------------------------------------------------

add_library(vayu_sitl_core STATIC
    ${HOST_SOURCES}
    ${HOST_MAIN_SOURCES}
    ${VAYU_SOURCES}
    ${VAIOS_HOST_SOURCES}
)
target_include_directories(vayu_sitl_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VAYU_ROOT}/include
    ${VAIOS_ROOT}/include
    ${NAVHAL_ROOT}/include
    ${NAVHAL_ROOT}/include/port/cortex-m4
)

# NavLink v2 codec: the firmware comm sources (telemetry_task.c) now emit v2
# frames, so the host SITL core needs the generated codec too. Reuse the parent
# build's navlink_codec target when present (the Navigator build defines it);
# otherwise generate + build our own so standalone SITL configs still work.
if(NOT TARGET navlink_codec)
  if(NOT DEFINED NAVLINK_GEN)
    set(NAVLINK_GEN ${CMAKE_CURRENT_BINARY_DIR}/navlink_gen)
    execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${NAVLINK_GEN})
    execute_process(
      COMMAND python3 ${VAYU_ROOT}/navlink/generate.py --lang c --out ${NAVLINK_GEN}
      RESULT_VARIABLE NAVLINK_GEN_RC)
    if(NOT NAVLINK_GEN_RC EQUAL 0)
      message(FATAL_ERROR "navlink codegen failed (rc=${NAVLINK_GEN_RC})")
    endif()
  endif()
  add_library(navlink_codec STATIC ${NAVLINK_GEN}/c/navlink_msgs.c)
  target_include_directories(navlink_codec SYSTEM PUBLIC ${NAVLINK_GEN}/c)
endif()

find_package(Threads REQUIRED)
target_link_libraries(vayu_sitl_core PUBLIC Threads::Threads m rt navlink_codec)

add_executable(vayu_sitl ${HOST_MAIN_SOURCES})
# main() is also in the library, so the executable picks the library's
# definition. Linker uses the .o from the explicit source. Same code,
# same behavior as before.
target_link_libraries(vayu_sitl PRIVATE vayu_sitl_core)

message(STATUS "vayu_sitl: static library + standalone binary configured")

# ----------------------------------------------------------------------------
# Phase 4 (experimental, opt-in): vayu_sitl_rtos
#
# Runs the REAL vaios scheduler on the host via the ucontext port
# (host_rtos_port.c) instead of the legacy pthread shims — deterministic and
# maximally faithful (the flight scheduler itself is under test). The host_vaios
# collision set (task/delay/sema/mutex/heap/scheduler/v_init) is compiled out by
# -DVAYU_SITL_RTOS and provided by the kernel; -DVAIOS_HOST_TEST drops the
# kernel's target-SRAM pointer guards. See docs/plans/sitl-lockstep-sim.md.
#
#   cmake -S tools/sim_host -B build_sitl_rtos -DVAYU_SITL_RTOS_BUILD=ON
#   cmake --build build_sitl_rtos --target vayu_sitl_rtos
# ----------------------------------------------------------------------------
option(VAYU_SITL_RTOS_BUILD "Build the experimental real-vaios-scheduler SITL" OFF)
if(VAYU_SITL_RTOS_BUILD)
  # In-process vsim physics as an ISOLATED C++ library: it must NOT see the
  # firmware's cortex-m4/vaios/navhal include env (which shadows system headers
  # for C++ TUs), so override INCLUDE_DIRECTORIES to just the vsim path — same
  # clean environment as the standalone build_vsim.
  add_library(vsim_phys STATIC
      src/vsim_inproc.cpp
      ${VAYU_ROOT}/tools/vsim/src/sim_controller.cpp
      ${VAYU_ROOT}/tools/vsim/src/physics_core.cpp
      ${VAYU_ROOT}/tools/vsim/src/motor_model.cpp
      ${VAYU_ROOT}/tools/vsim/src/sensor_models.cpp
  )
  set_target_properties(vsim_phys PROPERTIES
      INCLUDE_DIRECTORIES "${VAYU_ROOT}/tools/vsim/include")
  target_include_directories(vsim_phys INTERFACE ${VAYU_ROOT}/tools/vsim/include)

  add_executable(vayu_sitl_rtos
      src/host_rtos_port.c
      src/host_rtos_main.c
      src/host_vaios.c        # collisions compiled out by VAYU_SITL_RTOS
      src/host_navhal.c
      src/host_vfs.c
      src/host_baro.c         # provides the i2c_manager_* stubs bme280.c needs
      src/host_imu_feeder.c   # single-shot pump for the stepper (thread unused)
      src/host_lifecycle.c
      src/vsim_iface.c
      # host_port.c (legacy critical_nesting + mutex) is replaced by the port's
      # own critical_nesting; not linked here.
      ${VAYU_SOURCES}
      ${VAIOS_ROOT}/kernel/structure.c
      ${VAIOS_ROOT}/kernel/task.c
      ${VAIOS_ROOT}/kernel/ipc.c
      ${VAIOS_ROOT}/kernel/memory.c
      ${VAIOS_ROOT}/kernel/vaios.c
      ${VAIOS_ROOT}/kernel/perf.c
  )
  target_compile_definitions(vayu_sitl_rtos PRIVATE
      VAYU_SITL_RTOS=1 VAIOS_HOST_TEST=1
      _GNU_SOURCE           # glibc __USE_MISC (int32_t in <stdlib.h>) for the C++ TUs
      HEAP_SIZE=0x180000)   # 1.5 MB: room for the firmware tasks' 8KB stacks on host
  target_link_libraries(vayu_sitl_rtos PRIVATE Threads::Threads m rt navlink_codec vsim_phys)
  message(STATUS "vayu_sitl_rtos (Phase 4, experimental) configured")
endif()

# ----------------------------------------------------------------------------
# Verification suite
#
# Test binaries provide their own main() and link the firmware sources
# from the static library. The library's host_main.o is not pulled in
# (its main symbol is already satisfied by the test's own main).
# Run with:  ctest --test-dir build_sitl   (or invoke the binary directly).
# ----------------------------------------------------------------------------
enable_testing()

add_executable(test_safety_phase2 tests/test_safety_phase2.c)
target_link_libraries(test_safety_phase2 PRIVATE vayu_sitl_core)
add_test(NAME safety_phase2 COMMAND test_safety_phase2)

add_executable(test_phase3_ctrl tests/test_phase3_ctrl.c)
target_link_libraries(test_phase3_ctrl PRIVATE vayu_sitl_core m)
add_test(NAME phase3_ctrl COMMAND test_phase3_ctrl)

add_executable(test_phase3_comm tests/test_phase3_comm.c)
target_link_libraries(test_phase3_comm PRIVATE vayu_sitl_core m)
add_test(NAME phase3_comm COMMAND test_phase3_comm)

add_executable(test_phase3_slog tests/test_phase3_slog.c)
target_link_libraries(test_phase3_slog PRIVATE vayu_sitl_core m)
add_test(NAME phase3_slog COMMAND test_phase3_slog)

# EKF estimator: thin adapter over the shared branch-coverage core in
# src/est/ekf_selftest.c, which we compile here with -DEKF_SELFTEST (the same
# core the firmware runs on target). ekf.c itself comes from vayu_sitl_core.
add_executable(test_phase3_est_ekf
    tests/test_phase3_est_ekf.c
    ${VAYU_ROOT}/src/est/ekf_selftest.c)
target_compile_definitions(test_phase3_est_ekf PRIVATE EKF_SELFTEST)
target_link_libraries(test_phase3_est_ekf PRIVATE vayu_sitl_core m)
add_test(NAME phase3_est_ekf COMMAND test_phase3_est_ekf)

# Standalone: compiles assert.c with NDEBUG (release branch) and stubs its
# deps locally — it must NOT link vayu_sitl_core, which carries the debug
# build of assert.c (duplicate symbol + wrong branch).
# VERT vertical estimator: pure 2-state baro+accel filter core (in
# vayu_sitl_core); the test feeds synthetic accel/baro and checks tracking.
add_executable(test_vertical_est tests/test_vertical_est.c)
target_link_libraries(test_vertical_est PRIVATE vayu_sitl_core m)
add_test(NAME vertical_est COMMAND test_vertical_est)

# flight_phase: pure takeoff/landing detector + FC-owned AGL (in vayu_sitl_core).
add_executable(test_flight_phase tests/test_flight_phase.c)
target_link_libraries(test_flight_phase PRIVATE vayu_sitl_core m)
add_test(NAME flight_phase COMMAND test_flight_phase)

add_executable(test_phase0_assert
    tests/test_phase0_assert.c
    ${VAYU_ROOT}/src/sys/assert.c)
target_compile_definitions(test_phase0_assert PRIVATE NDEBUG)
target_link_libraries(test_phase0_assert PRIVATE Threads::Threads)
add_test(NAME phase0_assert COMMAND test_phase0_assert)

# ----------------------------------------------------------------------------
# Coverage report (CONV-05 / R12.4)
#   cmake -S tools/sim_host -B build_cov -DVAYU_COVERAGE=ON
#   cmake --build build_cov --target coverage
# Prints a gcovr summary over src/ (vendored extern/ excluded).
# ----------------------------------------------------------------------------
add_custom_target(coverage
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    COMMAND gcovr --root ${VAYU_ROOT} --filter "${VAYU_ROOT}/src/"
            --print-summary --decisions
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Running unit tests + gcovr coverage summary")
