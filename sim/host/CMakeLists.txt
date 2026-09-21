cmake_minimum_required(VERSION 3.20)
project(vayu_sitl C CXX)   # CXX for the in-process vsim physics (Phase 4 RTOS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Native host SITL build for vayu. Reuses the firmware controller sources
# (src/control, src/maths, src/utils, src/actuator/motor.c) and links them
# against host shims for the vaios kernel + NavHAL.
#
# Configure:  cmake -S sim/host -B build_sitl
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
                  # the ARM-specific bits via sim/host/include/port.h
    NAVHAL=1
    VAYU_FFT_NOTCH=1   # compile the REAL FFT dynamic gyro notch (default-ON on
                       # the FC), so SITL runs the same code as hardware — not
                       # the inert stub. Runtime-disabled until the NOTCH command
                       # enables it + props pass the throttle gate.
    VAIOS_CONFIG_FILE="${VAYU_ROOT}/firmware/include/vaios_app_config.h"
)

# Shadow + firmware + vaios + NavHAL + SDK + vsim headers, in priority order.
#
# sdk/ and include/ are NOT interchangeable. include/ holds NavHAL port shims
# that shadow system headers (atomic.h, family) and is private to this project.
# sdk/ holds the two headers a runtime loader compiles against -- keep it free
# of anything that would follow it out of the tree.
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk        # vsim_proto.h + vayu_sitl_abi.h
    ${VAYU_ROOT}/firmware/include
    ${VAIOS_ROOT}/include
    ${NAVHAL_ROOT}/include
    ${NAVHAL_ROOT}/include/port/cortex-m4   # utility type-only headers
    ${VAYU_ROOT}/sim/vsim/include         # physics headers (trimesh_bvh.h etc.)
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
# Firmware sources the host binary reuses verbatim.
set(VAYU_SOURCES
    ${VAYU_ROOT}/firmware/src/calib/calib_ellipsoid.c # shared sensor-agnostic ellipsoid fit
    ${VAYU_ROOT}/firmware/src/calib/calib_engine.c    # calibration engine (acquire + fit + commit)
    ${VAYU_ROOT}/firmware/src/actuator/esc.c
    ${VAYU_ROOT}/firmware/src/actuator/motor.c
    ${VAYU_ROOT}/firmware/src/control/angle_controller.c
    ${VAYU_ROOT}/firmware/src/control/height_controller.c
    ${VAYU_ROOT}/firmware/src/control/throttle_curve.c
    ${VAYU_ROOT}/firmware/src/control/angle_rate_controller.c
    ${VAYU_ROOT}/firmware/src/control/mixer.c        # control allocation (mix + airmode desaturation)
    ${VAYU_ROOT}/firmware/src/control/sysid.c        # rate-loop system-ID excite/capture (angle_rate_controller + navlink refs)
    ${VAYU_ROOT}/firmware/src/control/pid_config.c   # CMD_SET_PID apply + SD persistence
    ${VAYU_ROOT}/firmware/src/control/flight_mode.c  # stabilise/acro arbitration (RC + GCS)
    ${VAYU_ROOT}/firmware/src/control/control_buffer.c
    ${VAYU_ROOT}/firmware/src/est/lpf.c
    ${VAYU_ROOT}/firmware/src/est/hover_estimate.c
    ${VAYU_ROOT}/firmware/src/storage/hover_store.c
    ${VAYU_ROOT}/firmware/src/maths/maths_interface.c
    ${VAYU_ROOT}/firmware/src/maths/fft.c              # m_rfft_* (dynamic-notch FFT analysis)
    ${VAYU_ROOT}/firmware/src/maths/biquad.c           # m_biquad_* (notch bank design/apply)
    ${VAYU_ROOT}/firmware/src/dsp/notch_fft.c          # FFT peak-tracking front-end
    ${VAYU_ROOT}/firmware/src/dsp/notch_bank.c         # per-axis biquad notch bank
    ${VAYU_ROOT}/firmware/src/dsp/gyro_notch.c         # FFT-driven dynamic gyro notch (angle_rate_controller dep)
    ${VAYU_ROOT}/firmware/src/control/pid.c
    ${VAYU_ROOT}/firmware/src/control/rate_indi.c   # optional INDI inner loop (RATE_CTRL_ALGO_USED)
    ${VAYU_ROOT}/firmware/src/sys/math_utils.c
    ${VAYU_ROOT}/firmware/src/sys/sys_utils.c
    ${VAYU_ROOT}/firmware/src/est/sensor_fusion.c
    ${VAYU_ROOT}/firmware/src/est/ekf.c
    ${VAYU_ROOT}/firmware/src/est/attitude_task.c   # the REAL firmware estimator task (now run in SITL)
    ${VAYU_ROOT}/firmware/src/est/vertical_estimator.c  # 2-state baro+accel vertical estimator (VERT)
    ${VAYU_ROOT}/firmware/src/est/vertical_task.c       # VERT task I/O wrapper (sibling of attitude_task)
    ${VAYU_ROOT}/firmware/src/est/flight_phase.c        # takeoff/landing detector + FC-owned AGL (IN_AIR)
    ${VAYU_ROOT}/firmware/src/sensor/imu_buffer.c
    ${VAYU_ROOT}/firmware/src/sensor/bme280.c   # real FC baro driver (altitude/publish/getters); host_baro.c feeds it
    ${VAYU_ROOT}/firmware/src/sensor/vl53l0x.c  # real FC ToF driver; no host feeder, so its boot probe fails on the i2c stubs and it reports absent (tof_valid=0 -> the height loop falls back to baro AGL)
    ${VAYU_ROOT}/firmware/src/storage/fs_owner.c # centralised FS owner (absorbs logger.c)
    ${VAYU_ROOT}/firmware/src/storage/imu_hs_log.c # high-speed IMU->SD stream (HSL), drained by the FS task
    ${VAYU_ROOT}/firmware/src/logger/log_text.c  # vayu_log + queue (moved from utils.c)
    ${VAYU_ROOT}/firmware/src/sys/state.c
    ${VAYU_ROOT}/firmware/src/sys/assert.c
    ${VAYU_ROOT}/firmware/src/comm/rc_buffer.c
    ${VAYU_ROOT}/firmware/src/comm/rc_safety.c   # RC watchdog + arm gate (host-buildable)
    # Telemetry chain: produces DroneProtocol packets the GCS reads.
    # UART2 in host_navhal pipes them to a pty (for live GCS attach) and
    # tees a raw copy to /tmp/vayu_uart2.log (for post-sim analysis).
    ${VAYU_ROOT}/firmware/src/comm/channel.c
    ${VAYU_ROOT}/firmware/src/comm/perf_packet.c  # perf_fifo_fill_row (buffers link it)
    ${VAYU_ROOT}/firmware/src/comm/serializer.c
    ${VAYU_ROOT}/firmware/src/comm/comm_processor.c
    ${VAYU_ROOT}/firmware/src/comm/navlink_router.c  # v2 RX handler table (uplink)
    ${VAYU_ROOT}/firmware/src/comm/navlink_tx.c      # TX seam (telemetry/cmd-resp framing)
    ${VAYU_ROOT}/firmware/src/comm/telemetry_task.c
    ${VAYU_ROOT}/firmware/src/comm/xfer/navlink_xfer.c     # bulk-transfer (FTP) substrate core SM
    ${VAYU_ROOT}/firmware/src/comm/xfer/navlink_xfer_tx.c  # xfer codec/channel seam (g_xfer_tx_ops)
    ${VAYU_ROOT}/firmware/src/comm/xfer/fs_query.c         # filesystem navigation (list/stat)
    ${VAYU_ROOT}/firmware/src/comm/xfer/providers/file_provider.c
    ${VAYU_ROOT}/firmware/src/comm/xfer/providers/log_provider.c
    ${VAYU_ROOT}/firmware/src/comm/xfer/providers/stream_provider.c
)

# vaios kernel pieces that are pure C and portable.
set(VAIOS_HOST_SOURCES
    ${VAIOS_ROOT}/kernel/structure.c
)

# ----------------------------------------------------------------------------
# Targets
#
# libvayu_sitl_core.a (static): firmware sources + host shims, compiled for the
# host. It exists as the link target for the firmware UNIT-TEST suite below
# (each test provides its own main() and pulls the firmware objects it needs).
# The interactive SITL now runs on vayu_sitl_rtos (the in-process engine); the
# legacy standalone vayu_sitl binary + its vsim_d FIFO peer were removed.
# ----------------------------------------------------------------------------

add_library(vayu_sitl_core STATIC
    ${HOST_SOURCES}
    ${VAYU_SOURCES}
    ${VAIOS_HOST_SOURCES}
)
target_include_directories(vayu_sitl_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VAYU_ROOT}/firmware/include
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
  # Also linked into the loadable module, so its objects must be relocatable.
  set_target_properties(navlink_codec PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

find_package(Threads REQUIRED)
target_link_libraries(vayu_sitl_core PUBLIC Threads::Threads m rt navlink_codec)

message(STATUS "vayu_sitl_core: firmware host test library configured")

# ----------------------------------------------------------------------------
# Phase 4 (experimental, opt-in): vayu_sitl_rtos
#
# Runs the REAL vaios scheduler on the host via the ucontext port
# (host_rtos_port.c) instead of the legacy pthread shims — deterministic and
# maximally faithful (the flight scheduler itself is under test). The host_vaios
# collision set (task/delay/sema/mutex/heap/scheduler/v_init) is compiled out by
# -DVAYU_SITL_RTOS and provided by the kernel; -DVAIOS_HOST_TEST drops the
# kernel's target-SRAM pointer guards. See firmware/docs/plans/sitl-lockstep-sim.md.
#
#   cmake -S sim/host -B build_sitl_rtos -DVAYU_SITL_RTOS_BUILD=ON
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
      ${VAYU_ROOT}/sim/vsim/src/sim_controller.cpp
      ${VAYU_ROOT}/sim/vsim/src/physics_core.cpp
      ${VAYU_ROOT}/sim/vsim/src/motor_model.cpp
      ${VAYU_ROOT}/sim/vsim/src/sensor_models.cpp
  )
  set_target_properties(vsim_phys PROPERTIES
      INCLUDE_DIRECTORIES "${VAYU_ROOT}/sim/vsim/include;${CMAKE_CURRENT_SOURCE_DIR}/sdk"
      POSITION_INDEPENDENT_CODE ON)   # linked into the loadable module
  target_include_directories(vsim_phys INTERFACE
      ${VAYU_ROOT}/sim/vsim/include ${CMAKE_CURRENT_SOURCE_DIR}/sdk)

  # The RTOS SITL as a CORE static library (#11e): everything EXCEPT
  # host_rtos_main.c (the CLI scenarios + main()). Navigator links this and
  # drives it from a worker thread via the host_rtos_engine API (boot/step/
  # set_rc/get_pose + the vsim_inproc config surface), so the GCS runs the SAME
  # deterministic in-process engine the headless binary does — no vsim_d, no
  # FIFO.
  #
  # The firmware build defs (VAYU_SITL_RTOS etc.) and the firmware/vaios/navhal
  # include dirs are PRIVATE: they wire the core's OWN .c files but must NOT leak
  # into a Qt consumer (Navigator) where they could shadow system/Qt headers or
  # flip firmware ifdefs. Consumers see only the slim, type-free surface — the
  # src/ dir (host_rtos_engine_api.h) and vsim_proto.h (via vsim_phys) — PUBLIC.
  add_library(vayu_sitl_rtos_core STATIC
      src/host_rtos_port.c
      src/host_rtos_engine.c   # reusable in-process step engine (#11a)
      src/host_vaios.c        # collisions compiled out by VAYU_SITL_RTOS
      src/host_navhal.c
      src/host_vfs.c
      src/host_baro.c         # provides the i2c_manager_* stubs bme280.c needs
      src/host_imu_feeder.c   # single-shot pump for the stepper (thread unused)
      src/host_rc_feeder.c    # serial RC feeder (GCS opt-in via rtos_engine_enable_serial_rc)
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
  target_compile_definitions(vayu_sitl_rtos_core PRIVATE
      VAYU_SITL_RTOS=1 VAIOS_HOST_TEST=1
      HEAP_SIZE=0x180000)   # 1.5 MB: room for the firmware tasks' 8KB stacks on host
  # The same objects are archived into the module below, so build them PIC.
  set_target_properties(vayu_sitl_rtos_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(vayu_sitl_rtos_core
      PRIVATE
        ${VAYU_ROOT}/firmware/include
        ${VAIOS_ROOT}/include
        ${NAVHAL_ROOT}/include
        ${NAVHAL_ROOT}/include/port/cortex-m4
      PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src   # host_rtos_engine_api.h (the GCS surface)
        ${CMAKE_CURRENT_SOURCE_DIR}/sdk   # vsim_proto.h + vayu_sitl_abi.h
        ${VAYU_ROOT}/sim/vsim/include     # trimesh_bvh.h
  )
  # navlink_codec/vsim_phys are linked PUBLIC (the engine API needs vsim types);
  # the thread/m/rt system libs the consumer also needs at link time.
  target_link_libraries(vayu_sitl_rtos_core PUBLIC Threads::Threads m rt navlink_codec vsim_phys)

  # Standalone scenario binary = the core + the CLI main (hold/doublet/disturb/
  # pose/cfg). main() lives ONLY here, never in the library Navigator links.
  # host_rtos_main.c is firmware-side, so it needs the PRIVATE defs/includes the
  # core uses (they don't propagate from a PRIVATE usage requirement).
  add_executable(vayu_sitl_rtos src/host_rtos_main.c)
  target_compile_definitions(vayu_sitl_rtos PRIVATE
      VAYU_SITL_RTOS=1 VAIOS_HOST_TEST=1 HEAP_SIZE=0x180000)
  target_include_directories(vayu_sitl_rtos PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${VAYU_ROOT}/firmware/include
      ${VAIOS_ROOT}/include
      ${NAVHAL_ROOT}/include
      ${NAVHAL_ROOT}/include/port/cortex-m4
      ${VAYU_ROOT}/sim/vsim/include)
  target_link_libraries(vayu_sitl_rtos PRIVATE vayu_sitl_rtos_core)

  # The SAME engine as a LOADABLE MODULE (libvayu_sitl.so / vayu_sitl.dll).
  # A host with no build-time knowledge of the firmware dlopens this and drives
  # it through the vtable in sdk/vayu_sitl_abi.h -- that is how Navigator gets
  # an in-process simulator without compiling one line of firmware, and what
  # lets the firmware ship on its own cadence.
  #
  # MODULE, not SHARED: it is never linked against, only loaded.
  #
  # Only vayu_sitl_get_api is exported. Hiding the rest keeps the firmware's
  # ~hundreds of globals (and its malloc/assert shims) out of the loader's
  # symbol namespace, where they would otherwise be free to collide.
  # Build identity, so a host can say WHICH firmware it loaded. The ABI version
  # only promises the struct layout matches; it says nothing about which engine
  # this is, and a module ships separately from whatever loads it.
  #
  # Resolved at configure time, not build time: it is one git call, and a stamp
  # that silently stopped updating would be worse than no stamp at all. Re-run
  # cmake to refresh it.
  # --exclude: the rolling sitl-sdk-latest release tag moves with main, so an
  # unfiltered describe stamps the module "sitl-sdk-latest-1-gNNNNNNN" -- it
  # names the release rather than the commit, and its meaning changes every
  # time the tag moves. The whole point of this string is to identify WHICH
  # firmware a host loaded, so the rolling tag must not be a candidate.
  execute_process(
      COMMAND git -C "${VAYU_ROOT}" describe --always --dirty --tags
              --exclude "sitl-sdk-*"
      OUTPUT_VARIABLE VAYU_SITL_GIT_ID
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
  if(NOT VAYU_SITL_GIT_ID)
    set(VAYU_SITL_GIT_ID "unknown")   # tarball export, or no git on PATH
  endif()
  string(TIMESTAMP VAYU_SITL_BUILD_DATE "%Y-%m-%d" UTC)
  set(VAYU_SITL_BUILD_ID "${VAYU_SITL_GIT_ID} ${VAYU_SITL_BUILD_DATE}")
  message(STATUS "vayu_sitl build id: ${VAYU_SITL_BUILD_ID}")

  add_library(vayu_sitl MODULE src/vayu_sitl_module.c)
  target_compile_definitions(vayu_sitl PRIVATE
      VAYU_SITL_BUILDING_MODULE=1
      VAYU_SITL_BUILD_ID="${VAYU_SITL_BUILD_ID}")
  target_link_libraries(vayu_sitl PRIVATE vayu_sitl_rtos_core)
  set_target_properties(vayu_sitl PROPERTIES
      C_VISIBILITY_PRESET hidden
      CXX_VISIBILITY_PRESET hidden
      VISIBILITY_INLINES_HIDDEN ON
      PREFIX "lib")   # libvayu_sitl.so on every platform, so loaders need no ifdef
  if(NOT APPLE AND NOT WIN32)
    # Belt and braces: the visibility presets cover our own TUs, but the static
    # archives we swallow were compiled without them.
    target_link_options(vayu_sitl PRIVATE
        "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/vayu_sitl.map")
    set_target_properties(vayu_sitl PROPERTIES
        LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/vayu_sitl.map)
  endif()

  # The shippable form of the engine. Both front doors onto the SAME engine
  # ship, because consumers use both: the Qt app loads lib/libvayu_sitl.so in
  # process, while the headless SDK and the autotuner spawn bin/vayu_sitl_rtos
  # and talk to it over its FIFOs. Plus the headers a host compiles against,
  # and nothing else -- a host needs no other part of this tree.
  #
  # A target rather than CI-only shell, so the tarball a developer can inspect
  # is byte-for-byte the one CI uploads -- a packaging step that only ever runs
  # in CI is a packaging step nobody can debug.
  set(VAYU_SITL_SDK_STAGE "${CMAKE_BINARY_DIR}/vayu-sitl-sdk")
  add_custom_target(vayu_sitl_package
      COMMAND ${CMAKE_COMMAND} -E rm -rf "${VAYU_SITL_SDK_STAGE}"
      COMMAND ${CMAKE_COMMAND} -E make_directory "${VAYU_SITL_SDK_STAGE}/lib"
                                                 "${VAYU_SITL_SDK_STAGE}/bin"
                                                 "${VAYU_SITL_SDK_STAGE}/include"
      COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:vayu_sitl>
                                       "${VAYU_SITL_SDK_STAGE}/lib/"
      COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:vayu_sitl_rtos>
                                       "${VAYU_SITL_SDK_STAGE}/bin/"
      COMMAND ${CMAKE_COMMAND} -E copy
                  "${CMAKE_CURRENT_SOURCE_DIR}/sdk/vayu_sitl_abi.h"
                  "${CMAKE_CURRENT_SOURCE_DIR}/sdk/vsim_proto.h"
                  "${CMAKE_CURRENT_SOURCE_DIR}/sdk/trimesh_bvh.h"
                  "${CMAKE_CURRENT_SOURCE_DIR}/sdk/vsim_math.h"
                  "${VAYU_SITL_SDK_STAGE}/include/"
      COMMAND ${CMAKE_COMMAND} -E echo "${VAYU_SITL_BUILD_ID}"
              > "${VAYU_SITL_SDK_STAGE}/BUILD_ID"
      COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_BINARY_DIR}"
              ${CMAKE_COMMAND} -E tar czf "vayu-sitl-sdk.tar.gz" "vayu-sitl-sdk"
      DEPENDS vayu_sitl vayu_sitl_rtos
      COMMENT "Packaging vayu-sitl-sdk.tar.gz (${VAYU_SITL_BUILD_ID})"
      VERBATIM)

  # GCS-integration test (#12): drives the SAME slim engine API SimWorker uses,
  # from a non-firmware C++ TU (only host_rtos_engine_api.h + vsim_iface.h) — no
  # Qt, no display. Proves boot+telemetry+pose+config link and behave.
  add_executable(test_gcs_engine_api test/test_gcs_engine_api.cpp)
  target_link_libraries(test_gcs_engine_api PRIVATE vayu_sitl_rtos_core)

  # Module-load test: links NO firmware, only libdl + the SDK headers, so it
  # fails loudly if the module ever stops being self-contained. Its include
  # path is the SDK dir plus the one header it needs to build a telemetry
  # iface -- deliberately NOT the firmware include env.
  add_executable(test_sitl_module test/test_sitl_module.c)
  set_target_properties(test_sitl_module PROPERTIES
      INCLUDE_DIRECTORIES
        "${CMAKE_CURRENT_SOURCE_DIR}/sdk;${CMAKE_CURRENT_SOURCE_DIR}/src")
  # navlink_codec is the wire protocol, not the firmware -- a real GCS links
  # it too, which is exactly the posture this test is checking.
  target_link_libraries(test_sitl_module PRIVATE ${CMAKE_DL_LIBS} navlink_codec)
  add_dependencies(test_sitl_module vayu_sitl)
  add_test(NAME sitl_module
           COMMAND test_sitl_module $<TARGET_FILE:vayu_sitl>)
  message(STATUS "vayu_sitl_rtos_core (lib) + vayu_sitl_rtos (bin) + test_gcs_engine_api configured")
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

# Centralised FS owner: snapshot/torn-read, reserved save lane, drop accounting,
# log round-trip (src/storage/fs_owner.c, in vayu_sitl_core).
# HSL wire format: the real encoder's bytes, then decoded by hslog.py below —
# the encoder has no runtime consumer that would ever notice a layout mistake.
add_executable(test_hslog tests/test_hslog.c)
target_link_libraries(test_hslog PRIVATE vayu_sitl_core m)
add_test(NAME hslog COMMAND test_hslog)
add_test(NAME hslog_decode
         COMMAND ${CMAKE_COMMAND} -E env python3
                 ${VAYU_ROOT}/tools/telemetry/hslog.py
                 --selftest-file /tmp/vayu_hslog_test/0_imuhs.bin)
set_tests_properties(hslog_decode PROPERTIES DEPENDS hslog)

add_executable(test_rc_buffer tests/test_rc_buffer.c)
target_link_libraries(test_rc_buffer PRIVATE vayu_sitl_core m)
add_test(NAME rc_buffer COMMAND test_rc_buffer)

add_executable(test_imu_buffer tests/test_imu_buffer.c)
target_link_libraries(test_imu_buffer PRIVATE vayu_sitl_core m)
add_test(NAME imu_buffer COMMAND test_imu_buffer)

add_executable(test_fs_owner tests/test_fs_owner.c)
target_link_libraries(test_fs_owner PRIVATE vayu_sitl_core m)
add_test(NAME fs_owner COMMAND test_fs_owner)

# xfer (FTP) substrate core SM (src/comm/xfer/navlink_xfer.c, in vayu_sitl_core).
add_executable(test_xfer_sm tests/test_xfer_sm.c)
target_link_libraries(test_xfer_sm PRIVATE vayu_sitl_core m)
add_test(NAME xfer_sm COMMAND test_xfer_sm)

# xfer end-to-end through the real codec seam (navlink_xfer_tx.c + navlink_codec).
add_executable(test_xfer_e2e tests/test_xfer_e2e.c)
target_link_libraries(test_xfer_e2e PRIVATE vayu_sitl_core navlink_codec m)
add_test(NAME xfer_e2e COMMAND test_xfer_e2e)

# xfer real providers: FILE round-trip, LOG download/deny, STREAM source.
add_executable(test_xfer_providers tests/test_xfer_providers.c)
target_link_libraries(test_xfer_providers PRIVATE vayu_sitl_core m)
add_test(NAME xfer_providers COMMAND test_xfer_providers)

# filesystem navigation (FS_LIST / FS_INFO) service over fs_owner + host VFS.
add_executable(test_fs_query tests/test_fs_query.c)
target_link_libraries(test_fs_query PRIVATE vayu_sitl_core m)
add_test(NAME fs_query COMMAND test_fs_query)

# shared ellipsoid fit (src/calib/calib_ellipsoid.c, in vayu_sitl_core):
# synthetic offset+scale+misalignment recovery from random orientations.
add_executable(test_calib_ellipsoid tests/test_calib_ellipsoid.c)
target_link_libraries(test_calib_ellipsoid PRIVATE vayu_sitl_core m)
add_test(NAME calib_ellipsoid COMMAND test_calib_ellipsoid)

# calibration engine (src/calib/calib_engine.c, in vayu_sitl_core): fake-provider
# acquire+fit+commit, plus cancel / too-few-samples rejection paths.
add_executable(test_calib_engine tests/test_calib_engine.c)
target_link_libraries(test_calib_engine PRIVATE vayu_sitl_core m)
add_test(NAME calib_engine COMMAND test_calib_engine)

# EKF estimator: thin adapter over the shared branch-coverage core in
# src/est/ekf_selftest.c, which we compile here with -DEKF_SELFTEST (the same
# core the firmware runs on target). ekf.c itself comes from vayu_sitl_core.
add_executable(test_phase3_est_ekf
    tests/test_phase3_est_ekf.c
    ${VAYU_ROOT}/firmware/src/est/ekf_selftest.c)
target_compile_definitions(test_phase3_est_ekf PRIVATE EKF_SELFTEST)
target_link_libraries(test_phase3_est_ekf PRIVATE vayu_sitl_core m)
add_test(NAME phase3_est_ekf COMMAND test_phase3_est_ekf)

# Standalone: compiles assert.c with NDEBUG (release branch) and stubs its
# deps locally — it must NOT link vayu_sitl_core, which carries the debug
# build of assert.c (duplicate symbol + wrong branch).
# VERT vertical estimator: pure 2-state baro+accel filter core (in
# vayu_sitl_core); the test feeds synthetic accel/baro and checks tracking.
# first-order LPF (src/est/lpf.c): pure math, no RTOS — recurrence, alpha
# end-cases and step response.
# RX seam (src/comm/navlink_router.c): dispatch + the 10.5 command gate,
# driven through the sim-only comm_rx_raw_inject() seam.
add_executable(test_navlink_router tests/test_navlink_router.c)
target_link_libraries(test_navlink_router PRIVATE vayu_sitl_core navlink_codec m)
add_test(NAME navlink_router COMMAND test_navlink_router)

# TX seam (src/comm/navlink_tx.c): framing + field mapping, captured through
# the in-process vsim_iface UART2 hook. NOT the generated codec — that is
# navlink/generated/c/navlink_msgs.c.
add_executable(test_navlink_tx tests/test_navlink_tx.c)
target_link_libraries(test_navlink_tx PRIVATE vayu_sitl_core m)
add_test(NAME navlink_tx COMMAND test_navlink_tx)

# accel/mag -> Euler helper (src/est/sensor_fusion.c): pure, no RTOS.
add_executable(test_sensor_fusion tests/test_sensor_fusion.c)
target_link_libraries(test_sensor_fusion PRIVATE vayu_sitl_core m)
add_test(NAME sensor_fusion COMMAND test_sensor_fusion)

add_executable(test_lpf tests/test_lpf.c)
target_link_libraries(test_lpf PRIVATE vayu_sitl_core m)
add_test(NAME lpf COMMAND test_lpf)

add_executable(test_vertical_est tests/test_vertical_est.c)
target_link_libraries(test_vertical_est PRIVATE vayu_sitl_core m)
add_test(NAME vertical_est COMMAND test_vertical_est)

# flight_phase: pure takeoff/landing detector + FC-owned AGL (in vayu_sitl_core).
add_executable(test_flight_phase tests/test_flight_phase.c)
target_link_libraries(test_flight_phase PRIVATE vayu_sitl_core m)
add_test(NAME flight_phase COMMAND test_flight_phase)

add_executable(test_phase0_assert
    tests/test_phase0_assert.c
    ${VAYU_ROOT}/firmware/src/sys/assert.c)
target_compile_definitions(test_phase0_assert PRIVATE NDEBUG)
target_link_libraries(test_phase0_assert PRIVATE Threads::Threads)
add_test(NAME phase0_assert COMMAND test_phase0_assert)

# ----------------------------------------------------------------------------
# Coverage report (CONV-05 / R12.4)
#   cmake -S sim/host -B build_cov -DVAYU_COVERAGE=ON
#   cmake --build build_cov --target coverage
# Prints a gcovr summary over src/ (vendored extern/ excluded).
# ----------------------------------------------------------------------------
# The firmware host UNIT tests (firmware/tests/host: pid/mixer/maths/fft) are a
# separate CTest project; build+run them instrumented and merge their coverage
# with this sim/host (integration) run so the per-file/-component numbers reflect
# both. gcovr accepts multiple search paths and unions the line hits.
set(VAYU_FWTEST_COV_STEPS
    COMMAND ${CMAKE_COMMAND} -S ${VAYU_ROOT}/firmware/tests/host -B fwtest_cov
            -DCMAKE_C_FLAGS=--coverage -DCMAKE_EXE_LINKER_FLAGS=--coverage
    COMMAND ${CMAKE_COMMAND} --build fwtest_cov
    COMMAND ${CMAKE_CTEST_COMMAND} --test-dir fwtest_cov --output-on-failure)

add_custom_target(coverage
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    ${VAYU_FWTEST_COV_STEPS}
    COMMAND gcovr ${CMAKE_CURRENT_BINARY_DIR} fwtest_cov --root ${VAYU_ROOT}
            --filter "${VAYU_ROOT}/firmware/src/" --print-summary --decisions
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Running unit tests (sim/host + firmware host) + gcovr summary")

# coverage-gate: same run, but emits a gcovr json-summary and enforces the
# per-component floor ratchet (tools/coverage_gate.py) — fails on a regression.
# This is the CI-enforcing target; `coverage` stays the human-readable report.
#   cmake --build build_cov --target coverage-gate
add_custom_target(coverage-gate
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    ${VAYU_FWTEST_COV_STEPS}
    COMMAND gcovr ${CMAKE_CURRENT_BINARY_DIR} fwtest_cov --root ${VAYU_ROOT}
            --filter "${VAYU_ROOT}/firmware/src/" --json-summary -o cov.json
    COMMAND python3 ${VAYU_ROOT}/tools/coverage_gate.py firmware cov.json
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Unit tests (sim/host + firmware host) + per-component coverage ratchet gate")
