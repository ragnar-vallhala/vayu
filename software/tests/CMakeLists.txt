# Unit tests (QtTest + ctest). Each test compiles the specific core sources
# it exercises directly — the core classes aren't a shared lib yet, so this
# keeps the test target self-contained without duplicating the GUI target.
#
# Tests run headless: ctest sets QT_QPA_PLATFORM=offscreen so the QApplication
# QTEST_MAIN spins up needs no display.

find_package(Qt6 REQUIRED COMPONENTS Test Widgets Core Network)

set(SRC ${CMAKE_SOURCE_DIR}/src)
set(REPLAY ${SRC}/replay)

# navigator_test(<name> <sources...>) — build + register one QtTest binary.
function(navigator_test name)
  qt_add_executable(${name} ${ARGN})
  target_include_directories(${name} PRIVATE
      ${SRC} ${SRC}/core ${SRC}/core/ui ${SRC}/comm ${SRC}/protocol)
  target_link_libraries(${name} PRIVATE
      Qt6::Test Qt6::Widgets Qt6::Core Qt6::Network Qt6::SerialPort)
  add_test(NAME ${name} COMMAND ${name})
  set_tests_properties(${name} PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
endfunction()

# ---- Time-sync estimator (NTP offset/delay, jitter filter, skew) ----------
navigator_test(tst_time_sync_estimator
    tst_time_sync_estimator.cpp
    ${SRC}/protocol/TimeSyncEstimator.cpp)
target_include_directories(tst_time_sync_estimator PRIVATE ${SRC}/protocol)

# ---- CMD_SET_PID command codec (AT-1 foundation) --------------------------
navigator_test(tst_command_codec
    tst_command_codec.cpp
    ${SRC}/protocol/CommandCodec.cpp
    ${SRC}/core/crc.cpp)
target_include_directories(tst_command_codec PRIVATE ${SRC}/protocol ${SRC}/core)
# CommandCodec emits NavLink v2 (CMD_SET_PID); link the generated codec (its
# PUBLIC SYSTEM include carries navlink_msgs.h).
target_link_libraries(tst_command_codec PRIVATE navlink_codec)

# ---- AT-1: autotune gains -> CMD_SET_PID mapping --------------------------
navigator_test(tst_autotune_gains
    tst_autotune_gains.cpp)
target_include_directories(tst_autotune_gains PRIVATE ${SRC}/ui/widgets)

# ---- C++ autotune optimizer port ------------------------------------------
navigator_test(tst_optimizer
    tst_optimizer.cpp
    ${SRC}/autotune/Optimizer.cpp)
target_include_directories(tst_optimizer PRIVATE ${SRC}/autotune)

# ---- C++ autotune cost functions ------------------------------------------
navigator_test(tst_cost
    tst_cost.cpp
    ${SRC}/autotune/Cost.cpp)
target_include_directories(tst_cost PRIVATE ${SRC}/autotune)

# ---- C++ autotune engine (Space + orchestration, synthetic rollout) -------
navigator_test(tst_autotune_engine
    tst_autotune_engine.cpp
    ${SRC}/autotune/AutotuneEngine.cpp
    ${SRC}/autotune/Space.cpp
    ${SRC}/autotune/Optimizer.cpp
    ${SRC}/autotune/Cost.cpp)
target_include_directories(tst_autotune_engine PRIVATE ${SRC}/autotune)

# ---- LIVE SITL stack bring-up (spawns the real sim; skips without binaries) -
navigator_test(tst_sitl_stack
    tst_sitl_stack.cpp
    ${SRC}/autotune/SitlStack.cpp
    ${SRC}/autotune/Rollout.cpp
    ${SRC}/autotune/Cost.cpp
    ${SRC}/autotune/Space.cpp
    ${SRC}/protocol/DroneProtocol.cpp
    ${SRC}/protocol/PacketDecoder.cpp
    ${SRC}/protocol/PacketDissector.cpp
    ${SRC}/protocol/CommandCodec.cpp
    ${SRC}/protocol/NavlinkRouter.cpp  # DroneProtocol owns a NavlinkRouter (v2 RX)
    ${SRC}/core/crc.cpp)
target_include_directories(tst_sitl_stack PRIVATE
    ${SRC}/autotune ${SRC}/protocol ${SRC}/core
    ${CMAKE_SOURCE_DIR}/../tools/vsim/include)
# DroneProtocol + CommandCodec pull in the NavLink v2 codec (RX router / TX).
target_link_libraries(tst_sitl_stack PRIVATE navlink_codec)

# ---- Phase 1A: command registry -------------------------------------------
navigator_test(tst_command_registry
    tst_command_registry.cpp
    ${SRC}/core/CommandRegistry.cpp)

# ---- Phase 1C: record format round-trip -----------------------------------
navigator_test(tst_record_sink
    tst_record_sink.cpp
    ${REPLAY}/RecordSink.cpp)
target_include_directories(tst_record_sink PRIVATE ${REPLAY})

# ---- Phase C: telemetry-source state machine (full transition matrix) ------
navigator_test(tst_source_controller
    tst_source_controller.cpp
    ${SRC}/core/SourceController.cpp)
target_include_directories(tst_source_controller PRIVATE ${SRC}/core)

# ---- Phase 2A: shortcuts manager (overrides + conflicts + persistence) -----
navigator_test(tst_shortcuts_manager
    tst_shortcuts_manager.cpp
    ${SRC}/core/ShortcutsManager.cpp
    ${SRC}/core/CommandRegistry.cpp)

# ---- Phase 2A: shortcuts editor dialog (GUI smoke) ------------------------
navigator_test(tst_shortcuts_editor
    tst_shortcuts_editor.cpp
    ${SRC}/ui/widgets/ShortcutsEditorDialog.cpp
    ${SRC}/core/ShortcutsManager.cpp
    ${SRC}/core/CommandRegistry.cpp
    ${SRC}/core/Notify.cpp
    ${SRC}/core/ToastOverlay.cpp)
target_include_directories(tst_shortcuts_editor PRIVATE ${SRC}/ui/widgets)

# ---- Phase 2B: command palette (fuzzy matcher + popup smoke) ---------------
navigator_test(tst_command_palette
    tst_command_palette.cpp
    ${SRC}/ui/widgets/CommandPalette.cpp
    ${SRC}/core/CommandRegistry.cpp)
target_include_directories(tst_command_palette PRIVATE ${SRC}/ui/widgets)

# ---- Phase 2C: view history (MRU logic) -----------------------------------
navigator_test(tst_view_history
    tst_view_history.cpp
    ${SRC}/core/ViewHistory.cpp)

# ---- Calibration wizard step machine --------------------------------------
navigator_test(tst_calibration_wizard
    tst_calibration_wizard.cpp
    ${SRC}/core/CalibrationWizard.cpp)
target_include_directories(tst_calibration_wizard PRIVATE ${SRC}/core)

# ---- Phase 2D: replay source (playback clock, crop, loop, seek) ------------
navigator_test(tst_replay_source
    tst_replay_source.cpp
    ${SRC}/comm/ITelemetrySource.h  # header-only Q_OBJECT base — needs AUTOMOC
    ${REPLAY}/ReplaySource.cpp
    ${REPLAY}/RecordSink.cpp)
target_include_directories(tst_replay_source PRIVATE ${REPLAY})

# ---- Phase 2E: replay bar (transport GUI smoke) ---------------------------
navigator_test(tst_replay_bar
    tst_replay_bar.cpp
    ${SRC}/comm/ITelemetrySource.h
    ${SRC}/ui/widgets/ReplayBar.cpp
    ${REPLAY}/ReplaySource.cpp
    ${REPLAY}/RecordSink.cpp)
target_include_directories(tst_replay_bar PRIVATE ${REPLAY} ${SRC}/ui/widgets)

# ---- Phase 3A/3B: about dialog + docs path --------------------------------
navigator_test(tst_about_dialog
    tst_about_dialog.cpp
    ${SRC}/ui/widgets/AboutDialog.cpp)
target_include_directories(tst_about_dialog PRIVATE ${SRC}/ui/widgets)
