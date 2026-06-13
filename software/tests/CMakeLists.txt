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

# ---- Phase 1A: command registry -------------------------------------------
navigator_test(tst_command_registry
    tst_command_registry.cpp
    ${SRC}/core/CommandRegistry.cpp)

# ---- Phase 1B: telemetry-source seam --------------------------------------
# ITelemetrySource is a header-only Q_OBJECT; list the headers so AUTOMOC
# generates their meta-objects for this target too.
navigator_test(tst_live_source
    tst_live_source.cpp
    ${SRC}/comm/ITelemetrySource.h
    ${SRC}/comm/LiveSource.h
    ${SRC}/comm/LiveSource.cpp
    ${SRC}/comm/SerialManager.cpp
    ${SRC}/comm/UdpManager.cpp)

# ---- Phase 1C: record format round-trip -----------------------------------
navigator_test(tst_record_sink
    tst_record_sink.cpp
    ${REPLAY}/RecordSink.cpp)
target_include_directories(tst_record_sink PRIVATE ${REPLAY})

# ---- Phase 1D: session mode / read-only authority -------------------------
navigator_test(tst_session_state
    tst_session_state.cpp
    ${SRC}/core/SessionMode.h)

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
    ${SRC}/core/Notify.cpp)
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
