# Unit tests (QtTest + ctest). Each test compiles the specific core sources
# it exercises directly — the core classes aren't a shared lib yet, so this
# keeps the test target self-contained without duplicating the GUI target.
#
# Tests run headless: ctest sets QT_QPA_PLATFORM=offscreen so the QApplication
# QTEST_MAIN spins up needs no display.

find_package(Qt6 REQUIRED COMPONENTS Test Widgets Core Network)

set(SRC ${CMAKE_SOURCE_DIR}/src)

# navigator_test(<name> <sources...>) — build + register one QtTest binary.
function(navigator_test name)
  qt_add_executable(${name} ${ARGN})
  target_include_directories(${name} PRIVATE
      ${SRC} ${SRC}/core ${SRC}/core/ui ${SRC}/comm ${SRC}/protocol)
  target_link_libraries(${name} PRIVATE
      Qt6::Test Qt6::Widgets Qt6::Core Qt6::Network)
  add_test(NAME ${name} COMMAND ${name})
  set_tests_properties(${name} PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
endfunction()

# ---- Phase 1A: command registry -------------------------------------------
navigator_test(tst_command_registry
    tst_command_registry.cpp
    ${SRC}/core/CommandRegistry.cpp)
