cmake_minimum_required(VERSION 3.20)
project(Navigator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_COMPILE_COMMANDS_OUTPUT "${CMAKE_SOURCE_DIR}/compile_commands.json")

# Qt6 required modules
find_package(Qt6 REQUIRED COMPONENTS
    Core
    Widgets
    SerialPort
    Network
    OpenGL
    OpenGLWidgets
)

# -------------------------------------------------------
# Sources
# -------------------------------------------------------
# Always-built sources. SITL sources (vsim/* + SimulatorWidget) are
# conditionally added below when sim_host is available, so Navigator
# degrades gracefully (FR-0d / FR-SIM-10).
set(GCS_SOURCES
    src/app/main.cpp
    src/ui/main/MainStatusBar.cpp
    src/ui/main/MainToolbar.cpp
    src/ui/main/MainWindow.cpp
    src/comm/SerialManager.cpp
    src/comm/UdpManager.cpp
    src/comm/PortArbiter.cpp
    src/comm/LiveSource.cpp
    src/protocol/DroneProtocol.cpp
    src/protocol/CommandCodec.cpp
    src/ui/widgets/AttitudeWidget.cpp
    src/ui/widgets/HorizonHud.cpp
    src/ui/widgets/ImuPanel.cpp
    src/ui/widgets/LogPanel.cpp
    src/core/crc.cpp
    src/core/CommandRegistry.cpp
    src/core/ShortcutsManager.cpp
    src/core/ViewHistory.cpp
    src/core/CsvExport.cpp
    src/core/Logger.cpp
    src/core/Notify.cpp
    src/core/Theme.cpp
    src/ui/widgets/PacketAnalyzerWidget.cpp
    src/ui/main/SettingsWidget.cpp
    src/protocol/PacketDecoder.cpp
    src/protocol/PacketDissector.cpp
    src/ui/widgets/PacketDetailWidget.cpp
    src/ui/widgets/HexView.cpp
    src/ui/widgets/PacketLogModel.cpp
    src/ui/widgets/PacketFilterExpr.cpp
    src/ui/widgets/PacketFilterProxy.cpp
    src/ui/widgets/RealTimeGraph.cpp
    src/core/SettingsManager.cpp
    src/replay/RecordSink.cpp
    src/replay/ReplaySource.cpp
    src/autotune/Optimizer.cpp
    src/autotune/Cost.cpp
    src/autotune/Space.cpp
    src/autotune/AutotuneEngine.cpp
    src/ui/widgets/FrequencyRibbon.cpp
    src/ui/widgets/LinkStatsPanel.cpp
    src/ui/widgets/RcChannelsWidget.cpp
    src/ui/widgets/CalibrationWidget.cpp
    src/ui/widgets/MotorStatusWidget.cpp
    src/ui/widgets/Drone3DWidget.cpp
    src/ui/widgets/ControlLoopPlot.cpp
    src/ui/widgets/PerfWidget.cpp
    src/ui/widgets/ShortcutsEditorDialog.cpp
    src/ui/widgets/CommandPalette.cpp
    src/ui/widgets/RecentViewsOverlay.cpp
    src/ui/widgets/ReplayBar.cpp
    src/ui/widgets/AboutDialog.cpp
)

set(GCS_HEADERS
    src/ui/main/MainWindow.h
    src/comm/SerialManager.h
    src/comm/UdpManager.h
    src/comm/ITelemetrySource.h
    src/comm/LiveSource.h
    src/protocol/DroneProtocol.h
    src/protocol/CommandCodec.h
    src/ui/widgets/AttitudeWidget.h
    src/ui/widgets/ImuPanel.h
    src/ui/widgets/LogPanel.h
    src/core/Types.h
    src/core/crc.h
    src/core/Command.h
    src/core/CommandRegistry.h
    src/core/ShortcutsManager.h
    src/core/ViewHistory.h
    src/core/SessionMode.h
    src/core/RollingStats.h
    src/core/CsvExport.h
    src/core/Logger.h
    src/core/Notify.h
    src/core/Theme.h
    src/core/ui/Buttons.h
    src/core/ui/WheelGuard.h
    src/ui/widgets/PacketAnalyzerWidget.h
    src/ui/main/SettingsWidget.h
    src/protocol/PacketDecoder.h
    src/protocol/PacketDissector.h
    src/ui/widgets/PacketDetailWidget.h
    src/ui/widgets/HexView.h
    src/ui/widgets/PacketLogModel.h
    src/ui/widgets/PacketFilterExpr.h
    src/ui/widgets/PacketFilterProxy.h
    src/ui/widgets/RealTimeGraph.h
    src/core/SettingsManager.h
    src/replay/RecordFormat.h
    src/replay/RecordSink.h
    src/replay/ReplaySource.h
    src/autotune/Optimizer.h
    src/autotune/Cost.h
    src/autotune/Space.h
    src/autotune/AutotuneEngine.h
    src/ui/widgets/FrequencyRibbon.h
    src/ui/widgets/LinkStatsPanel.h
    src/ui/widgets/RcChannelsWidget.h
    src/ui/widgets/CalibrationWidget.h
    src/ui/widgets/MotorStatusWidget.h
    src/ui/widgets/Drone3DWidget.h
    src/ui/widgets/ControlLoopPlot.h
    src/ui/widgets/PerfWidget.h
    src/ui/widgets/ShortcutsEditorDialog.h
    src/ui/widgets/CommandPalette.h
    src/ui/widgets/RecentViewsOverlay.h
    src/ui/widgets/ReplayBar.h
    src/ui/widgets/AboutDialog.h
)

# SITL-only sources. These pull in vsim_iface.h from tools/sim_host and
# can't compile without it. Added to the target only when sim_host is
# present (see the add_subdirectory check below).
#
# After the vsim_d split, Navigator no longer contains the physics
# simulator -- PhysicsCore/MotorModel/SensorModels/SimController moved
# to tools/vsim/. SimWorker became a thin process supervisor that
# spawns vsim_d and decodes pose frames off /tmp/vsim_pose.
set(GCS_SITL_SOURCES
    src/ui/widgets/SimulatorWidget.cpp
    src/ui/widgets/GeometryEditorWidget.cpp
    src/ui/widgets/WorldEditorWidget.cpp
    src/ui/widgets/CollapsibleSection.cpp
    src/ui/widgets/SimHudWidget.cpp
    src/vsim/SimWorker.cpp
    src/vsim/SimRendererWidget.cpp
    src/vsim/MeshLoader.cpp
    src/vsim/WorldMeshBuilder.cpp
    src/vsim/MassProperties.cpp
    src/vsim/RcBridge.cpp
    src/audio/PropAudio.cpp
)
set(GCS_SITL_HEADERS
    src/ui/widgets/SimulatorWidget.h
    src/ui/widgets/GeometryEditorWidget.h
    src/ui/widgets/WorldEditorWidget.h
    src/ui/widgets/CollapsibleSection.h
    src/ui/widgets/SimHudWidget.h
    src/vsim/VsimTypes.h
    src/vsim/SimWorker.h
    src/vsim/SimRendererWidget.h
    src/vsim/MeshLoader.h
    src/vsim/WorldMeshBuilder.h
    src/vsim/MassProperties.h
    src/vsim/RcBridge.h
)

set(GCS_RESOURCES
    resources/resources.qrc
)

# -------------------------------------------------------
# Executable
# -------------------------------------------------------
qt_add_executable(Navigator
    ${GCS_SOURCES}
    ${GCS_HEADERS}
    ${GCS_RESOURCES}
)

target_include_directories(Navigator PRIVATE
    src
    src/core
    src/core/ui
    src/comm
    src/protocol
    src/replay
    src/autotune
    src/ui/main
    src/ui/widgets
    src/vsim
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools/vsim/include  # vsim_proto.h
)

target_link_libraries(Navigator PRIVATE
    Qt6::Core
    Qt6::Widgets
    Qt6::SerialPort
    Qt6::Network
    Qt6::OpenGL
    Qt6::OpenGLWidgets
)

# Surfaced in the About dialog (FR-UX-22).
target_compile_definitions(Navigator PRIVATE
    NAVIGATOR_VERSION="${PROJECT_VERSION}")

# Optional propeller audio via PulseAudio (PipeWire-pulse works too). When the
# simple API isn't present we compile a silent stub, so this never blocks the
# build.
find_library(PULSE_LIB pulse)
find_library(PULSE_SIMPLE_LIB pulse-simple)
if(PULSE_LIB AND PULSE_SIMPLE_LIB)
    target_compile_definitions(Navigator PRIVATE HAVE_PULSE_SIMPLE)
    target_link_libraries(Navigator PRIVATE ${PULSE_LIB} ${PULSE_SIMPLE_LIB})
    message(STATUS "Navigator: prop audio enabled (PulseAudio)")
else()
    message(STATUS "Navigator: prop audio = silent stub (libpulse-simple not found)")
endif()

# -------------------------------------------------------
# vayu_sitl_core: the firmware-side SITL built as a static library.
# Navigator links it in directly so the simulator + firmware live in
# one process (the legacy /tmp/vayu_pwm.fifo + /tmp/vayu_imu.fifo
# round-trip is gone for in-app simulation).
# -------------------------------------------------------
set(VAYU_SITL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../tools/sim_host")
if(EXISTS "${VAYU_SITL_DIR}/CMakeLists.txt")
    add_subdirectory("${VAYU_SITL_DIR}"
                     "${CMAKE_BINARY_DIR}/vayu_sitl"
                     EXCLUDE_FROM_ALL)
    # assimp imports the airframe mesh for the geometry/MoI editor; it's
    # only needed on the SITL path, so it's found inside this gate.
    find_package(assimp REQUIRED)
    target_sources(Navigator PRIVATE ${GCS_SITL_SOURCES} ${GCS_SITL_HEADERS})
    target_link_libraries(Navigator PRIVATE vayu_sitl_core assimp::assimp)
    target_compile_definitions(Navigator PRIVATE NAVIGATOR_HAS_SITL=1)
    message(STATUS "Navigator: linking against vayu_sitl_core + assimp (SITL ON)")
else()
    message(WARNING "Navigator: ${VAYU_SITL_DIR} not found; SITL features disabled")
endif()

# Platform-specific settings
if(APPLE)
    set_target_properties(Navigator PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_GUI_IDENTIFIER com.vayu.gcs
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    )
endif()

if(WIN32)
    set_target_properties(Navigator PROPERTIES WIN32_EXECUTABLE TRUE)
endif()

message(STATUS "Qt6 Navigator build configured")
message(STATUS "Qt6 version: ${Qt6_VERSION}")

# -------------------------------------------------------
# Unit tests (QtTest + ctest). On by default; -DNAVIGATOR_BUILD_TESTS=OFF
# to skip. Run with `ctest` from the build dir.
# -------------------------------------------------------
option(NAVIGATOR_BUILD_TESTS "Build Navigator unit tests" ON)
if(NAVIGATOR_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
    message(STATUS "Navigator: unit tests ON (ctest)")
endif()
