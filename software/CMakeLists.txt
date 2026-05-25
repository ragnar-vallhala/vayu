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
    OpenGL
    OpenGLWidgets
)

# -------------------------------------------------------
# Sources
# -------------------------------------------------------
set(GCS_SOURCES
    src/app/main.cpp
    src/ui/main/MainWindow.cpp
    src/comm/SerialManager.cpp
    src/protocol/DroneProtocol.cpp
    src/ui/widgets/AttitudeWidget.cpp
    src/ui/widgets/ImuPanel.cpp
    src/ui/widgets/LogPanel.cpp
    src/core/crc.cpp
    src/ui/widgets/PacketAnalyzerWidget.cpp
    src/ui/main/SettingsWidget.cpp
    src/protocol/PacketDecoder.cpp
    src/ui/widgets/PacketDetailWidget.cpp
    src/ui/widgets/RealTimeGraph.cpp
    src/core/SettingsManager.cpp
    src/ui/widgets/FrequencyRibbon.cpp
    src/ui/widgets/RcChannelsWidget.cpp
    src/ui/widgets/CalibrationWidget.cpp
    src/ui/widgets/MotorStatusWidget.cpp
    src/ui/widgets/Drone3DWidget.cpp
    src/ui/widgets/ControlLoopPlot.cpp
    src/ui/widgets/SimulatorWidget.cpp
    src/vsim/PhysicsCore.cpp
    src/vsim/MotorModel.cpp
    src/vsim/SensorModels.cpp
    src/vsim/SimController.cpp
    src/vsim/SimWorker.cpp
    src/vsim/SimRendererWidget.cpp
)

set(GCS_HEADERS
    src/ui/main/MainWindow.h
    src/comm/SerialManager.h
    src/protocol/DroneProtocol.h
    src/ui/widgets/AttitudeWidget.h
    src/ui/widgets/ImuPanel.h
    src/ui/widgets/LogPanel.h
    src/core/Types.h
    src/core/crc.h
    src/core/RollingStats.h
    src/ui/widgets/PacketAnalyzerWidget.h
    src/ui/main/SettingsWidget.h
    src/protocol/PacketDecoder.h
    src/ui/widgets/PacketDetailWidget.h
    src/ui/widgets/RealTimeGraph.h
    src/core/SettingsManager.h
    src/ui/widgets/FrequencyRibbon.h
    src/ui/widgets/RcChannelsWidget.h
    src/ui/widgets/CalibrationWidget.h
    src/ui/widgets/MotorStatusWidget.h
    src/ui/widgets/Drone3DWidget.h
    src/ui/widgets/ControlLoopPlot.h
    src/ui/widgets/SimulatorWidget.h
    src/vsim/VsimTypes.h
    src/vsim/PhysicsCore.h
    src/vsim/MotorModel.h
    src/vsim/SensorModels.h
    src/vsim/SimController.h
    src/vsim/SimWorker.h
    src/vsim/SimRendererWidget.h
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
    src/comm 
    src/protocol 
    src/ui/main
    src/ui/widgets
    src/vsim
)

target_link_libraries(Navigator PRIVATE
    Qt6::Core
    Qt6::Widgets
    Qt6::SerialPort
    Qt6::OpenGL
    Qt6::OpenGLWidgets
)

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
    target_link_libraries(Navigator PRIVATE vayu_sitl_core)
    target_compile_definitions(Navigator PRIVATE NAVIGATOR_HAS_SITL=1)
    message(STATUS "Navigator: linking against vayu_sitl_core")
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
