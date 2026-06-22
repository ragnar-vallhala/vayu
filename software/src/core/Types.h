#pragma once

#include <QString>
#include <QVector>
#include <cstdint>

// -----------------------------------------------------------
// IMU telemetry – mirrors bmx160_all_converted_reading_t
// -----------------------------------------------------------
struct ImuData {
  float acc[3] = {0, 0, 0}; // m/s²  X Y Z
  float gyr[3] = {0, 0, 0}; // deg/s X Y Z
  float mag[3] = {0, 0, 0}; // µT    X Y Z
  float tempC = 0.0f;
  uint64_t timestamp = 0; // ms since connection open
};

// -----------------------------------------------------------
// Attitude – mirrors attitude_t from sensor_fusion.h
// -----------------------------------------------------------
struct AttitudeData {
  float roll = 0.0f;  // degrees
  float pitch = 0.0f; // degrees
  float yaw = 0.0f;   // degrees
};

// -----------------------------------------------------------
// Barometer – mirrors NavLink BARO (msgid 1039), BME280 source
// -----------------------------------------------------------
struct BaroData {
  float pressurePa = 0.0f;    // Pa
  float temperatureC = 0.0f;  // °C
  float humidityRh = 0.0f;    // %RH
  float altitudeM = 0.0f;     // m (ISA, from sea-level reference)
  uint64_t timestamp = 0;
};

// -----------------------------------------------------------
// Vertical state – mirrors NavLink VERTICAL_STATE (msgid 1040), the fused
// 2-state baro+accel estimate (VERT). altitude/climbRate are the fused outputs;
// baroAltitude is the raw sensor altitude carried alongside so the GCS can chart
// fused-vs-raw. valid=false until the filter is seeded by the first baro fix.
// -----------------------------------------------------------
struct VerticalStateData {
  float altitudeM = 0.0f;        // fused, m (same reference as BaroData.altitudeM)
  float climbRateMs = 0.0f;      // fused, m/s (positive climbing)
  float verticalAccelMs2 = 0.0f; // m/s² (up-positive)
  float baroAltitudeM = 0.0f;    // raw baro altitude, m
  float aglM = 0.0f;             // FC-authoritative height above ground ref, m
  bool  valid = false;           // filter seeded
  uint64_t timestamp = 0;
};

// -----------------------------------------------------------
// RC Channels – raw values (us)
// -----------------------------------------------------------
struct RcData {
  uint16_t channels[14] = {0};
  uint64_t timestamp = 0;
};

// -----------------------------------------------------------
// Connection state
// -----------------------------------------------------------
enum class ConnectionState { Disconnected, Connecting, Connected, Error };

// -----------------------------------------------------------
// Calibration Updates – mirrors SYSTEM_ORIGIN_CALIBRATION
// -----------------------------------------------------------
enum class CalibUpdateType : uint8_t {
  Progress = 0x00,
  NoseUp = 0x01,
  NoseDown = 0x02,
  RightDown = 0x03,
  LeftDown = 0x04,
  Upright = 0x05,
  UpsideDown = 0x06,
  FreeRot = 0x07,
  MagAxisCoverage = 0x08,
  // Terminal status from the FC: the routine ended. Complete = persisted OK;
  // Failed = aborted/fit failure/save failure. The wizard finishes on these
  // explicit events instead of inferring completion from a STANDBY heartbeat
  // (absent when calibration legitimately ends in FAILSAFE).
  Complete = 0x09,
  Failed = 0x0A
};

struct CalibrationUpdate {
  CalibUpdateType type;
  float data;
  float values[3];
};

// -----------------------------------------------------------
// Motor Telemetry
// -----------------------------------------------------------
struct MotorData {
  float speeds[4] = {0, 0, 0, 0};
};
// -----------------------------------------------------------
// Control Loop Data  – mirrors SYSTEM_ORIGIN_CONTROL_DATA (0x05)
// -----------------------------------------------------------
struct ControlLoopData {
  float roll_angle_setpoint = 0.0f;
  float pitch_angle_setpoint = 0.0f;
  float yaw_angle_setpoint = 0.0f;

  float roll_angle_current = 0.0f;
  float pitch_angle_current = 0.0f;
  float yaw_angle_current = 0.0f;

  float roll_rate_setpoint = 0.0f;
  float pitch_rate_setpoint = 0.0f;
  float yaw_rate_setpoint = 0.0f;

  float roll_rate_current = 0.0f;
  float pitch_rate_current = 0.0f;
  float yaw_rate_current = 0.0f;

  float roll_output = 0.0f;
  float pitch_output = 0.0f;
  float yaw_output = 0.0f;
  float throttle_output = 0.0f;

  float outer_dt = 0.0f;
  float inner_dt = 0.0f;

  uint64_t timestamp = 0;
};

// -----------------------------------------------------------
// Estimator cost probe – mirrors SYSTEM_ORIGIN_EST_PERF (0x08)
//   Per-update estimator (EKF/Mahony) cost over the last ~1 s window.
//   peak_us / mean_us are microseconds; decim / rate_hz describe the
//   estimator's effective update cadence.
// -----------------------------------------------------------
struct EstPerfData {
  float peak_us = 0.0f;
  float mean_us = 0.0f;
  float decim = 0.0f;
  float rate_hz = 0.0f;
  uint64_t timestamp = 0;
};

// -----------------------------------------------------------
// Flight Mode  – mirrors SYSTEM_ORIGIN_FLIGHT_MODE (0x07)
//   mode:   0 = stabilise/angle, 1 = acro
//   source: 0 = RC switch, 1 = GCS override
// -----------------------------------------------------------
struct FlightModeStatus {
  uint8_t mode = 0;
  uint8_t source = 0;
};

// -----------------------------------------------------------
// Kernel / observability telemetry – mirrors the firmware
// PACKET_TYPE_PERF_STATS wire format (comm/perf_packet.h). A report is
// reassembled from GLOBAL + TASKS + FIFOS fragments before emission.
// -----------------------------------------------------------
struct PerfTaskRow {
  uint8_t id = 0;
  uint8_t priority = 0;
  uint8_t state = 0; // 0 READY, 1 RUNNING, 2 BLOCKED, 3 DELAYED
  uint16_t stackPeak = 0;
  uint16_t stackSize = 0;
  uint32_t cycles = 0;
  uint32_t switches = 0;
  uint32_t maxBurst = 0;
  // Name is resolved on demand (PACKET_TYPE_PERF_TASKNAME) and cached in the
  // widget, not carried in the perf report.
};

// Reply to a task-name request (PACKET_TYPE_PERF_TASKNAME, FC -> GCS).
struct TaskNameInfo {
  uint8_t id = 0;
  QString name;
};

struct PerfFifoRow {
  uint8_t id = 0; // perf_fifo_id_t
  uint16_t peak = 0;
  uint16_t capacity = 0;
  uint16_t drops = 0;
};

struct PerfReport {
  uint32_t seq = 0;
  bool enabled = false;
  uint32_t uptimeTicks = 0;
  uint32_t schedSwitches = 0;
  uint32_t cpuCyclesLo = 0;
  uint32_t idleCyclesLo = 0;
  uint32_t systickCount = 0;
  uint32_t systickLastCyc = 0;
  uint32_t systickMaxCyc = 0;
  uint32_t systickPreemptions = 0;
  uint32_t ipcTakes = 0;
  uint32_t ipcBlocked = 0;
  uint32_t ipcGives = 0;
  uint32_t ipcTimeouts = 0;
  uint32_t heapAllocs = 0;
  uint32_t heapFrees = 0;
  uint32_t heapOom = 0;
  uint32_t heapPeakBytes = 0;
  uint32_t heapTotalBytes = 0;
  QVector<PerfTaskRow> tasks;
  QVector<PerfFifoRow> fifos;
};
