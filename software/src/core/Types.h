#pragma once

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
  MagAxisCoverage = 0x08
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
  float roll_angle_error = 0.0f;
  float pitch_angle_error = 0.0f;
  float yaw_angle_error = 0.0f;
  float roll_rate_error = 0.0f;
  float pitch_rate_error = 0.0f;
  float yaw_rate_error = 0.0f;
  float dt = 0.0f;
  float roll_angle_setpoint = 0.0f;
  float pitch_angle_setpoint = 0.0f;
  float yaw_angle_setpoint = 0.0f;
  float roll_rate_setpoint = 0.0f;
  float pitch_rate_setpoint = 0.0f;
  float yaw_rate_setpoint = 0.0f;
  float roll_output = 0.0f;
  float pitch_output = 0.0f;
  float yaw_output = 0.0f;
  float throttle_output = 0.0f;
  uint64_t timestamp = 0;
};
