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
