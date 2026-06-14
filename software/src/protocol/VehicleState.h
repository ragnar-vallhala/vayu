#pragma once

#include "../core/Types.h"
#include <QString>
#include <QtGlobal>
#include <cstdint>

// ---------------------------------------------------------------------------
// VehicleState — the canonical UI-facing telemetry snapshot.
//
// Holds the latest of everything the dashboard shows. Owned by TelemetryEngine
// and updated by the parser as packets arrive; the UI never touches it directly
// — it pulls an immutable copy via TelemetryEngine::snapshot() at the render
// rate (see docs/telemetry-engine-architecture.md, Phase A).
//
// Plain value type: trivially/cheaply copyable so snapshot() is an O(1) copy
// under a short mutex hold.
// ---------------------------------------------------------------------------
struct VehicleState {
  AttitudeData attitude{};
  ImuData      imu{};
  RcData       rc{};
  MotorData    motors{};

  uint8_t flightMode = 0;        // 0 = stabilise/angle, 1 = acro
  uint8_t flightModeSource = 0;  // 0 = RC switch, 1 = GCS override

  QString vehicleState;          // last *recognised* state name ("" = none yet)
  bool    armed = false;         // derived: ARMED | IN_AIR | FAILSAFE

  // Last-rx wall-clock (ms since epoch) per feed; 0 = never received. The UI
  // shows "-" once these go stale, distinguishing absent telemetry from a real
  // zero.
  qint64 lastImuMs = 0;
  qint64 lastAttMs = 0;
  qint64 lastRcMs = 0;
  qint64 lastMotorMs = 0;

  // Rolling attitude std-devs (deg), computed by the engine on each attitude
  // packet from a fixed-window RollingStats — the per-packet accumulation must
  // not be down-sampled to the render rate.
  float rollStd = 0.0f;
  float pitchStd = 0.0f;
  float yawStd = 0.0f;

  // Monotonic wire-packet counter: one tick per valid + unknown packet. Drives
  // the status-bar "Rate: N Hz" readout (sampled once a second by the UI).
  quint64 packetCount = 0;
};
