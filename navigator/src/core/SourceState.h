#pragma once

#include <QMetaType>

// The active telemetry source — the single authority over which feed drives the
// GCS (software/docs/roadmap/gcs-source-state-machine.md). Exactly one is active
// at a time (strict single source), plus Idle for "no source". The transition
// graph is fully connected: any state can move to any other, and each move tears
// down the current source before setting up the new one.
//
//   Idle     - app start / disconnected; no feed.
//   Fc       - live serial/UDP link to the flight controller.
//   Sim      - in-app SITL (interactive).
//   Autotune - isolated tuner run (its own SITL; does not feed the GCS engine).
//   Replay   - a recorded .bin played through the parser.
enum class SourceState { Idle, Fc, Sim, Autotune, Replay };

Q_DECLARE_METATYPE(SourceState)
