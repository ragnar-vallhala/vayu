#pragma once

#include "core/Types.h"  // AttitudeData, ...
#include <QByteArray>
#include <functional>

// The single home for the GCS's NavLink v2 *receive* handler table. This is the
// only translation unit that includes the generated codec for decoding, so the
// rest of the GCS never sees a navlink_* type — decoded leaves arrive as plain
// GCS structs through the std::function hooks below. Assign a hook from whatever
// module owns that data; any decoded leaf without a hook falls to onDefault
// (which just logs). Adding a message = one thunk + one hook + one ctor line in
// NavlinkRouter.cpp. See navlink/INTEGRATION.md.
class NavlinkRouter {
public:
  NavlinkRouter();
  ~NavlinkRouter();
  NavlinkRouter(const NavlinkRouter &) = delete;
  NavlinkRouter &operator=(const NavlinkRouter &) = delete;

  // Push one whole v2 frame (sync..CRC) through the parser; hooks fire inline.
  void feed(const QByteArray &v2Frame);

  // Per-leaf hooks — set by the owning GCS module (default: unset -> onDefault).
  std::function<void(const AttitudeData &)> onAttitude;
  std::function<void(const ImuData &)> onImu;
  std::function<void(const RcData &)> onRc;
  std::function<void(const MotorData &)> onMotor;
  std::function<void(const ControlLoopData &)> onControlLoop;
  std::function<void(const EstPerfData &)> onEstPerf;
  std::function<void(const BaroData &)> onBaro;
  std::function<void(const VerticalStateData &)> onVerticalState;
  std::function<void(const NotchStatusData &)> onNotchStatus;
  std::function<void(uint8_t mode, uint8_t source)> onFlightMode;
  // HEARTBEAT: nav_state is the flight-state enum index; timestamp + device id
  // (frame sysid) drive link liveness. The owner maps nav_state -> state name.
  std::function<void(uint8_t navState, uint64_t timestamp, uint8_t deviceId)>
      onHeartbeat;
  std::function<void(const QString &line)> onLog;          // STATUSTEXT
  std::function<void(const CalibrationUpdate &)> onCalibration;
  std::function<void(const PerfReport &)> onPerf;          // reassembled report
  std::function<void(int taskId, const QString &name)> onTaskName;
  // TIME_SYNC RESPONSE: the four NTP stamps (t4 captured here, on receipt).
  std::function<void(uint8_t seq, uint64_t t1, uint64_t t2, uint64_t t3,
                     uint64_t t4)>
      onTimeSync;
  // COMMAND_ACK: the FC's response to a command, correlated by (command,reqSeq).
  std::function<void(uint32_t command, uint8_t reqSeq, uint8_t result)>
      onCommandAck;
  // SYSTEM_HEALTH: decoded but has no GCS consumer today (parity with v1, which
  // never surfaced it). A hook is provided for when one is added.
  std::function<void(uint32_t txOverflow, uint32_t imuDrop, uint32_t logWrap,
                     uint8_t cpuLoad)>
      onSystemHealth;
  // Fallback for every decoded leaf without a specific hook above.
  std::function<void(uint32_t msgid, int payloadLen)> onDefault;

  // IMU delta state: IMU_COMPRESSED carries half-float deltas vs the last
  // IMU_RAW, so the router reconstructs the absolute sample (as v1's
  // PacketDecoder did). Public so the file-scope thunks can reach it.
  ImuData lastImu;
  bool hasLastImu = false;

  // PERF reassembly: PERF_GLOBAL opens a report (carrying the expected task/fifo
  // counts), then PERF_TASK/PERF_FIFO rows (sharing its seq) fill it; onPerf
  // fires once both row sets have arrived. Public for the file-scope thunks.
  PerfReport perfAccum;
  bool perfHaveGlobal = false;
  int perfPendingTasks = 0;
  int perfPendingFifos = 0;

private:
  struct Impl;
  Impl *d_;
};
