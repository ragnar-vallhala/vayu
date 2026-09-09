#include "NavlinkRouter.h"

#include "core/MathUtils.h" // float16_to_float32 (IMU delta reconstruction)
#include <QDateTime>        // t4 capture for TIME_SYNC
#include <QtGlobal>

extern "C" {
#include "navlink_msgs.h" // the generated codec — included ONLY here (RX side)
}

namespace {
constexpr float kRad2Deg = 57.29577951308232f;

// One thunk per wired leaf: cast ctx back to the router and call its hook,
// converting the v2 message into the GCS's own struct first.
void thunkAttitude(void *ctx, const navlink_frame_hdr_t *,
                   const navlink_attitude_euler_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (r->onAttitude) {
    AttitudeData a; // v2 is radians; AttitudeData is degrees (core/Types.h)
    a.roll = m->roll * kRad2Deg;
    a.pitch = m->pitch * kRad2Deg;
    a.yaw = m->yaw * kRad2Deg;
    r->onAttitude(a);
  }
}

void thunkImuRaw(void *ctx, const navlink_frame_hdr_t *,
                 const navlink_imu_raw_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  ImuData d;
  for (int i = 0; i < 3; i++) {
    d.acc[i] = m->acc[i];
    d.gyr[i] = m->gyr[i];
    d.mag[i] = m->mag[i];
  }
  d.tempC = m->temp;
  d.timestamp = m->sample_time_us;
  r->lastImu = d; // anchor for subsequent IMU_COMPRESSED deltas
  r->hasLastImu = true;
  if (r->onImu)
    r->onImu(d);
}

void thunkImuCompressed(void *ctx, const navlink_frame_hdr_t *,
                        const navlink_imu_compressed_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->hasLastImu)
    return; // no anchor yet — drop until the next IMU_RAW (as v1 did)
  ImuData d = r->lastImu;
  for (int i = 0; i < 3; i++) {
    d.acc[i] += MathUtils::float16_to_float32(m->delta[i]);
    d.gyr[i] += MathUtils::float16_to_float32(m->delta[i + 3]);
    d.mag[i] += MathUtils::float16_to_float32(m->delta[i + 6]);
  }
  d.tempC += MathUtils::float16_to_float32(m->delta[9]);
  r->lastImu = d; // deltas chain off the reconstructed sample
  if (r->onImu)
    r->onImu(d);
}

void thunkRc(void *ctx, const navlink_frame_hdr_t *,
             const navlink_rc_channels_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->onRc)
    return;
  RcData d; // RcData holds 14 channels; v2 carries 18 — take the first 14
  for (int i = 0; i < 14; i++)
    d.channels[i] = m->chan[i];
  r->onRc(d);
}

void thunkMotor(void *ctx, const navlink_frame_hdr_t *,
                const navlink_motor_telemetry_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->onMotor)
    return;
  MotorData d; // MotorData holds 4 motors; v2 carries 8 — take the first 4
  for (int i = 0; i < 4; i++)
    d.speeds[i] = m->cmd[i];
  r->onMotor(d);
}

void thunkControlTrace(void *ctx, const navlink_frame_hdr_t *,
                       const navlink_control_trace_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->onControlLoop)
    return;
  ControlLoopData d;
  d.roll_angle_setpoint = m->roll_angle_sp;
  d.pitch_angle_setpoint = m->pitch_angle_sp;
  d.yaw_angle_setpoint = m->yaw_angle_sp;
  d.roll_angle_current = m->roll_angle_curr;
  d.pitch_angle_current = m->pitch_angle_curr;
  d.yaw_angle_current = m->yaw_angle_curr;
  d.roll_rate_setpoint = m->roll_rate_sp;
  d.pitch_rate_setpoint = m->pitch_rate_sp;
  d.yaw_rate_setpoint = m->yaw_rate_sp;
  d.roll_rate_current = m->roll_rate_curr;
  d.pitch_rate_current = m->pitch_rate_curr;
  d.yaw_rate_current = m->yaw_rate_curr;
  d.roll_output = m->roll_out;
  d.pitch_output = m->pitch_out;
  d.yaw_output = m->yaw_out;
  d.throttle_output = m->thro_out;
  d.outer_dt = m->outer_dt;
  d.inner_dt = m->inner_dt;
  r->onControlLoop(d);
}

void thunkEstPerf(void *ctx, const navlink_frame_hdr_t *,
                  const navlink_est_perf_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->onEstPerf)
    return;
  EstPerfData d;
  d.peak_us = m->peak_us;
  d.mean_us = m->mean_us;
  d.decim = m->decimation;
  d.rate_hz = m->rate_hz;
  r->onEstPerf(d);
}

void thunkBaro(void *ctx, const navlink_frame_hdr_t *,
               const navlink_baro_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->onBaro)
    return;
  BaroData d;
  d.pressurePa = m->pressure;
  d.temperatureC = m->temperature;
  d.humidityRh = m->humidity;
  d.altitudeM = m->altitude;
  r->onBaro(d);
}

void thunkNotchStatus(void *ctx, const navlink_frame_hdr_t *,
                      const navlink_notch_status_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->onNotchStatus)
    return;
  NotchStatusData d;
  d.enabled = m->enabled != 0;
  d.centerHz[0][0] = m->roll_hz0;
  d.centerHz[0][1] = m->roll_hz1;
  d.centerHz[0][2] = m->roll_hz2;
  d.centerHz[1][0] = m->pitch_hz0;
  d.centerHz[1][1] = m->pitch_hz1;
  d.centerHz[1][2] = m->pitch_hz2;
  d.centerHz[2][0] = m->yaw_hz0;
  d.centerHz[2][1] = m->yaw_hz1;
  d.centerHz[2][2] = m->yaw_hz2;
  r->onNotchStatus(d);
}

void thunkHslStatus(void *ctx, const navlink_frame_hdr_t *,
                    const navlink_hsl_status_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->onHslStatus)
    return;
  HslStatusData d;
  d.recording = m->recording != 0;
  d.session = m->session;
  d.headSlot = m->head_slot;
  d.ringSectors = m->ring_sectors;
  d.wraps = m->wraps;
  d.droppedSectors = m->dropped_sectors;
  d.seq = m->seq;
  r->onHslStatus(d);
}

void thunkVerticalState(void *ctx, const navlink_frame_hdr_t *,
                        const navlink_vertical_state_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->onVerticalState)
    return;
  VerticalStateData d;
  d.altitudeM = m->altitude;
  d.climbRateMs = m->climb_rate;
  d.verticalAccelMs2 = m->vertical_accel;
  d.baroAltitudeM = m->baro_altitude;
  d.aglM = m->agl;
  d.accelBiasMs2 = m->accel_bias;
  d.accelUnhealthy = m->accel_unhealthy != 0;
  d.valid = m->valid != 0;
  r->onVerticalState(d);
}

void thunkFlightMode(void *ctx, const navlink_frame_hdr_t *,
                     const navlink_flight_mode_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (r->onFlightMode)
    r->onFlightMode(m->mode, m->source);
}

void thunkHeartbeat(void *ctx, const navlink_frame_hdr_t *hdr,
                    const navlink_heartbeat_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (r->onHeartbeat)
    r->onHeartbeat(m->nav_state, m->timestamp, hdr->sysid);
}

void thunkSystemHealth(void *ctx, const navlink_frame_hdr_t *,
                       const navlink_system_health_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (r->onSystemHealth)
    r->onSystemHealth(m->tx_overflow, m->imu_drop, m->log_wrap, m->cpu_load);
}

void thunkStatustext(void *ctx, const navlink_frame_hdr_t *,
                     const navlink_statustext_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->onLog)
    return;
  size_t n = 0;
  while (n < sizeof(m->text) && m->text[n] != '\0')
    n++;
  r->onLog(QString::fromLatin1(m->text, static_cast<int>(n)));
}

void thunkCalibration(void *ctx, const navlink_frame_hdr_t *,
                      const navlink_calibration_status_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->onCalibration)
    return;
  CalibrationUpdate u;
  u.type = static_cast<CalibUpdateType>(m->step);
  u.data = static_cast<float>(m->progress);
  u.values[0] = m->coverage[0];
  u.values[1] = m->coverage[1];
  u.values[2] = m->coverage[2];
  r->onCalibration(u);
}

void thunkPerfGlobal(void *ctx, const navlink_frame_hdr_t *,
                     const navlink_perf_global_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  PerfReport &a = r->perfAccum;
  a = PerfReport{}; // fresh report
  a.seq = m->seq;
  a.enabled = (m->flags & 0x01) != 0;
  a.uptimeTicks = m->uptime_ticks;
  a.schedSwitches = m->sched_switches;
  a.cpuCyclesLo = m->cpu_cycles_lo;
  a.idleCyclesLo = m->idle_cycles_lo;
  a.systickCount = m->systick_count;
  a.systickLastCyc = m->systick_last_cyc;
  a.systickMaxCyc = m->systick_max_cyc;
  a.systickPreemptions = m->systick_preemptions;
  a.ipcTakes = m->ipc_takes;
  a.ipcBlocked = m->ipc_takes_blocked;
  a.ipcGives = m->ipc_gives;
  a.ipcTimeouts = m->ipc_timeouts;
  a.heapAllocs = m->heap_allocs;
  a.heapFrees = m->heap_frees;
  a.heapOom = m->heap_oom;
  a.heapPeakBytes = m->heap_peak_bytes;
  a.heapTotalBytes = m->heap_total_bytes;
  r->perfPendingTasks = m->total_tasks;
  r->perfPendingFifos = m->total_fifos;
  r->perfHaveGlobal = true;
}

// Emit the report once both row sets for the open GLOBAL have arrived.
void perfMaybeComplete(NavlinkRouter *r) {
  if (!r->perfHaveGlobal)
    return;
  if (r->perfAccum.tasks.size() >= r->perfPendingTasks &&
      r->perfAccum.fifos.size() >= r->perfPendingFifos) {
    r->perfHaveGlobal = false;
    if (r->onPerf)
      r->onPerf(r->perfAccum);
  }
}

void thunkPerfTask(void *ctx, const navlink_frame_hdr_t *,
                   const navlink_perf_task_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->perfHaveGlobal || m->seq != r->perfAccum.seq)
    return; // stray row without its GLOBAL
  PerfTaskRow t;
  t.id = m->task_id;
  t.priority = m->priority;
  t.state = m->state;
  t.stackPeak = m->stack_peak;
  t.stackSize = m->stack_size;
  t.cycles = m->cycles_lo;
  t.switches = m->switches_in;
  t.maxBurst = m->max_burst_cyc;
  r->perfAccum.tasks.push_back(t);
  perfMaybeComplete(r);
}

void thunkPerfFifo(void *ctx, const navlink_frame_hdr_t *,
                   const navlink_perf_fifo_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->perfHaveGlobal || m->seq != r->perfAccum.seq)
    return;
  PerfFifoRow f;
  f.id = m->fifo_id;
  f.peak = m->peak;
  f.capacity = m->capacity;
  f.drops = m->drops;
  r->perfAccum.fifos.push_back(f);
  perfMaybeComplete(r);
}

void thunkPerfTaskname(void *ctx, const navlink_frame_hdr_t *,
                       const navlink_perf_taskname_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->onTaskName)
    return;
  size_t n = 0;
  while (n < sizeof(m->name) && m->name[n] != '\0')
    n++;
  r->onTaskName(m->task_id, QString::fromLatin1(m->name, static_cast<int>(n)));
}

void thunkTimeSync(void *ctx, const navlink_frame_hdr_t *,
                   const navlink_time_sync_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (m->role != 1 /* RESPONSE */ || !r->onTimeSync)
    return;
  const quint64 t4 = static_cast<quint64>(QDateTime::currentMSecsSinceEpoch());
  r->onTimeSync(m->seq, m->t1_gcs_tx, m->t2_fc_rx, m->t3_fc_tx, t4);
}

void thunkCommandAck(void *ctx, const navlink_frame_hdr_t *,
                     const navlink_command_ack_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (r->onCommandAck)
    r->onCommandAck(m->command, m->req_seq, m->result);
}

void thunkDefault(void *ctx, const navlink_frame_hdr_t *, uint32_t msgid,
                  const uint8_t *, size_t len) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (r->onDefault)
    r->onDefault(msgid, static_cast<int>(len));
}
} // namespace

struct NavlinkRouter::Impl {
  navlink_parser_t parser;
  navlink_handlers_t handlers;
};

NavlinkRouter::NavlinkRouter() : d_(new Impl) {
  navlink_parser_init(&d_->parser);
  d_->handlers = {};       // zero every slot
  d_->handlers.ctx = this; // shared ctx: the router itself
  d_->handlers.on_default = thunkDefault;
  d_->handlers.on_attitude_euler = thunkAttitude;
  d_->handlers.on_imu_raw = thunkImuRaw;
  d_->handlers.on_imu_compressed = thunkImuCompressed;
  d_->handlers.on_rc_channels = thunkRc;
  d_->handlers.on_motor_telemetry = thunkMotor;
  d_->handlers.on_control_trace = thunkControlTrace;
  d_->handlers.on_est_perf = thunkEstPerf;
  d_->handlers.on_baro = thunkBaro;
  d_->handlers.on_vertical_state = thunkVerticalState;
  d_->handlers.on_notch_status = thunkNotchStatus;
  d_->handlers.on_hsl_status = thunkHslStatus;
  d_->handlers.on_flight_mode = thunkFlightMode;
  d_->handlers.on_heartbeat = thunkHeartbeat;
  d_->handlers.on_system_health = thunkSystemHealth;
  d_->handlers.on_statustext = thunkStatustext;
  d_->handlers.on_calibration_status = thunkCalibration;
  d_->handlers.on_perf_global = thunkPerfGlobal;
  d_->handlers.on_perf_task = thunkPerfTask;
  d_->handlers.on_perf_fifo = thunkPerfFifo;
  d_->handlers.on_perf_taskname = thunkPerfTaskname;
  d_->handlers.on_time_sync = thunkTimeSync;
  d_->handlers.on_command_ack = thunkCommandAck;
  // Sensible default so an unhandled leaf is never silent, even if the owner
  // didn't override onDefault.
  onDefault = [](uint32_t msgid, int len) {
    qInfo("[navlink] unhandled v2 msgid %u (%d B)", msgid, len);
  };
}

NavlinkRouter::~NavlinkRouter() { delete d_; }

void NavlinkRouter::feed(const QByteArray &f) {
  navlink_parser_push(&d_->parser, &d_->handlers,
                      reinterpret_cast<const uint8_t *>(f.constData()),
                      static_cast<size_t>(f.size()));
}
