#include "NavlinkRouter.h"

#include "core/MathUtils.h"  // float16_to_float32 (IMU delta reconstruction)
#include <QtGlobal>

extern "C" {
#include "navlink_msgs.h"  // the generated codec — included ONLY here (RX side)
}

namespace {
constexpr float kRad2Deg = 57.29577951308232f;

// One thunk per wired leaf: cast ctx back to the router and call its hook,
// converting the v2 message into the GCS's own struct first.
void thunkAttitude(void *ctx, const navlink_frame_hdr_t *,
                   const navlink_attitude_euler_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (r->onAttitude) {
    AttitudeData a;  // v2 is radians; AttitudeData is degrees (core/Types.h)
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
  r->lastImu = d;  // anchor for subsequent IMU_COMPRESSED deltas
  r->hasLastImu = true;
  if (r->onImu)
    r->onImu(d);
}

void thunkImuCompressed(void *ctx, const navlink_frame_hdr_t *,
                        const navlink_imu_compressed_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->hasLastImu)
    return;  // no anchor yet — drop until the next IMU_RAW (as v1 did)
  ImuData d = r->lastImu;
  for (int i = 0; i < 3; i++) {
    d.acc[i] += MathUtils::float16_to_float32(m->delta[i]);
    d.gyr[i] += MathUtils::float16_to_float32(m->delta[i + 3]);
    d.mag[i] += MathUtils::float16_to_float32(m->delta[i + 6]);
  }
  d.tempC += MathUtils::float16_to_float32(m->delta[9]);
  r->lastImu = d;  // deltas chain off the reconstructed sample
  if (r->onImu)
    r->onImu(d);
}

void thunkRc(void *ctx, const navlink_frame_hdr_t *,
             const navlink_rc_channels_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->onRc)
    return;
  RcData d;  // RcData holds 14 channels; v2 carries 18 — take the first 14
  for (int i = 0; i < 14; i++)
    d.channels[i] = m->chan[i];
  r->onRc(d);
}

void thunkMotor(void *ctx, const navlink_frame_hdr_t *,
                const navlink_motor_telemetry_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (!r->onMotor)
    return;
  MotorData d;  // MotorData holds 4 motors; v2 carries 8 — take the first 4
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

void thunkFlightMode(void *ctx, const navlink_frame_hdr_t *,
                     const navlink_flight_mode_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (r->onFlightMode)
    r->onFlightMode(m->mode, m->source);
}

void thunkSystemHealth(void *ctx, const navlink_frame_hdr_t *,
                       const navlink_system_health_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (r->onSystemHealth)
    r->onSystemHealth(m->tx_overflow, m->imu_drop, m->log_wrap, m->cpu_load);
}

void thunkDefault(void *ctx, const navlink_frame_hdr_t *, uint32_t msgid,
                  const uint8_t *, size_t len) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (r->onDefault)
    r->onDefault(msgid, static_cast<int>(len));
}
}  // namespace

struct NavlinkRouter::Impl {
  navlink_parser_t parser;
  navlink_handlers_t handlers;
};

NavlinkRouter::NavlinkRouter() : d_(new Impl) {
  navlink_parser_init(&d_->parser);
  d_->handlers = {};               // zero every slot
  d_->handlers.ctx = this;         // shared ctx: the router itself
  d_->handlers.on_default = thunkDefault;
  d_->handlers.on_attitude_euler = thunkAttitude;
  d_->handlers.on_imu_raw = thunkImuRaw;
  d_->handlers.on_imu_compressed = thunkImuCompressed;
  d_->handlers.on_rc_channels = thunkRc;
  d_->handlers.on_motor_telemetry = thunkMotor;
  d_->handlers.on_control_trace = thunkControlTrace;
  d_->handlers.on_est_perf = thunkEstPerf;
  d_->handlers.on_flight_mode = thunkFlightMode;
  d_->handlers.on_system_health = thunkSystemHealth;
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
