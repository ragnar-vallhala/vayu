/*
 * vsim_inproc.cpp -- in-process vsim physics for the RTOS SITL stepper
 * (Phase 4 step 2a, docs/plans/sitl-lockstep-sim.md).
 *
 * The two-process FIFO handshake (vsim_d <-> firmware) was measured at ~98% of
 * the stepper's wall time — pure process round-trip latency, not compute (the
 * firmware runs 50x realtime, and vsim free-running keeps up). Linking vsim's
 * SimController straight into vayu_sitl_rtos removes the FIFO entirely: the
 * stepper advances physics and reads back PWM inline, so it is BOTH fast (no
 * round-trip) AND faithful (PWM is never stale) AND deterministic (single
 * process, single thread, stepper owns the seed). C ABI for the C stepper.
 */
#include "sim_controller.h"

#include <array>
#include <cstdint>
#include <cstring>

namespace {
constexpr float kRad2Deg = 57.29577951308232f;
vsim::SimController g_ctl;

// Identical to vsim_d's packImu (tools/vsim/src/main.cpp): 19 floats = 76 B,
// matching the firmware's bmx160_all_converted_reading_t wire layout.
void packImu(const vsim::ImuSample &s, uint8_t out[76]) {
  float f[19];
  f[0] = s.acc.x();  f[1] = s.acc.y();  f[2] = s.acc.z();
  f[3] = s.gyr.x() * kRad2Deg;
  f[4] = s.gyr.y() * kRad2Deg;
  f[5] = s.gyr.z() * kRad2Deg;
  f[6] = s.mag.x();  f[7] = s.mag.y();  f[8] = s.mag.z();
  f[9]  = f[0]; f[10] = f[1]; f[11] = f[2];
  f[12] = f[3]; f[13] = f[4]; f[14] = f[5];
  f[15] = f[6]; f[16] = f[7]; f[18] = s.temp;
  f[17] = f[8];
  std::memcpy(out, f, 76);
}
}  // namespace

extern "C" {

/* Reset to a seeded, level, pinned-rig state (translation pinned so the
 * attitude scenario stays bounded; same as the autotune rig). seed != 0 makes
 * the sensor-noise + wind trajectory reproducible — the basis for determinism. */
void vsim_inproc_reset(uint32_t seed) {
  vsim::RigidBodyState s0;
  s0.pos_w = vsim::Vec3(0.0f, 0.0f, -2.0f);
  g_ctl.resetState(s0);
  if (seed) {
    g_ctl.seedSensors(seed);
    g_ctl.seedWind(seed);
  }
  g_ctl.setTestRig(true, vsim::Vec3(0.0f, 0.0f, -2.0f), 0.0f);
}

/* Advance physics by dt (8 RK4 substeps, matching vsim_d's physics_hz/imu_hz),
 * sample the IMU, and write the 76-byte firmware payload. */
void vsim_inproc_step(const float duty[4], float dt, uint8_t out_imu[76]) {
  std::array<float, 4> d{duty[0], duty[1], duty[2], duty[3]};
  const int substeps = 8;
  const float dt_sub = dt / static_cast<float>(substeps);
  for (int i = 0; i < substeps; ++i)
    g_ctl.stepOnce(d, dt_sub);
  const vsim::ImuSample s = g_ctl.sampleImu(dt_sub);
  packImu(s, out_imu);
}

}  // extern "C"
