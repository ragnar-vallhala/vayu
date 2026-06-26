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
#include "vsim_proto.h"  // vsim_ctl_geometry_t (the GCS wire layout)

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
constexpr float kRad2Deg = 57.29577951308232f;
vsim::SimController g_ctl;

// Identical to vsim_d's packImu (sim/vsim/src/main.cpp): 22 floats = 88 B,
// matching the firmware's bmx160_all_converted_reading_t layout (acc, gyr, mag,
// acc_raw, gyr_raw, mag_compensated, mag_fusion, temp). mag_fusion[3] is the
// estimator's heading reference — it MUST be filled or yaw is unobservable.
void packImu(const vsim::ImuSample &s, uint8_t out[88]) {
  float f[22];
  f[0] = s.acc.x();  f[1] = s.acc.y();  f[2] = s.acc.z();
  f[3] = s.gyr.x() * kRad2Deg;
  f[4] = s.gyr.y() * kRad2Deg;
  f[5] = s.gyr.z() * kRad2Deg;
  f[6] = s.mag.x();  f[7] = s.mag.y();  f[8] = s.mag.z();
  f[9]  = f[0]; f[10] = f[1]; f[11] = f[2];   // acc_raw
  f[12] = f[3]; f[13] = f[4]; f[14] = f[5];   // gyr_raw
  f[15] = f[6]; f[16] = f[7]; f[17] = f[8];   // mag_compensated
  // mag_fusion[3]: unit-normalized mag (estimator heading input).
  const float mn = std::sqrt(f[6] * f[6] + f[7] * f[7] + f[8] * f[8]);
  if (mn > 1e-6f) { f[18] = f[6] / mn; f[19] = f[7] / mn; f[20] = f[8] / mn; }
  else            { f[18] = f[19] = f[20] = 0.0f; }
  f[21] = s.temp;
  std::memcpy(out, f, 88);
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

/* Soft-rig stiffness for the attitude doublet. tether_k > 0 spring-tethers
 * translation instead of hard-pinning it, so a tilt produces the free-flight
 * thrust-tilt translation the estimator sees — matching the realtime autotune
 * rig (RolloutParams.tetherK, default 30). On a HARD pin (k=0) the angle cost is
 * pathological: a near-motionless craft minimises tracking IAE, so the search
 * drives angle_kp to its floor. The soft rig makes angle_kp the responsiveness
 * lever, exactly as in the realtime tuner. Call after vsim_inproc_reset. */
void vsim_inproc_set_tether(float tether_k) {
  g_ctl.setTestRig(true, vsim::Vec3(0.0f, 0.0f, -2.0f), tether_k);
}

/* Load a serialized vsim_ctl_geometry_t from `path` and apply it to the
 * in-process physics — the SAME mass+inertia+per-rotor mapping vsim_d does for
 * VSIM_CTL_SET_GEOMETRY (sim/vsim/src/main.cpp), on top of the default
 * DroneParams/MotorParams (world/drag untouched). The per-motor x/y/spin are
 * written back so the caller can drive the matching firmware mix
 * (angle_rate_controller_set_motor_geometry) from the SAME geometry source.
 * Returns 1 on success, 0 if the file can't be read. Geometry persists across
 * vsim_inproc_reset (reset only re-seeds state, not params), so call once at
 * startup. */
int vsim_inproc_load_geometry(const char *path, float out_x[4], float out_y[4],
                              int out_spin[4]) {
  vsim_ctl_geometry_t g;
  std::FILE *f = std::fopen(path, "rb");
  if (!f)
    return 0;
  const size_t n = std::fread(&g, 1, sizeof g, f);
  std::fclose(f);
  if (n != sizeof g)
    return 0;

  vsim::DroneParams drone;  // defaults; geometry overwrites mass + inertia only
  vsim::MotorParams motor;  // defaults; geometry overwrites the rotor layout
  drone.mass = g.mass;
  for (int k = 0; k < 9; ++k)
    drone.inertia.m[k] = g.inertia[k];
  for (int i = 0; i < 4; ++i) {
    motor.pos_b[i] = vsim::Vec3(g.motors[i].pos[0], g.motors[i].pos[1], g.motors[i].pos[2]);
    motor.axis_b[i] = vsim::Vec3(g.motors[i].axis[0], g.motors[i].axis[1], g.motors[i].axis[2]);
    motor.spin[i] = (g.motors[i].spin >= 0.0f) ? +1 : -1;
    motor.k_thrust[i] = g.motors[i].k_thrust;
    motor.k_moment[i] = g.motors[i].k_moment;
    motor.max_omega[i] = g.motors[i].max_omega;
    motor.tau[i] = (g.motors[i].tau > 1e-6f) ? g.motors[i].tau : 0.0125f;
    out_x[i] = g.motors[i].pos[0];
    out_y[i] = g.motors[i].pos[1];
    out_spin[i] = motor.spin[i];
  }
  // Higher-fidelity actuator imperfections (opt-in via env; mirrors vsim_d's
  // main.cpp so the in-process twin carries the SAME identified model — the
  // ~100 ms transport delay + idle-stall that reproduce the real failure).
  if (const char *e = std::getenv("VSIM_MOTOR_DELAY_MS")) motor.transport_delay = std::atof(e) * 1e-3f;
  if (const char *e = std::getenv("VSIM_STALL_DUTY"))     motor.stall_duty = std::atof(e);
  if (const char *e = std::getenv("VSIM_RESPIN_TAU"))     motor.respin_tau = std::atof(e);
  if (const char *e = std::getenv("VSIM_VIBE_G"))         g_ctl.setVibeGain(std::atof(e));
  if (motor.transport_delay > 0.0f || motor.stall_duty > 0.0f)
    std::fprintf(stderr, "vsim_inproc: actuator imperfections ON "
                 "(delay=%.0fms stall_duty=%.3f respin=%.0fms)\n",
                 motor.transport_delay * 1e3f, motor.stall_duty, motor.respin_tau * 1e3f);
  g_ctl.setDroneParams(drone);
  g_ctl.setMotorParams(motor);
  return 1;
}

/* Advance physics by dt (8 RK4 substeps, matching vsim_d's physics_hz/imu_hz),
 * sample the IMU, and write the 88-byte firmware payload. */
void vsim_inproc_step(const float duty[4], float dt, uint8_t out_imu[88]) {
  std::array<float, 4> d{duty[0], duty[1], duty[2], duty[3]};
  const int substeps = 8;
  const float dt_sub = dt / static_cast<float>(substeps);
  for (int i = 0; i < substeps; ++i)
    g_ctl.stepOnce(d, dt_sub);
  const vsim::ImuSample s = g_ctl.sampleImu(dt_sub);
  packImu(s, out_imu);
}

}  // extern "C"
