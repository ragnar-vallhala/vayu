/*
 * vsim_inproc.cpp -- in-process vsim physics for the RTOS SITL stepper
 * (Phase 4 step 2a, firmware/docs/plans/sitl-lockstep-sim.md).
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
#include "trimesh_bvh.h"  // vsim::trimesh::Bvh for VSIM_CTL_SET_WORLD_MESH
#include "vsim_proto.h"   // vsim_ctl_* wire layouts (the GCS config surface)

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <fcntl.h>     // open()        — world-mesh mmap
#include <sys/mman.h>  // mmap/munmap
#include <sys/stat.h>  // fstat
#include <unistd.h>    // close

namespace {
constexpr float kRad2Deg = 57.29577951308232f;
vsim::SimController g_ctl;

// ---- pose snapshot (#11c) -------------------------------------------------
// The stepper thread publishes a pose frame after each step; the GCS GUI thread
// reads it via vsim_inproc_get_pose(). A seqlock keeps the read lock-free and
// torn-write-free: the reader retries while the sequence is odd (write in
// progress) or changed across the copy. Headless runs simply never read it.
std::atomic<uint32_t> g_pose_seq{0};
vsim_pose_frame_t      g_pose_buf{};
uint64_t               g_pose_tick = 0;   // physics steps since reset
std::array<float, 4>   g_last_duty{0, 0, 0, 0};

// ---- live config + fault state (#11d) -------------------------------------
// Persistent params the config setters mutate then re-push (SET_WORLD keeps the
// rotor layout; SET_GEOMETRY keeps drag/world — same split as vsim_d's locals).
vsim::DroneParams g_drone;
vsim::MotorParams g_motor;
std::vector<vsim::SimObstacle> g_obstacles;
// Faults / sensor enables / pause, applied in vsim_inproc_step exactly where
// vsim_d applies them (motor-kill before stepping, dropout/enable after sampling).
bool g_motor_kill[4] = {false, false, false, false};
bool g_imu_dropout = false;
bool g_acc_en = true, g_gyr_en = true, g_mag_en = true;
bool g_paused = false;
int  g_substeps = 8;        // physics_hz / imu_hz; SET_RATES overrides
vsim::ImuSample g_imu_held; // last good sample, frozen during imu_dropout
// World collision mesh: the mmap must outlive setWorldMesh (Bvh points into it).
void  *g_world_map = nullptr;
size_t g_world_map_size = 0;

void dropWorldMesh() {
  g_ctl.clearWorldMesh();
  if (g_world_map) { ::munmap(g_world_map, g_world_map_size); g_world_map = nullptr; g_world_map_size = 0; }
}

// Shared geometry application (used by both the file loader and SET_GEOMETRY):
// mass + full inertia + per-rotor layout onto g_drone/g_motor, then push.
void applyGeometry(const vsim_ctl_geometry_t &g) {
  g_drone.mass = g.mass;
  for (int k = 0; k < 9; ++k) g_drone.inertia.m[k] = g.inertia[k];
  for (int i = 0; i < 4; ++i) {
    g_motor.pos_b[i]     = vsim::Vec3(g.motors[i].pos[0], g.motors[i].pos[1], g.motors[i].pos[2]);
    g_motor.axis_b[i]    = vsim::Vec3(g.motors[i].axis[0], g.motors[i].axis[1], g.motors[i].axis[2]);
    g_motor.spin[i]      = (g.motors[i].spin >= 0.0f) ? +1 : -1;
    g_motor.k_thrust[i]  = g.motors[i].k_thrust;
    g_motor.k_moment[i]  = g.motors[i].k_moment;
    g_motor.max_omega[i] = g.motors[i].max_omega;
    g_motor.tau[i]       = (g.motors[i].tau > 1e-6f) ? g.motors[i].tau : 0.0125f;
  }
  g_ctl.setDroneParams(g_drone);
  g_ctl.setMotorParams(g_motor);
}

void publishPose() {
  const vsim::RigidBodyState &st = g_ctl.state();
  const std::array<float, 4> &wm = g_ctl.motorOmegas();
  const vsim::Vec3 &vw = g_ctl.windWorld();

  uint32_t s = g_pose_seq.load(std::memory_order_relaxed);
  g_pose_seq.store(s + 1, std::memory_order_release);            // odd: writing
  std::atomic_thread_fence(std::memory_order_release);

  vsim_pose_frame_t &f = g_pose_buf;
  f.hdr.magic         = VSIM_MAGIC;
  f.hdr.version       = VSIM_PROTO_VERSION;
  f.hdr.type          = VSIM_FRAME_POSE;
  f.hdr.payload_bytes = sizeof(vsim_pose_frame_t) - sizeof(vsim_hdr_t);
  f.hdr.seq_no        = static_cast<uint32_t>(g_pose_tick);
  f.tick_lo           = static_cast<uint32_t>(g_pose_tick & 0xFFFFFFFFu);
  f.tick_hi           = static_cast<uint32_t>(g_pose_tick >> 32);
  f.pos_w[0] = st.pos_w.x(); f.pos_w[1] = st.pos_w.y(); f.pos_w[2] = st.pos_w.z();
  f.quat_wxyz[0] = st.att.scalar();
  f.quat_wxyz[1] = st.att.x();
  f.quat_wxyz[2] = st.att.y();
  f.quat_wxyz[3] = st.att.z();
  f.vel_w[0] = st.vel_w.x(); f.vel_w[1] = st.vel_w.y(); f.vel_w[2] = st.vel_w.z();
  f.omega_b[0] = st.omega_b.x(); f.omega_b[1] = st.omega_b.y(); f.omega_b[2] = st.omega_b.z();
  for (int i = 0; i < 4; ++i) { f.motor_omega[i] = wm[i]; f.motor_duty[i] = g_last_duty[i]; }
  f.wind_w[0] = vw.x(); f.wind_w[1] = vw.y(); f.wind_w[2] = vw.z();
  // airspeed/ge_factor/battery (proto v3) left zero until those models are wired.

  std::atomic_thread_fence(std::memory_order_release);
  g_pose_seq.store(s + 2, std::memory_order_release);            // even: stable
}

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

// Read the opt-in actuator-imperfection envs into `motor` (mirrors vsim_d's
// main.cpp so the in-process twin carries the SAME identified model — the
// ~100 ms transport delay + idle-stall that reproduce the real failure).
// Returns true if any imperfection was enabled. Defaults (unset) = ideal no-op.
bool applyActuatorEnv(vsim::MotorParams &motor) {
  if (const char *e = std::getenv("VSIM_MOTOR_DELAY_MS")) motor.transport_delay = std::atof(e) * 1e-3f;
  if (const char *e = std::getenv("VSIM_STALL_DUTY"))     motor.stall_duty = std::atof(e);
  if (const char *e = std::getenv("VSIM_RESPIN_TAU"))     motor.respin_tau = std::atof(e);
  if (const char *e = std::getenv("VSIM_VIBE_G"))         g_ctl.setVibeGain(std::atof(e));
  bool on = motor.transport_delay > 0.0f || motor.stall_duty > 0.0f;
  if (on)
    std::fprintf(stderr, "vsim_inproc: actuator imperfections ON "
                 "(delay=%.0fms stall_duty=%.3f respin=%.0fms)\n",
                 motor.transport_delay * 1e3f, motor.stall_duty, motor.respin_tau * 1e3f);
  return on;
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
  g_pose_tick = 0;
  g_last_duty = {0, 0, 0, 0};
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

  applyGeometry(g);   // mass + inertia + rotor layout onto g_drone/g_motor
  // Higher-fidelity actuator imperfections (opt-in via env), on top of the
  // geometry-derived rotor layout; re-push since applyActuatorEnv mutates g_motor.
  if (applyActuatorEnv(g_motor))
    g_ctl.setMotorParams(g_motor);
  for (int i = 0; i < 4; ++i) {
    out_x[i] = g.motors[i].pos[0];
    out_y[i] = g.motors[i].pos[1];
    out_spin[i] = g_motor.spin[i];
  }
  return 1;
}

/* Apply the actuator-imperfection envs (VSIM_MOTOR_DELAY_MS / VSIM_STALL_DUTY /
 * VSIM_RESPIN_TAU / VSIM_VIBE_G) on top of the DEFAULT reference-quad motor
 * params — for runs with no VAYU_RTOS_GEOMETRY file, where load_geometry (which
 * otherwise carries these envs) never runs. Call ONLY when geometry was not
 * loaded; otherwise it would clobber the geometry-derived rotor layout. No-op
 * if none of the envs are set. */
void vsim_inproc_apply_actuator_env_default(void) {
  vsim::MotorParams motor;  // reference-quad defaults
  if (applyActuatorEnv(motor))
    g_ctl.setMotorParams(motor);
}

/* Advance physics by dt (8 RK4 substeps, matching vsim_d's physics_hz/imu_hz),
 * sample the IMU, and write the 88-byte firmware payload. */
void vsim_inproc_step(const float duty[4], float dt, uint8_t out_imu[88]) {
  std::array<float, 4> d{duty[0], duty[1], duty[2], duty[3]};
  // Motor-kill faults: a dead ESC produces no thrust regardless of command
  // (also reflected in the pose duty). Same point vsim_d applies it.
  for (int i = 0; i < 4; ++i)
    if (g_motor_kill[i]) d[i] = 0.0f;

  const int substeps = g_substeps > 0 ? g_substeps : 8;
  const float dt_sub = dt / static_cast<float>(substeps);
  vsim::ImuSample s;
  if (!g_paused) {
    for (int i = 0; i < substeps; ++i)
      g_ctl.stepOnce(d, dt_sub);
    s = g_ctl.sampleImu(dt_sub);
    if (g_imu_dropout) s = g_imu_held;   // freeze on the last good sample
    else g_imu_held = s;
    if (!g_acc_en) s.acc = vsim::Vec3(0.0f, 0.0f, 0.0f);  // disabled sensor -> 0
    if (!g_gyr_en) s.gyr = vsim::Vec3(0.0f, 0.0f, 0.0f);
    if (!g_mag_en) s.mag = vsim::Vec3(0.0f, 0.0f, 0.0f);
  } else {
    s = g_imu_held;   // paused: hold physics, keep feeding the last sample
  }
  packImu(s, out_imu);
  g_last_duty = d;
  ++g_pose_tick;
  publishPose();   // refresh the GUI snapshot (cheap; no RNG, determinism-safe)
}

/* Modelled barometer (BME280 analog): derive static pressure from the true
 * altitude via the ISA formula — the EXACT inverse of the firmware's altitude
 * derivation, so the FC's own bme280 path recovers this altitude. Same values
 * vsim_d wrote on the /tmp/vsim_baro FIFO; the firmware-aware step engine feeds
 * them into bme280_publish (this TU can't see the firmware headers). */
void vsim_inproc_get_baro(float *pressure_pa, float *temperature_c,
                          float *humidity_rh) {
  const double altitude_up = -static_cast<double>(g_ctl.state().pos_w.z());
  double ratio = 1.0 - altitude_up / 44330.0;
  if (ratio < 0.0) ratio = 0.0;  // guard absurd altitudes
  if (pressure_pa)   *pressure_pa   = static_cast<float>(101325.0 * std::pow(ratio, 5.255));
  if (temperature_c) *temperature_c = 25.0f;   // modelled cabin/air temperature
  if (humidity_rh)   *humidity_rh   = 50.0f;   // modelled relative humidity
}

/* Latest pose snapshot for the GCS renderer — lock-free seqlock read, safe to
 * call from a different thread than the stepper. Format is the SAME
 * vsim_pose_frame_t the decoupled vsim_d publishes, so SimWorker's consumer is
 * unchanged. Reads all-zero before the first step. */
void vsim_inproc_get_pose(vsim_pose_frame_t *out) {
  for (;;) {
    uint32_t s1 = g_pose_seq.load(std::memory_order_acquire);
    if (s1 & 1u) continue;                     // writer mid-update — retry
    *out = g_pose_buf;
    std::atomic_thread_fence(std::memory_order_acquire);
    uint32_t s2 = g_pose_seq.load(std::memory_order_acquire);
    if (s1 == s2) return;                       // stable copy
  }
}

/* ===== in-process config surface (#11d) ===============================
 * One function per VSIM_CTL_* message, reusing the vsim_proto.h wire structs as
 * args. Each is a faithful port of vsim_d's dispatch switch (sim/vsim/src/main.cpp
 * ~L313-537) onto the SAME SimController setters, so the GCS configures the
 * in-process physics by direct call instead of writing the ctl FIFO. */

void vsim_inproc_reset_to(const vsim_ctl_reset_t *b) {
  vsim::RigidBodyState s;
  s.pos_w   = vsim::Vec3(b->pos_w[0], b->pos_w[1], b->pos_w[2]);
  s.att     = vsim::Quat(b->quat_wxyz[0], b->quat_wxyz[1], b->quat_wxyz[2], b->quat_wxyz[3]);
  s.vel_w   = vsim::Vec3(b->vel_w[0], b->vel_w[1], b->vel_w[2]);
  s.omega_b = vsim::Vec3(b->omega_b[0], b->omega_b[1], b->omega_b[2]);
  g_ctl.resetState(s);
  if (b->seed != 0) { g_ctl.seedSensors(b->seed); g_ctl.seedWind(b->seed); }
  g_pose_tick = 0;
  g_last_duty = {0, 0, 0, 0};
}

void vsim_inproc_set_testrig(const vsim_ctl_testrig_t *t) {
  g_ctl.setTestRig(t->enable != 0, vsim::Vec3(t->pos[0], t->pos[1], t->pos[2]), t->tether_k);
}

void vsim_inproc_set_geometry(const vsim_ctl_geometry_t *g) {
  applyGeometry(*g);
  if (applyActuatorEnv(g_motor)) g_ctl.setMotorParams(g_motor);
  // Echo the applied per-rotor layout (mirrors vsim_d's SET_GEOMETRY echo) so a
  // headless harness can assert it is flying the loaded frame's geometry, not
  // the compiled-in defaults (see fidelity verify_frame). Prefixed
  // "vsim_inproc:" and emitted on stderr, which the driver routes to the
  // harness's captured log.
  std::fprintf(stderr,
               "vsim_inproc: geometry set (m=%.3f kg, Idiag=%.4g/%.4g/%.4g)\n",
               g_drone.mass, g_drone.inertia.at(0, 0), g_drone.inertia.at(1, 1),
               g_drone.inertia.at(2, 2));
  for (int i = 0; i < 4; ++i)
    std::fprintf(stderr,
                 "vsim_inproc:   motor%d pos=(%.5f, %.5f, %.5f) spin=%d kt=%.4g\n",
                 i, g_motor.pos_b[i].x(), g_motor.pos_b[i].y(),
                 g_motor.pos_b[i].z(), g_motor.spin[i], g_motor.k_thrust[i]);
}

void vsim_inproc_set_world(const vsim_ctl_world_t *w) {
  g_drone.gravity            = w->gravity;
  g_drone.ground_z           = w->ground_z;
  g_drone.ground_restitution = w->restitution;
  g_drone.linear_drag        = w->linear_drag;
  g_drone.angular_drag       = w->angular_drag;
  g_drone.ground_right_gain  = w->ground_right_gain;
  g_drone.ground_right_damp  = w->ground_right_damp;
  g_ctl.setDroneParams(g_drone);
}

void vsim_inproc_clear_obstacles(void) {
  g_obstacles.clear();
  g_ctl.setObstacles(g_obstacles);
}

void vsim_inproc_add_obstacle(const vsim_ctl_obstacle_t *b) {
  vsim::SimObstacle ob;
  ob.type        = b->type;
  ob.pos         = vsim::Vec3(b->pos[0], b->pos[1], b->pos[2]);
  ob.size        = vsim::Vec3(b->size[0], b->size[1], b->size[2]);
  ob.rot_deg     = vsim::Vec3(b->rot_deg[0], b->rot_deg[1], b->rot_deg[2]);
  ob.restitution = b->restitution;
  g_obstacles.push_back(ob);
  g_ctl.setObstacles(g_obstacles);
}

int vsim_inproc_set_world_mesh(const vsim_ctl_world_mesh_t *m) {
  char path[sizeof m->path];
  std::memcpy(path, m->path, sizeof path);
  path[sizeof path - 1] = '\0';
  dropWorldMesh();                          // release any previous mapping first
  int fd = ::open(path, O_RDONLY);
  if (fd < 0) { std::fprintf(stderr, "vsim_inproc: world mesh open(%s) failed\n", path); return 0; }
  struct stat st {};
  if (::fstat(fd, &st) != 0 || st.st_size <= 0) {
    std::fprintf(stderr, "vsim_inproc: world mesh fstat failed\n"); ::close(fd); return 0;
  }
  void *base = ::mmap(nullptr, static_cast<size_t>(st.st_size), PROT_READ, MAP_PRIVATE, fd, 0);
  ::close(fd);                              // mapping survives the fd
  if (base == MAP_FAILED) { std::fprintf(stderr, "vsim_inproc: world mesh mmap failed\n"); return 0; }
  vsim::trimesh::Bvh bvh = vsim::trimesh::Bvh::fromBytes(
      static_cast<const uint8_t *>(base), static_cast<size_t>(st.st_size));
  if (!bvh.valid() || bvh.h->triangle_count != m->triangle_count ||
      bvh.h->vertex_count != m->vertex_count) {
    std::fprintf(stderr, "vsim_inproc: world mesh invalid/mismatch\n");
    ::munmap(base, static_cast<size_t>(st.st_size));
    return 0;
  }
  g_world_map = base;
  g_world_map_size = static_cast<size_t>(st.st_size);
  g_ctl.setWorldMesh(bvh, m->restitution);
  return 1;
}

void vsim_inproc_clear_world_mesh(void) { dropWorldMesh(); }

void vsim_inproc_set_rates(const vsim_ctl_rates_t *r) {
  int imu = static_cast<int>(r->imu_hz), phys = static_cast<int>(r->physics_hz);
  g_substeps = (imu > 0 && phys >= imu) ? phys / imu : 8;   // physics substeps/sample
}

void vsim_inproc_set_noise(const vsim_ctl_noise_t *n) {
  vsim::SensorNoise sn;   // start from defaults, override σ/clip
  sn.acc_noise_std = n->acc_sigma; sn.acc_bias_clip = n->acc_bias_clip;
  sn.gyr_noise_std = n->gyr_sigma; sn.gyr_bias_clip = n->gyr_bias_clip;
  sn.mag_noise_std = n->mag_sigma; sn.mag_bias_clip = n->mag_bias_clip;
  g_ctl.setNoise(sn);
  g_acc_en = (n->acc_enable != 0);
  g_gyr_en = (n->gyr_enable != 0);
  g_mag_en = (n->mag_enable != 0);
}

void vsim_inproc_set_faults(const vsim_ctl_faults_t *f) {
  for (int i = 0; i < 4; ++i) g_motor_kill[i] = (f->motor_kill[i] != 0);
  g_imu_dropout = (f->imu_dropout != 0);
}

void vsim_inproc_set_wind(const vsim_ctl_wind_t *w) {
  vsim::WindConfig wc;
  wc.steady      = vsim::Vec3(w->steady[0], w->steady[1], w->steady[2]);
  wc.gust_amp    = w->gust_amp;
  wc.gust_period = w->gust_period;
  wc.turb_sigma  = w->turb_sigma;
  wc.turb_tau    = w->turb_tau;
  wc.enable      = (w->enable != 0);
  g_ctl.setWind(wc);
}

void vsim_inproc_set_pause(int paused) { g_paused = (paused != 0); }

}  // extern "C"
