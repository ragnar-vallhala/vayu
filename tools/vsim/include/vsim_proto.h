// vsim_proto.h — wire protocol between the vsim_d physics daemon and
// its peers (firmware host inside Navigator + the GCS viewer).
//
// Four named-pipe channels under /tmp, one direction each:
//
//   /tmp/vsim_pwm   firmware -> vsim_d   pwm_frame_t   ~1 kHz, latest-wins
//   /tmp/vsim_imu   vsim_d   -> firmware imu_frame_t   ~1 kHz, latest-wins
//   /tmp/vsim_pose  vsim_d   -> viewer   pose_frame_t  ~60 Hz, latest-wins
//   /tmp/vsim_ctl   viewer   -> vsim_d   ctl_msg_t     async, one-shot
//
// Frames are POD, little-endian (host-native on x86_64), with a fixed-
// size header so a reader can resync after a torn write by hunting for
// the magic word. seq_no monotonically increases on the producer side;
// a consumer that only cares about "latest" can drop any frame with
// seq_no <= last_consumed_seq.
//
// All multi-byte ints/floats are little-endian. No padding inside the
// structs — they're packed via natural alignment (verified by static
// asserts at the bottom of the header).
//
// This header has no library dependencies and compiles as either C or
// C++. Include from both Navigator (C++) and the firmware host (C).
#ifndef VSIM_PROTO_H
#define VSIM_PROTO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Magic bytes that lead every frame. ASCII "VSIM" little-endian =
// 0x4D495356 on x86_64. Chosen so a tcpdump-style ascii dump of the
// FIFO is human-readable enough to spot framing errors.
#define VSIM_MAGIC 0x4D495356u

// Bump on any wire-incompatible change. Producer / consumer compare
// versions on first frame and exit if mismatched.
//   v3: pose frame extended with environment + battery telemetry
//       (wind_w, airspeed, ge_factor, batt_*) for the sim-fidelity features.
#define VSIM_PROTO_VERSION 3u

// Frame type tags. Each one is locked to a specific struct; the
// receiver dispatches on type after validating magic + length.
enum {
    VSIM_FRAME_PWM   = 1,
    VSIM_FRAME_IMU   = 2,
    VSIM_FRAME_POSE  = 3,
    VSIM_FRAME_CTL   = 4,
};

// Common 16-byte header. Fixed prefix on every frame on every channel.
//   magic    -- VSIM_MAGIC. Lets a recovering reader find the next frame.
//   version  -- VSIM_PROTO_VERSION. Bumps on wire breaks.
//   type     -- one of VSIM_FRAME_*.
//   payload_bytes -- size of the body that follows this header, in bytes.
//   seq_no   -- monotonically increasing per-channel sequence number.
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t type;
    uint32_t payload_bytes;
    uint32_t seq_no;
} vsim_hdr_t;

// PWM body: latest motor duty cycles in [0, 1], M1..M4.
typedef struct {
    vsim_hdr_t hdr;
    float duty[4];
} vsim_pwm_frame_t;

// IMU body: 88-byte bmx160_all_converted_reading_t mirror. The exact
// layout is fixed by the firmware's host_imu_feeder.c expectations:
//   float acc[3];           // m/s^2 in body NED
//   float gyr[3];           // deg/s in body NED  (NOT rad/s on the wire)
//   float mag[3];           // microtesla in body NED
//   float acc_raw[3];       // mirror of acc (firmware ignores when via bridge)
//   float gyr_raw[3];       // mirror of gyr
//   float mag_compensated[3]; // mirror of mag
//   float mag_fusion[3];    // calibrated + unit-normalized (estimator input)
//   float temp;             // degC
// = 22 floats = 88 bytes. We keep the type opaque here and let the
// firmware-side header own the field decomposition; what matters on the
// wire is just the byte count. (Was 19 floats / 76 B before bmx160's
// mag_fusion[3] field was added — a wire break, hence the proto-version bump.)
#define VSIM_IMU_PAYLOAD_BYTES 88

typedef struct {
    vsim_hdr_t hdr;
    uint8_t imu_payload[VSIM_IMU_PAYLOAD_BYTES];
} vsim_imu_frame_t;

// Pose body: snapshot of rigid-body state + motor visuals, for the
// renderer. NED frame; orientation as w-first quaternion.
//
// tick_count is split into two u32s so the whole struct stays
// 4-byte-aligned and has no trailing pad. Reassemble as
// ((uint64_t)tick_hi << 32) | tick_lo on the consumer side.
typedef struct {
    vsim_hdr_t hdr;
    uint32_t tick_lo;        // low  32 bits of physics tick (8 kHz)
    uint32_t tick_hi;        // high 32 bits
    float pos_w[3];          // NED position [m]
    float quat_wxyz[4];      // body->world quaternion, w first
    float vel_w[3];          // NED velocity [m/s]
    float omega_b[3];        // body angular velocity [rad/s]
    float motor_omega[4];    // per-rotor [rad/s], M1..M4
    float motor_duty[4];     // last commanded duty [0,1]
    // ---- sim-fidelity telemetry (proto v3) -------------------------------
    // Reserved here so the wire breaks once; each block is populated by its
    // own phase and reads zero until then (see docs/sim-fidelity/00-phasing.md).
    float wind_w[3];         // instantaneous world-frame wind [m/s] NED (Phase 1)
    float airspeed;          // air-relative speed ‖v_rel‖ [m/s]      (Phase 2)
    float ge_factor;         // live ground-effect thrust multiplier  (Phase 2)
    float batt_voltage;      // terminal voltage [V]                  (Phase 5)
    float batt_current;      // pack current [A]                      (Phase 5)
    float batt_mah_used;     // consumed charge [mAh]                 (Phase 5)
    float batt_soc;          // state of charge [0,1]                 (Phase 5)
} vsim_pose_frame_t;

// Ctl message types. Body interpretation varies; readers should
// branch on hdr.type's subtype field encoded in payload[0..3].
enum {
    VSIM_CTL_RESET        = 1,  // body: vsim_ctl_reset_t
    VSIM_CTL_PAUSE        = 2,  // body: int32 paused (0/1)
    VSIM_CTL_SET_NOISE    = 3,  // body: vsim_ctl_noise_t (future)
    VSIM_CTL_PING         = 4,  // body: empty; daemon replies via stderr log
    VSIM_CTL_SET_GEOMETRY = 5,  // body: vsim_ctl_geometry_t (mass+inertia+motors)
    VSIM_CTL_SET_WORLD    = 6,  // body: vsim_ctl_world_t (gravity+ground+drag)
    VSIM_CTL_CLEAR_OBSTACLES = 7,  // body: empty — drop all world obstacles
    VSIM_CTL_ADD_OBSTACLE    = 8,  // body: vsim_ctl_obstacle_t — append one
    VSIM_CTL_SET_RATES       = 9,  // body: vsim_ctl_rates_t — loop/sample rates
    VSIM_CTL_SET_WORLD_MESH  = 10, // body: vsim_ctl_world_mesh_t — mmap a BVH file
    VSIM_CTL_CLEAR_WORLD_MESH= 11, // body: empty — drop the world mesh
    VSIM_CTL_SET_TESTRIG     = 12, // body: vsim_ctl_testrig_t — pin translation
    VSIM_CTL_SET_FAULTS      = 13, // body: vsim_ctl_faults_t — injected failures
    VSIM_CTL_SET_WIND        = 14, // body: vsim_ctl_wind_t — world wind field
};

// Body for VSIM_CTL_SET_FAULTS: latched failure injection for testing the
// firmware's failsafe paths. Latest-wins; clearing a flag restores normal.
//   motor_kill[i] -- non-zero forces rotor i's applied duty to 0 (dead ESC).
//   imu_dropout   -- non-zero freezes the IMU sample (stuck sensor): the frame
//                    keeps emitting at the loop rate but holds the last reading,
//                    so the estimator drifts the way a wedged sensor would.
typedef struct {
    int32_t motor_kill[4];
    int32_t imu_dropout;
} vsim_ctl_faults_t;

// Body for VSIM_CTL_SET_NOISE: per-sensor synthetic-noise model + enable.
// enable==0 drops that sensor's feed (the channel is zeroed in the sample),
// which is also how the UI's sensor-enable toggle injects a dropout fault.
typedef struct {
    float   acc_sigma;      // accel white noise RMS [m/s^2]
    float   acc_bias_clip;  // accel bias random-walk clip [m/s^2]
    int32_t acc_enable;
    float   gyr_sigma;      // gyro white noise RMS [rad/s]
    float   gyr_bias_clip;  // gyro bias clip [rad/s]
    int32_t gyr_enable;
    float   mag_sigma;      // mag white noise RMS [uT]
    float   mag_bias_clip;  // mag bias clip [uT]
    int32_t mag_enable;
} vsim_ctl_noise_t;

// Body for VSIM_CTL_SET_WIND: a world-frame wind field the airframe feels as
// relative-velocity drag (docs/sim-fidelity/01-wind-turbulence.md). Sum of a
// steady component, a deterministic 1-cos-style gust, and a Dryden first-order
// band-limited turbulence filter. enable==0 is a fast bypass (no wind, no RNG
// draw) so the default sim behaviour is byte-identical to "no wind sent".
typedef struct {
    float   steady[3];     // v_steady NED [m/s]   (windN, windE, windD)
    float   gust_amp;      // peak gust [m/s]      (windGust)
    float   gust_period;   // gust period [s], <=0 disables the gust
    float   turb_sigma;    // turbulence RMS [m/s] (windTurb)
    float   turb_tau;      // correlation time [s] (<=0 -> default 1.0)
    int32_t enable;        // 0 = no wind at all (fast bypass)
} vsim_ctl_wind_t;         // 32 B

typedef struct {
    vsim_hdr_t hdr;
    uint32_t subtype;        // one of VSIM_CTL_*
    uint32_t reserved;       // pad to 8-byte alignment for the body below
    // Body varies by subtype. 256 B is sized to hold the largest payload
    // (vsim_ctl_geometry_t, ~200 B); unused bytes ignored. NOTE: the ctl
    // channel is Navigator <-> vsim_d ONLY -- the firmware host shims
    // never touch it -- so growing this body does NOT change the
    // pwm/imu/pose wire formats and needs no VSIM_PROTO_VERSION bump.
    uint8_t body[256];
} vsim_ctl_frame_t;

// Body for VSIM_CTL_RESET: re-spawn at this pose.
//   seed -- if non-zero, deterministically re-seed the sensor-noise RNG and
//           zero the random-walk biases, so an identical reset reproduces an
//           identical noise trajectory (autotuner repeatability). seed==0
//           leaves the noise stream free-running (legacy GCS viewer behavior).
//           Appended at the tail so older senders that zero-pad the ctl body
//           (Python _ctl_frame, GCS `vsim_ctl_reset_t{}`) decode as seed==0.
typedef struct {
    float pos_w[3];
    float quat_wxyz[4];
    float vel_w[3];
    float omega_b[3];
    uint32_t seed;
} vsim_ctl_reset_t;

// Body for VSIM_CTL_SET_GEOMETRY: full mass properties + 4-motor layout,
// computed GCS-side from the airframe mesh + the motor-mapping editor.
//   mass     -- kg
//   inertia  -- body-frame 3x3 tensor [kg*m^2], row-major (9 floats)
//   motors[] -- per rotor: position [m], unit thrust axis, spin (+1 CCW /
//               -1 CW), k_thrust, k_moment, max_omega [rad/s]
typedef struct {
    float mass;
    float inertia[9];
    struct {
        float pos[3];
        float axis[3];
        float spin;
        float k_thrust;
        float k_moment;
        float max_omega;
        float tau;        // first-order rotor spin-up time constant [s]; <=0 = daemon default
    } motors[4];
} vsim_ctl_geometry_t;

// Body for VSIM_CTL_SET_WORLD: environment + aerodynamics, edited in the
// World tab. Composes with SET_GEOMETRY on the daemon side (each message
// touches a disjoint set of DroneParams fields).
//   gravity      -- m/s^2 (world +Z down)
//   ground_z     -- NED z of the ground plane [m]
//   restitution  -- ground bounce factor [0,1]
//   linear_drag  -- N per (m/s)
//   angular_drag -- N*m per (rad/s)
typedef struct {
    float gravity;
    float ground_z;
    float restitution;
    float linear_drag;
    float angular_drag;
    float ground_right_gain;  // tipped-airframe righting gain [rad/s^2]
    float ground_right_damp;  // righting angular damping [1/s]
} vsim_ctl_world_t;

// Body for VSIM_CTL_ADD_OBSTACLE: one static world shape (NED world frame).
//   type        -- 0 box, 1 sphere, 2 cylinder
//   pos         -- world center [m]
//   size        -- box: full extents; sphere: x is radius; cylinder: x is
//                  radius, z is height
//   rot_deg     -- Euler XYZ [deg]
//   restitution -- bounce factor [0,1] on collision
typedef struct {
    int32_t type;
    float   pos[3];
    float   size[3];
    float   rot_deg[3];
    float   restitution;
} vsim_ctl_obstacle_t;

// Body for VSIM_CTL_SET_TESTRIG: a "tuning rig" that pins the body's
// translation to `pos` (zeroing linear velocity every step) while leaving
// rotation free. Turns the sim into a frictionless 3-DOF attitude gimbal so a
// PID autotuner can excite clean roll/pitch/yaw step responses without the
// craft drifting or needing altitude hold. enable=0 restores free flight.
typedef struct {
    int32_t enable;     // 0 = free flight, non-zero = pinned attitude rig
    float   pos[3];     // NED world position to hold the body at [m]
    // tether_k > 0 => SOFT rig: instead of hard-pinning translation, pull the
    // body back to `pos` with a critically-damped spring (stiffness tether_k
    // [1/s^2]). The body can then translate during a maneuver, so the
    // accelerometer sees the thrust-tilt corruption free flight has (a hard pin
    // hides it, which makes the autotuner over-tune). 0 => legacy hard pin.
    // Appended at the tail; zero-padding senders decode as 0 (hard pin).
    float   tether_k;
} vsim_ctl_testrig_t;

// Body for VSIM_CTL_SET_RATES: simulation loop rates [Hz].
//   imu_hz     -- IMU emit + wall-clock pace rate; the firmware's inner loop
//                 runs once per IMU sample, so this IS the firmware loop rate.
//   physics_hz -- RK4 integration rate; the daemon runs physics_hz/imu_hz
//                 substeps per IMU sample (>= imu_hz). Higher = finer
//                 integration / less collision tunnelling, same sample rate.
//   pose_hz    -- pose-frame (render) rate to the GCS.
typedef struct {
    uint32_t imu_hz;
    uint32_t physics_hz;
    uint32_t pose_hz;
} vsim_ctl_rates_t;

// Body for VSIM_CTL_SET_WORLD_MESH: the world collision mesh is too big for the
// 256 B ctl body, so the GCS writes a serialized BVH blob (trimesh_bvh.h) to a
// file and sends just the path + counts. The daemon mmaps it read-only. Counts
// are carried redundantly so the daemon can cross-check the blob header.
typedef struct {
    uint32_t vertex_count;
    uint32_t triangle_count;
    uint32_t node_count;
    uint32_t flags;             // bit0: double-sided
    float    restitution;       // global world-mesh bounce factor
    uint32_t path_len;
    char     path[216];         // NUL-terminated mmap-file path (suffixed)
} vsim_ctl_world_mesh_t;

// Canonical FIFO paths. Daemon and clients both default to these.
#define VSIM_FIFO_PWM   "/tmp/vsim_pwm"
#define VSIM_FIFO_IMU   "/tmp/vsim_imu"
#define VSIM_FIFO_POSE  "/tmp/vsim_pose"
#define VSIM_FIFO_CTL   "/tmp/vsim_ctl"

// Static size locks. If any of these fail to compile, the wire format
// has drifted and producer/consumer pair will desync silently.
#ifdef __cplusplus
static_assert(sizeof(vsim_hdr_t)        == 16, "vsim_hdr_t size");
static_assert(sizeof(vsim_pwm_frame_t)  == 16 + 16,  "vsim_pwm_frame_t size");
static_assert(sizeof(vsim_imu_frame_t)  == 16 + 88,  "vsim_imu_frame_t size");
static_assert(sizeof(vsim_pose_frame_t) == 16 + 128, "vsim_pose_frame_t size");
static_assert(sizeof(vsim_ctl_frame_t)  == 16 + 264, "vsim_ctl_frame_t size");
static_assert(sizeof(vsim_ctl_geometry_t) == 216,    "vsim_ctl_geometry_t size");
static_assert(sizeof(vsim_ctl_world_t)   == 28,      "vsim_ctl_world_t size");
static_assert(sizeof(vsim_ctl_wind_t)    == 32,      "vsim_ctl_wind_t size");
static_assert(sizeof(vsim_ctl_world_mesh_t) <= 256,  "vsim_ctl_world_mesh_t fits ctl body");
#else
_Static_assert(sizeof(vsim_hdr_t)        == 16, "vsim_hdr_t size");
_Static_assert(sizeof(vsim_pwm_frame_t)  == 16 + 16,  "vsim_pwm_frame_t size");
_Static_assert(sizeof(vsim_imu_frame_t)  == 16 + 88,  "vsim_imu_frame_t size");
_Static_assert(sizeof(vsim_pose_frame_t) == 16 + 128, "vsim_pose_frame_t size");
_Static_assert(sizeof(vsim_ctl_frame_t)  == 16 + 264, "vsim_ctl_frame_t size");
_Static_assert(sizeof(vsim_ctl_geometry_t) == 216,    "vsim_ctl_geometry_t size");
_Static_assert(sizeof(vsim_ctl_world_t)   == 28,      "vsim_ctl_world_t size");
#endif

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // VSIM_PROTO_H
