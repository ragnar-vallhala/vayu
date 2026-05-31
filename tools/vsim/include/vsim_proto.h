// vsim_proto.h — wire protocol between the vsim_d physics daemon and
// its peers (firmware host inside Navigator + the GCS viewer).
//
// Four named-pipe channels under /tmp, one direction each:
//
//   /tmp/vsim_pwm   firmware -> vsim_d   pwm_frame_t   ~1 kHz, latest-wins
//   /tmp/vsim_imu   vsim_d   -> firmware imu_frame_t   ~200 Hz, latest-wins
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
#define VSIM_PROTO_VERSION 1u

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

// IMU body: 76-byte bmx160_all_converted_reading_t mirror. The exact
// layout is fixed by the firmware's host_imu_feeder.c expectations:
//   float acc[3];           // m/s^2 in body NED
//   float gyr[3];           // deg/s in body NED  (NOT rad/s on the wire)
//   float mag[3];           // microtesla in body NED
//   float acc_raw[3];       // mirror of acc (firmware ignores when via bridge)
//   float gyr_raw[3];       // mirror of gyr
//   float mag_compensated[3]; // mirror of mag
//   float temp;             // degC
// = 19 floats = 76 bytes. We keep the type opaque here and let the
// firmware-side header own the field decomposition; what matters on the
// wire is just the byte count.
#define VSIM_IMU_PAYLOAD_BYTES 76

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
    uint32_t tick_lo;        // low  32 bits of physics tick (1 kHz)
    uint32_t tick_hi;        // high 32 bits
    float pos_w[3];          // NED position [m]
    float quat_wxyz[4];      // body->world quaternion, w first
    float vel_w[3];          // NED velocity [m/s]
    float omega_b[3];        // body angular velocity [rad/s]
    float motor_omega[4];    // per-rotor [rad/s], M1..M4
    float motor_duty[4];     // last commanded duty [0,1]
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
};

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
typedef struct {
    float pos_w[3];
    float quat_wxyz[4];
    float vel_w[3];
    float omega_b[3];
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
} vsim_ctl_world_t;

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
static_assert(sizeof(vsim_imu_frame_t)  == 16 + 76,  "vsim_imu_frame_t size");
static_assert(sizeof(vsim_pose_frame_t) == 16 + 92,  "vsim_pose_frame_t size");
static_assert(sizeof(vsim_ctl_frame_t)  == 16 + 264, "vsim_ctl_frame_t size");
static_assert(sizeof(vsim_ctl_geometry_t) == 200,    "vsim_ctl_geometry_t size");
static_assert(sizeof(vsim_ctl_world_t)   == 20,      "vsim_ctl_world_t size");
#else
_Static_assert(sizeof(vsim_hdr_t)        == 16, "vsim_hdr_t size");
_Static_assert(sizeof(vsim_pwm_frame_t)  == 16 + 16,  "vsim_pwm_frame_t size");
_Static_assert(sizeof(vsim_imu_frame_t)  == 16 + 76,  "vsim_imu_frame_t size");
_Static_assert(sizeof(vsim_pose_frame_t) == 16 + 92,  "vsim_pose_frame_t size");
_Static_assert(sizeof(vsim_ctl_frame_t)  == 16 + 264, "vsim_ctl_frame_t size");
_Static_assert(sizeof(vsim_ctl_geometry_t) == 200,    "vsim_ctl_geometry_t size");
_Static_assert(sizeof(vsim_ctl_world_t)   == 20,      "vsim_ctl_world_t size");
#endif

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // VSIM_PROTO_H
