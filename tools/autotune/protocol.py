"""Wire-protocol helpers for the SITL PID autotuner.

Two protocols are spoken here:

1. NavLink (firmware <-> GCS over the UART2 pty): an 8-byte header
   [0x56][type<<4|ver][len][dev_id][ts:4] + payload + CRC32(4). We *decode*
   the control-telemetry packet (setpoint vs measured for every loop) and
   *encode* CMD_ARM / CMD_DISARM / CMD_SET_PID.

2. vsim ctl frames (autotuner -> vsim_d over the /tmp/vsim_ctl FIFO): a
   16-byte header + subtype + reserved + 256-byte body. We build RESET,
   SET_TESTRIG, SET_RATES and parse the pose frame.

Everything is little-endian (host-native on x86_64), matching the C structs in
src/comm/serializer.c, software/src/protocol/PacketDecoder.cpp and
tools/vsim/include/vsim_proto.h. Keep this file dependency-free (stdlib only).
"""

import struct

# ---------------------------------------------------------------------------
# CRC32 — MSB-first, poly 0x04C11DB7, init 0xFFFFFFFF, no reflection, no final
# XOR. Byte-for-byte port of software/src/core/crc.cpp (the STM32 HAL CRC).
# ---------------------------------------------------------------------------
def crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= (b << 24) & 0xFFFFFFFF
        for _ in range(8):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc & 0xFFFFFFFF


# ---- NavLink framing ------------------------------------------------------
SYNC = 0x56
PROTO_VER = 0x1
PKT_SYSTEM_STATUS = 0x6
ORIGIN_CONTROL_DATA = 0x05
ORIGIN_SYS_STATE = 0x04          # SYSTEM_ORIGIN_SYS_STATE: [0x04][pad][state:f32]
ORIGIN_FLIGHT_MODE = 0x07        # [0x07][pad][mode:u8][source:u8]
PKT_COMMAND = 0x3

# sys_state_t bit values (include/sys/state.h) — the autotuner watches for ARMED
# to confirm an arm command actually took before exciting the doublet.
SYSTEM_STATE_STANDBY = 0x4
SYSTEM_STATE_ARMED = 0x10
SYSTEM_STATE_FAILSAFE = 0x40

CMD_ARM = 0x0002
CMD_DISARM = 0x0003
CMD_SET_PID = 0x000A
CMD_SET_GYRO_LPF = 0x000B
CMD_SET_MOTOR_GEOMETRY = 0x000C
CMD_SET_FLIGHT_MODE = 0x000D

# Flight-mode arg / telemetry values (firmware flight_mode_t).
FLIGHT_MODE_ANGLE = 0
FLIGHT_MODE_ACRO = 1
FLIGHT_MODE_RELEASE = 2          # CMD arg only: hand control back to the RC switch

# Index of each field inside the 18-float control-telemetry payload.
CT = {
    "roll_angle_sp": 0, "pitch_angle_sp": 1, "yaw_angle_sp": 2,
    "roll_angle_curr": 3, "pitch_angle_curr": 4, "yaw_angle_curr": 5,
    "roll_rate_sp": 6, "pitch_rate_sp": 7, "yaw_rate_sp": 8,
    "roll_rate_curr": 9, "pitch_rate_curr": 10, "yaw_rate_curr": 11,
    "roll_out": 12, "pitch_out": 13, "yaw_out": 14, "thro_out": 15,
    "outer_dt": 16, "inner_dt": 17,
}


def build_command(cmd_id: int, args=None, device_id: int = 0, timestamp: int = 0) -> bytes:
    """Frame a NavLink COMMAND packet.

    args=None -> bare command (payload is just the 2-byte cmd id, like ARM).
    args=list of floats -> payload is [cmd_id:2][argc:1][argc * f32] (SET_PID).
    """
    payload = struct.pack("<H", cmd_id)
    if args is not None:
        payload += struct.pack("<B", len(args))
        for a in args:
            payload += struct.pack("<f", float(a))
    type_byte = ((PKT_COMMAND & 0xF) << 4) | (PROTO_VER & 0xF)
    header = struct.pack("<BBBBI", SYNC, type_byte, len(payload), device_id, timestamp)
    pkt = header + payload
    return pkt + struct.pack("<I", crc32(pkt))


def set_pid_command(controller: int, axis: int, kp: float, ki: float,
                    kd: float, kff: float) -> bytes:
    """CMD_SET_PID: controller 0=angle/1=rate, axis 0=roll/1=pitch/2=yaw."""
    return build_command(CMD_SET_PID,
                         [float(controller), float(axis), kp, ki, kd, kff])


def set_gyro_lpf_command(axis: int, rc: float) -> bytes:
    """CMD_SET_GYRO_LPF: rate-loop gyro low-pass time constant [s] (<=0 = off)."""
    return build_command(CMD_SET_GYRO_LPF, [float(axis), float(rc)])


def set_flight_mode_command(mode: int) -> bytes:
    """CMD_SET_FLIGHT_MODE: 0=stabilise/angle, 1=acro, 2=release to RC switch."""
    return build_command(CMD_SET_FLIGHT_MODE, [float(mode)])


def set_motor_geometry_command(motors) -> bytes:
    """CMD_SET_MOTOR_GEOMETRY: per-motor body x,y,spin -> firmware mixer signs.
    Args: x[4], y[4], spin[4] (12 floats), so the firmware mix matches the sim."""
    xs = [m["pos"][0] for m in motors]
    ys = [m["pos"][1] for m in motors]
    sp = [m["spin"] for m in motors]
    return build_command(CMD_SET_MOTOR_GEOMETRY, xs + ys + sp)


class NavlinkDecoder:
    """Incremental decoder. Feed bytes; pull decoded control-telemetry dicts.

    Mirrors DroneProtocol::parseBuffer: scan for sync, check version nibble,
    length, validate CRC32 over header+payload, then dispatch.
    """

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
        """Append bytes and yield every decoded frame found.

        Yields two kinds of dict:
          - control-telemetry: the 18-float CT.* fields (ORIGIN_CONTROL_DATA).
          - system-state:      {"sys_state": float} (ORIGIN_SYS_STATE) — used to
            confirm ARMED before a rollout. Consumers branch on key presence.
        """
        self._buf.extend(data)
        out = []
        while True:
            i = self._buf.find(SYNC)
            if i < 0:
                self._buf.clear()
                break
            if i > 0:
                del self._buf[:i]
            if len(self._buf) < 8:
                break
            type_byte = self._buf[1]
            if (type_byte & 0x0F) != PROTO_VER:
                del self._buf[0]
                continue
            length = self._buf[2]
            total = 8 + length + 4
            if len(self._buf) < total:
                break
            frame = bytes(self._buf[:total])
            if crc32(frame[:8 + length]) != struct.unpack_from("<I", frame, 8 + length)[0]:
                del self._buf[0]            # false sync; resync
                continue
            ptype = (type_byte >> 4) & 0x0F
            if (ptype == PKT_SYSTEM_STATUS and length == 74
                    and frame[8] == ORIGIN_CONTROL_DATA):
                vals = struct.unpack_from("<18f", frame, 10)
                out.append({k: vals[i] for k, i in CT.items()})
            elif (ptype == PKT_SYSTEM_STATUS and length == 6
                    and frame[8] == ORIGIN_SYS_STATE):
                # [0x04][pad][state:f32] — state is a sys_state_t bit value.
                out.append({"sys_state": struct.unpack_from("<f", frame, 10)[0]})
            elif (ptype == PKT_SYSTEM_STATUS and length == 4
                    and frame[8] == ORIGIN_FLIGHT_MODE):
                # [0x07][pad][mode:u8][source:u8]
                out.append({"flight_mode": frame[10], "flight_mode_src": frame[11]})
            del self._buf[:total]
        return out


# ---- vsim ctl / pose frames ----------------------------------------------
VSIM_MAGIC = 0x4D495356
VSIM_PROTO_VERSION = 1
VSIM_FRAME_POSE = 3
VSIM_FRAME_CTL = 4
VSIM_CTL_RESET = 1
VSIM_CTL_SET_GEOMETRY = 5
VSIM_CTL_SET_WORLD = 6
VSIM_CTL_SET_RATES = 9
VSIM_CTL_SET_TESTRIG = 12

_CTL_BODY = 256
_CTL_TOTAL = 16 + 8 + _CTL_BODY      # hdr(16) + subtype(4) + reserved(4) + body


def _ctl_frame(subtype: int, body: bytes) -> bytes:
    body = (body + b"\x00" * _CTL_BODY)[:_CTL_BODY]
    hdr = struct.pack("<IHHII", VSIM_MAGIC, VSIM_PROTO_VERSION,
                      VSIM_FRAME_CTL, _CTL_TOTAL - 16, 0)
    return hdr + struct.pack("<II", subtype, 0) + body


def ctl_reset(pos=(0, 0, -0.05), quat=(1, 0, 0, 0), vel=(0, 0, 0), omega=(0, 0, 0),
              seed=0) -> bytes:
    # Trailing u32 seed (vsim_ctl_reset_t.seed): non-zero => vsim_d re-seeds the
    # sensor-noise RNG + zeroes biases for a reproducible rollout; 0 => legacy
    # free-running noise.
    body = struct.pack("<3f4f3f3f", *pos, *quat, *vel, *omega) + struct.pack("<I", int(seed) & 0xFFFFFFFF)
    return _ctl_frame(VSIM_CTL_RESET, body)


def ctl_testrig(enable: bool, pos=(0, 0, -0.05), tether_k=0.0) -> bytes:
    # tether_k>0 => soft rig (spring-damped pull-back, lets the body translate so
    # the accel sees free-flight thrust-tilt); 0 => legacy hard pin.
    return _ctl_frame(VSIM_CTL_SET_TESTRIG,
                      struct.pack("<i3ff", 1 if enable else 0, *pos, float(tether_k)))


def ctl_world(gravity=9.81, ground_z=0.0, restitution=0.3, linear_drag=0.10,
              angular_drag=0.005, ground_right_gain=8.0, ground_right_damp=3.0) -> bytes:
    """vsim_ctl_world_t: environment + aerodynamics. Lets the tuner fly the SAME
    plant the GCS flies (esp. angular_drag — a no-rate_kd tune is only stable if
    SOME damping exists, passive or active)."""
    body = struct.pack("<7f", gravity, ground_z, restitution, linear_drag,
                       angular_drag, ground_right_gain, ground_right_damp)
    return _ctl_frame(VSIM_CTL_SET_WORLD, body)


def ctl_rates(imu_hz: int, physics_hz: int, pose_hz: int) -> bytes:
    return _ctl_frame(VSIM_CTL_SET_RATES, struct.pack("<III", imu_hz, physics_hz, pose_hz))


def ctl_geometry(mass: float, inertia9, motors) -> bytes:
    """vsim_ctl_geometry_t: mass, 3x3 inertia (row-major, 9 floats), then 4
    motors {pos[3], axis[3], spin, k_thrust, k_moment, max_omega}. 200 bytes."""
    body = struct.pack("<f", float(mass)) + struct.pack("<9f", *[float(v) for v in inertia9])
    for m in motors:
        body += struct.pack("<3f3f4f",
                            *[float(v) for v in m["pos"]],
                            *[float(v) for v in m["axis"]],
                            float(m["spin"]), float(m["k_thrust"]),
                            float(m["k_moment"]), float(m["max_omega"]))
    return _ctl_frame(VSIM_CTL_SET_GEOMETRY, body)


_POSE_FMT = "<IHHII"          # header
_POSE_BODY = "<II3f4f3f3f4f4f"   # tick_lo,tick_hi,pos,quat,vel,omega,m_omega,m_duty
POSE_SIZE = 108


def parse_pose(frame: bytes):
    """Return dict with pos/quat/vel/omega from a 108-byte pose frame, or None."""
    if len(frame) < POSE_SIZE:
        return None
    magic, _, ftype, _, _ = struct.unpack_from(_POSE_FMT, frame, 0)
    if magic != VSIM_MAGIC or ftype != VSIM_FRAME_POSE:
        return None
    v = struct.unpack_from(_POSE_BODY, frame, 16)
    return {"pos": v[2:5], "quat": v[5:9], "vel": v[9:12], "omega": v[12:15]}
