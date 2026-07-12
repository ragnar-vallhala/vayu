"""Wire-protocol helpers for the SITL PID autotuner.

Two protocols are spoken here:

1. NavLink **v2** (firmware <-> GCS over the UART2 pty): 10-byte header
   [0x56][ver][len][incompat][seq][sysid][compid][msgid:3] + payload + CRC16.
   We encode the GCS->FC commands (SET_PID / SET_GYRO_LPF / SET_FLIGHT_MODE /
   SET_MOTOR_GEOMETRY) and decode the control-telemetry / heartbeat / flight-
   mode frames. Both directions go through the generated codec
   (navlink/generated/python/navlink_msgs.py) so this stays in lockstep with
   the dialect — the firmware RX is v2-only (navlink_parser_push), so the old
   hand-rolled v1 framing here silently applied NOTHING.

2. vsim ctl frames (autotuner -> the engine over the /tmp/vsim_ctl FIFO): a
   16-byte header + subtype + reserved + 256-byte body. We build RESET,
   SET_TESTRIG, SET_RATES and parse the pose frame. This is the vsim daemon
   protocol, NOT NavLink — left untouched.

Everything is little-endian (host-native on x86_64). Keep this file dependency-
free beyond the generated NavLink codec (stdlib only otherwise).
"""

import os as _os
import struct
import sys as _sys

# The generated NavLink v2 codec is the single source of truth for framing +
# message layout (shared with the firmware C codec and the GCS).
_GEN = _os.path.join(_os.path.dirname(__file__), "..", "..",
                     "navlink", "generated", "python")
if _GEN not in _sys.path:
    _sys.path.insert(0, _GEN)
import navlink_msgs as _nl  # noqa: E402

# Match the GCS CommandCodec exactly (proven to apply on the firmware): the
# command targets device 42 / component 1, and the frame is stamped sysid 0xFF,
# compid 1, with a rolling per-command sequence.
_TARGET_SYS = 42
_TARGET_COMP = 1
_seq = 0


def _next_seq() -> int:
    global _seq
    _seq = (_seq + 1) & 0xFF
    return _seq


def _frame(msg) -> bytes:
    """A populated CmdX dataclass -> full NavLink v2 frame bytes."""
    return _nl.encode(msg, seq=msg.req_seq, sysid=0xFF, compid=_TARGET_COMP)


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


def set_pid_command(controller: int, axis: int, kp: float, ki: float,
                    kd: float, kff: float) -> bytes:
    """CMD_SET_PID (v2): controller 0=angle/1=rate, axis 0=roll/1=pitch/2=yaw."""
    m = _nl.CmdSetPid(target_sys=_TARGET_SYS, target_comp=_TARGET_COMP,
                      req_seq=_next_seq(), controller=int(controller),
                      axis=int(axis), kp=float(kp), ki=float(ki),
                      kd=float(kd), kff=float(kff))
    return _frame(m)


def time_sync_command(seq: int = 1) -> bytes:
    """TIME_SYNC REQUEST (v2, role=0). The FC starts UNSYNCHRONISED and the
    §10.5 command gate (navlink_router command_gate -> time_sync_is_synced)
    rejects every command with TEMPORARILY_REJECTED until the GCS has
    disciplined its clock at least once. A single REQUEST with a non-INT32_MIN
    commanded_offset_ms makes the FC call time_sync_set_offset() -> _clock_synced
    = 1, after which set_pid/set_flight_mode/etc. are accepted. The harness must
    send this before any command or it runs the loaded plant, inert."""
    m = _nl.TimeSync(role=0, seq=int(seq) & 0xFF, t1_gcs_tx=0, t2_fc_rx=0,
                     t3_fc_tx=0, commanded_offset_ms=0, commanded_offset_hi_ms=0)
    return _nl.encode(m, seq=int(seq) & 0xFF, sysid=0xFF, compid=_TARGET_COMP)


def set_gyro_lpf_command(axis: int, rc: float) -> bytes:
    """CMD_SET_GYRO_LPF (v2): rate-loop gyro LPF time constant [s] (<=0 = off)."""
    m = _nl.CmdSetGyroLpf(target_sys=_TARGET_SYS, target_comp=_TARGET_COMP,
                          req_seq=_next_seq(), axis=int(axis), rc=float(rc))
    return _frame(m)


def set_flight_mode_command(mode: int) -> bytes:
    """CMD_SET_FLIGHT_MODE (v2): 0=stabilise/angle, 1=acro, 2=release to RC."""
    m = _nl.CmdSetFlightMode(target_sys=_TARGET_SYS, target_comp=_TARGET_COMP,
                             req_seq=_next_seq(), mode=int(mode), source=1)
    return _frame(m)


def set_motor_geometry_command(motors) -> bytes:
    """CMD_SET_MOTOR_GEOMETRY (v2): per-motor body x,y + spin -> firmware mixer
    signs. The v2 message carries pos_x[4], pos_y[4], spin[4]."""
    m = _nl.CmdSetMotorGeometry(target_sys=_TARGET_SYS, target_comp=_TARGET_COMP,
                                req_seq=_next_seq(), layout=0,
                                pos_x=[float(mo["pos"][0]) for mo in motors],
                                pos_y=[float(mo["pos"][1]) for mo in motors],
                                spin=[int(mo["spin"]) for mo in motors])
    return _frame(m)


class NavlinkDecoder:
    """Incremental decoder. Feed bytes; pull decoded control-telemetry dicts.

    The firmware speaks NavLink **v2** (sync 0x56, 10-byte header keyed by a
    24-bit msgid, CRC16) — not the v1 SYSTEM_STATUS/ORIGIN framing this file
    used to hand-parse (that left every rollout sample-starved -> phantom
    divergences). Decode through the generated v2 codec
    (navlink/generated/python/navlink_msgs.py) so we stay in lockstep with the
    dialect. Consumers still branch on dict key presence, exactly as before.
    """

    # ControlTrace (msgid 1030) carries the 18-float loop trace the cost
    # function scores; Heartbeat carries nav_state; FlightMode the mode/source.
    _CT_FIELDS = ("roll_angle_sp", "pitch_angle_sp", "yaw_angle_sp",
                  "roll_angle_curr", "pitch_angle_curr", "yaw_angle_curr",
                  "roll_rate_sp", "pitch_rate_sp", "yaw_rate_sp",
                  "roll_rate_curr", "pitch_rate_curr", "yaw_rate_curr",
                  "roll_out", "pitch_out", "yaw_out", "thro_out",
                  "outer_dt", "inner_dt")

    def __init__(self):
        import os, sys
        gen = os.path.join(os.path.dirname(__file__), "..", "..",
                           "navlink", "generated", "python")
        if gen not in sys.path:
            sys.path.insert(0, gen)
        import navlink_msgs as nl
        self._nl = nl
        self._out = []
        h = nl.Handlers()
        h.on_unknown = None
        h.on_crc_error = None

        def on_default(frame, msg):
            mid = frame.msgid
            if mid == nl.ControlTrace.MSGID:
                self._out.append({k: getattr(msg, k) for k in self._CT_FIELDS})
            elif mid == nl.Heartbeat.MSGID:
                # nav_state is the sequential v2 enum (STANDBY=2, ARMED=4, ...);
                # the harness compares firmware bit values (0x4/0x10/...), so
                # map enum n -> bit (1 << n).
                self._out.append(
                    {"sys_state": float(1 << int(msg.nav_state))})
            elif mid == nl.FlightMode.MSGID:
                self._out.append({"flight_mode": int(msg.mode),
                                  "flight_mode_src": int(msg.source)})

        # Null any per-msg handler so everything routes through on_default.
        for mid in (nl.ControlTrace.MSGID, nl.Heartbeat.MSGID,
                    nl.FlightMode.MSGID):
            nm = nl.MSGID_TO_HANDLER.get(mid)
            if nm and hasattr(h, nm):
                setattr(h, nm, None)
        h.on_default = on_default
        self._parser = nl.Parser(h)

    def feed(self, data: bytes):
        """Push bytes through the v2 parser; return the dicts decoded this call.

        Yields three kinds of dict (consumers branch on key presence):
          - control-telemetry: the 18 ControlTrace fields.
          - system-state:      {"sys_state": float} (firmware bit value).
          - flight-mode:       {"flight_mode": int, "flight_mode_src": int}.
        """
        self._out = []
        self._parser.push(data)
        return self._out


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
    # Trailing u32 seed (vsim_ctl_reset_t.seed): non-zero => the engine re-seeds the
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
    motors {pos[3], axis[3], spin, k_thrust, k_moment, max_omega, tau}. The
    trailing `tau` (rotor spin-up time constant) is REQUIRED: the C struct has
    it, so omitting it misaligns every motor after the first and feeds vsim a
    scrambled layout (motors read each other's fields). tau<=0 => daemon
    default. 216 bytes."""
    body = struct.pack("<f", float(mass)) + struct.pack("<9f", *[float(v) for v in inertia9])
    for m in motors:
        body += struct.pack("<3f3f5f",
                            *[float(v) for v in m["pos"]],
                            *[float(v) for v in m["axis"]],
                            float(m["spin"]), float(m["k_thrust"]),
                            float(m["k_moment"]), float(m["max_omega"]),
                            float(m.get("tau", 0.0)))
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
