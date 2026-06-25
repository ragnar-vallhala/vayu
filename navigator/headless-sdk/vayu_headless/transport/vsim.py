"""vsim_d wire protocol: control-frame builders + pose decoding.

Mirrors tools/vsim/include/vsim_proto.h. These are pure functions (no I/O), so
they unit-test without booting anything. Frame layout (carved verbatim from the
original tools/sim_host/sitl_lab.py so behaviour is identical):

    16-byte header (vsim_hdr_t)  +  payload
    CTL frames: subtype(u32) + reserved(u32) + body[256]
"""
import math
import struct

MAGIC, VERSION = 0x4D495356, 3
FRAME_POSE = 3

# vsim_ctl_* subtypes (see vsim_proto.h).
CTL_RESET = 1
CTL_SET_GEOMETRY = 5
CTL_SET_WORLD = 6
CTL_SET_WORLD_MESH = 10
CTL_SET_TESTRIG = 12
CTL_SET_WIND = 14

HDR = struct.Struct("<IHHII")
POSE = struct.Struct("<II3f4f3f3f4f4f3fffffff")   # v3 body, 128 B
assert HDR.size == 16 and POSE.size == 128


def vhdr(typ, plen, seq=0):
    return HDR.pack(MAGIC, VERSION, typ, plen, seq)


def ctl(subtype, body):
    body = body[:256].ljust(256, b"\x00")
    payload = struct.pack("<II", subtype, 0) + body
    return vhdr(4, len(payload)) + payload


def reset(pos=(0, 0, -0.05), seed=1):
    return ctl(CTL_RESET, struct.pack("<3f4f3f3fI", *pos, 1, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, seed))


def world(gravity=9.81, ground_z=50.0, lin_drag=0.10):
    # ground_z far below so a rig/airborne craft never clamps unexpectedly.
    return ctl(CTL_SET_WORLD, struct.pack("<7f", gravity, ground_z, 0.0,
                                          lin_drag, 0.005, 40.0, 6.0))


def testrig(enable, pos=(0, 0, -1.0), tether_k=0.0):
    return ctl(CTL_SET_TESTRIG, struct.pack("<i3ff", 1 if enable else 0,
                                            pos[0], pos[1], pos[2], tether_k))


def wind(steady=(0, 0, 0), turb=0.0, enable=True):
    return ctl(CTL_SET_WIND, struct.pack("<3f4fi", steady[0], steady[1],
               steady[2], 0.0, 0.0, turb, 1.0, 1 if enable else 0))


def world_mesh(path, nverts, ntris, nodes, restitution, double_sided):
    """vsim_ctl_world_mesh_t: counts + flags + restitution + the BVH file path
    the daemon mmaps. Counts must match the blob header (daemon cross-checks)."""
    pb = path.encode()[:215]
    body = struct.pack("<IIIIfI", nverts, ntris, nodes,
                       1 if double_sided else 0, float(restitution), len(pb))
    body += pb + b"\x00" * (216 - len(pb))
    return ctl(CTL_SET_WORLD_MESH, body)


def quat_to_euler(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(v) for v in (roll, pitch, yaw)]


def parse_pose(buf):
    """Consume vsim frames from `buf` (a bytearray, trimmed in place) and return
    the LATEST decoded pose dict, or None if no complete pose frame was present.
    Matches the original SitlLab._parse_pose semantics exactly."""
    truth = None
    pos = 0
    while pos + HDR.size <= len(buf):
        magic, ver, t, plen, seq = HDR.unpack_from(buf, pos)
        if magic != MAGIC:
            pos += 1
            continue
        if pos + HDR.size + plen > len(buf):
            break
        if t == FRAME_POSE and plen >= POSE.size:
            p = POSE.unpack_from(buf, pos + HDR.size)
            truth = {
                "pos": p[2:5], "quat": p[5:9], "vel": p[9:12],
                "omega": p[12:15], "motor_omega": p[15:19],
                "wind": p[23:26],
            }
        pos += HDR.size + plen
    del buf[:pos]
    return truth
