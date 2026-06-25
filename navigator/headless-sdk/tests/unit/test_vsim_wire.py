"""Unit tests for the vsim wire layer (no I/O, no daemon)."""
import struct

import pytest

from vayu_headless.transport import vsim


def test_struct_sizes_match_proto():
    # vsim_hdr_t == 16, pose body == 128 (vsim_proto.h static_asserts).
    assert vsim.HDR.size == 16
    assert vsim.POSE.size == 128


def test_ctl_frame_shape():
    # CTL frame = 16-byte hdr + subtype(4) + reserved(4) + body(256) = 280.
    f = vsim.world()
    assert len(f) == 16 + 8 + 256
    magic, ver, typ, plen, seq = vsim.HDR.unpack_from(f, 0)
    assert magic == vsim.MAGIC and ver == vsim.VERSION
    assert typ == 4                      # CTL frame type
    assert plen == 8 + 256
    subtype, reserved = struct.unpack_from("<II", f, 16)
    assert subtype == vsim.CTL_SET_WORLD and reserved == 0


@pytest.mark.parametrize("builder,subtype", [
    (lambda: vsim.reset(), vsim.CTL_RESET),
    (lambda: vsim.world(), vsim.CTL_SET_WORLD),
    (lambda: vsim.testrig(True), vsim.CTL_SET_TESTRIG),
    (lambda: vsim.wind((1, 0, 0), 0.2), vsim.CTL_SET_WIND),
])
def test_builders_carry_their_subtype(builder, subtype):
    f = builder()
    assert len(f) == 280
    got, _ = struct.unpack_from("<II", f, 16)
    assert got == subtype


def test_world_mesh_counts_and_path():
    f = vsim.world_mesh("/tmp/x.bin", 17202, 5734, 2047, 0.3, True)
    nverts, ntris, nodes, flags, rest, plen = struct.unpack_from("<IIIIfI", f, 24)
    assert (nverts, ntris, nodes) == (17202, 5734, 2047)
    assert flags == 1                    # double-sided
    assert abs(rest - 0.3) < 1e-6
    path = f[24 + 24: 24 + 24 + plen].decode()
    assert path == "/tmp/x.bin"


def test_quat_to_euler_identity_and_axes():
    assert vsim.quat_to_euler(1, 0, 0, 0) == pytest.approx([0, 0, 0])
    # 90° about body X (roll): q = (cos45, sin45, 0, 0)
    import math
    r, p, y = vsim.quat_to_euler(math.cos(math.pi / 4), math.sin(math.pi / 4), 0, 0)
    assert r == pytest.approx(90.0, abs=1e-3)
    assert p == pytest.approx(0.0, abs=1e-3)


def test_parse_pose_roundtrip():
    # Build a POSE frame, confirm parse_pose recovers pos/quat/vel and trims buf.
    body = struct.pack(
        "<II3f4f3f3f4f4f3fffffff",
        0, 0,                       # two leading u32 (seq/flags slot)
        1.0, 2.0, -3.0,             # pos
        1.0, 0.0, 0.0, 0.0,         # quat
        0.1, 0.2, 0.3,              # vel
        0.0, 0.0, 0.0,              # omega
        10, 20, 30, 40,             # motor_omega
        0, 0, 0, 0,                 # (4f)
        0.0, 0.0, 0.0,              # wind
        0, 0, 0, 0, 0, 0)           # trailing 6f
    frame = vsim.vhdr(vsim.FRAME_POSE, vsim.POSE.size) + body
    buf = bytearray(frame)
    truth = vsim.parse_pose(buf)
    assert truth is not None
    assert truth["pos"] == pytest.approx((1.0, 2.0, -3.0))
    assert truth["quat"] == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert truth["vel"] == pytest.approx((0.1, 0.2, 0.3))
    assert truth["motor_omega"] == pytest.approx((10, 20, 30, 40))
    assert len(buf) == 0             # fully consumed


def test_parse_pose_partial_frame_kept():
    frame = vsim.vhdr(vsim.FRAME_POSE, vsim.POSE.size) + b"\x00" * vsim.POSE.size
    buf = bytearray(frame[:20])       # header + a few body bytes only
    assert vsim.parse_pose(buf) is None
    assert len(buf) == 20             # incomplete frame retained for next read
