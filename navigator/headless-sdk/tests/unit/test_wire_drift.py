"""Phase 5 drift guard: the Python vsim wire layer must match vsim_proto.h.

Parses the C static_assert sizes and asserts the Python struct formats agree, so
a change to the C protocol fails loudly here instead of silently corrupting
frames at runtime.
"""
import os
import re
import struct

import pytest

from vayu_headless._repo import repo_root
from vayu_headless.transport import vsim

PROTO = os.path.join(repo_root(), "sim", "vsim", "include", "vsim_proto.h")


def _proto_sizes():
    txt = open(PROTO).read()
    sizes = {}
    for m in re.finditer(r"static_assert\(sizeof\((\w+)\)\s*==\s*([0-9 +]+)", txt):
        sizes[m.group(1)] = eval(m.group(2))   # noqa: S307 — trusted, digits/+ only
    return sizes


@pytest.fixture(scope="module")
def sizes():
    s = _proto_sizes()
    assert s, "no static_assert sizes parsed from vsim_proto.h"
    return s


def test_header_and_pose_match(sizes):
    assert sizes["vsim_hdr_t"] == vsim.HDR.size == 16
    assert sizes["vsim_pose_frame_t"] == vsim.HDR.size + vsim.POSE.size


def test_ctl_frame_total_matches(sizes):
    # ctl frame on the wire == hdr + subtype(4) + reserved(4) + body(256)
    assert sizes["vsim_ctl_frame_t"] == len(vsim.world())


def test_ctl_bodies_match_python_formats(sizes):
    assert sizes["vsim_ctl_world_t"] == struct.calcsize("<7f")        # gravity..damp
    assert sizes["vsim_ctl_wind_t"] == struct.calcsize("<3f4fi")      # steady..enable
    geometry_fmt = "<f" + "9f" + "3f3f5f" * 4                          # mass+I+4 motors
    assert sizes["vsim_ctl_geometry_t"] == struct.calcsize(geometry_fmt)
