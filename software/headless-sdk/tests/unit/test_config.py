"""Unit tests for GCS .conf parsing + geometry framing (no I/O beyond a tmp file)."""
import struct

from vayu_headless import config
from vayu_headless.transport import vsim

SAMPLE_CONF = """\
[General]
foo=bar

[simulator]
geometry\\mass=1.25
geometry\\I0=0.01
geometry\\m0_px=0.1
world\\gravity=9.81
world\\ground_z=0
world\\worldMeshPath=/tmp/course.glb
world\\worldMeshDoubleSided=true
ignored=skip
"""


def test_read_gcs_conf_splits_sections(tmp_path):
    p = tmp_path / "Vayu GCS.conf"
    p.write_text(SAMPLE_CONF)
    g, w = config.read_gcs_conf(str(p))
    assert g["mass"] == "1.25"
    assert g["I0"] == "0.01"
    assert g["m0_px"] == "0.1"
    assert w["gravity"] == "9.81"
    assert w["worldMeshPath"] == "/tmp/course.glb"
    assert "ignored" not in g and "ignored" not in w


def test_read_gcs_conf_missing_file_is_empty():
    g, w = config.read_gcs_conf("/no/such/file.conf")
    assert g == {} and w == {}


def test_geometry_frame_uses_mass_and_forces_thrust_axis():
    g = {"mass": "2.0"}
    f = config.geometry_frame(g)
    subtype, _ = struct.unpack_from("<II", f, 16)
    assert subtype == vsim.CTL_SET_GEOMETRY
    # body starts after hdr(16)+subtype(4)+reserved(4) = offset 24: first float = mass
    mass = struct.unpack_from("<f", f, 24)[0]
    assert mass == 2.0
    # first motor thrust axis is forced to (0,0,-1): mass(4) + inertia(9*4=36) = 40,
    # then m0 pos(3f=12) -> axis at offset 24+40+12 = 76
    ax, ay, az = struct.unpack_from("<3f", f, 76)
    assert (ax, ay, az) == (0.0, 0.0, -1.0)
