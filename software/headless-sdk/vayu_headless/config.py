"""GCS config parsing + vehicle-geometry framing.

Reads the Navigator's QSettings .conf so a headless run flies the SAME
vehicle/world (vveh/vworld) the operator has loaded. Carved verbatim from the
original sitl_lab.py.
"""
import struct

from .transport import vsim


def read_gcs_conf(path):
    r"""Parse the [simulator] geometry\* and world\* keys from the GCS's
    QSettings .conf. Returns (geometry_dict, world_dict)."""
    g, w = {}, {}
    sect = None
    try:
        for ln in open(path):
            ln = ln.strip()
            if ln.startswith("[") and ln.endswith("]"):
                sect = ln[1:-1]
                continue
            if sect != "simulator" or "=" not in ln:
                continue
            k, v = ln.split("=", 1)
            if k.startswith("geometry\\"):
                g[k[len("geometry\\"):]] = v
            elif k.startswith("world\\"):
                w[k[len("world\\"):]] = v
    except OSError:
        pass
    return g, w


def geometry_frame(g):
    """Pack vsim_ctl_geometry_t from the parsed geometry dict (54 floats)."""
    f = lambda k, d=0.0: float(g.get(k, d))   # noqa: E731
    body = struct.pack("<f", f("mass", 1.0))
    body += struct.pack("<9f", *[f("I%d" % i) for i in range(9)])
    for i in range(4):
        p = "m%d_" % i
        body += struct.pack("<3f", f(p + "px"), f(p + "py"), f(p + "pz"))
        # Thrust axis: vsim lift is body -Z (F = axis*thrust, motor_model.cpp).
        # The GCS conf stores az in a frame where +1 is "up", which is -Z in
        # vsim's NED — pushing it verbatim thrusts DOWNWARD and jams the craft
        # into the ground. A standard quad's rotors all lift up, so force -Z.
        body += struct.pack("<3f", 0.0, 0.0, -1.0)
        body += struct.pack("<5f", f(p + "spin", 1.0), f(p + "kt", 1.522e-5),
                            f(p + "km", 2.44e-7), f(p + "wmax", 1200.0),
                            f(p + "tau", 0.0125))
    return vsim.ctl(vsim.CTL_SET_GEOMETRY, body)
