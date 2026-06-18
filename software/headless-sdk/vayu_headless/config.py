"""GCS config parsing + vehicle-geometry framing.

Reads the Navigator's QSettings .conf so a headless run flies the SAME
vehicle/world (vveh/vworld) the operator has loaded. Carved verbatim from the
original sitl_lab.py.
"""
import json
import struct

from .transport import vsim


def geometry_from_vveh(path):
    r"""Parse a .vveh vehicle file into the same geometry dict shape
    read_gcs_conf() produces (keys: mass, I0..I8, comX/Y/Z, m{i}_px/py/pz,
    m{i}_spin/kt/km/wmax/tau). Lets a fidelity run pin the frame straight from
    the authoritative .vveh file instead of the (possibly stale) GCS conf."""
    d = json.load(open(path))
    g = {"mass": d.get("mass", 1.0)}
    for i, v in enumerate(d.get("inertia", [0.0] * 9)):
        g["I%d" % i] = v
    com = d.get("com", {})
    g["comX"], g["comY"], g["comZ"] = com.get("x", 0.0), com.get("y", 0.0), \
        com.get("z", 0.0)
    for i, m in enumerate(d.get("motors", [])[:4]):
        p, a = m.get("pos", {}), m.get("axis", {})
        g["m%d_px" % i], g["m%d_py" % i], g["m%d_pz" % i] = \
            p.get("x", 0.0), p.get("y", 0.0), p.get("z", 0.0)
        g["m%d_ax" % i], g["m%d_ay" % i], g["m%d_az" % i] = \
            a.get("x", 0.0), a.get("y", 0.0), a.get("z", 1.0)
        g["m%d_spin" % i] = m.get("spin", 1)
        g["m%d_kt" % i] = m.get("k_thrust", 1.522e-5)
        g["m%d_km" % i] = m.get("k_moment", 2.44e-7)
        g["m%d_wmax" % i] = m.get("max_omega", 1200.0)
        g["m%d_tau" % i] = m.get("tau", 0.0125)
    return g


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
