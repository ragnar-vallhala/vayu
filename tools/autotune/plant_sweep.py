#!/usr/bin/env python3
"""Limit-cycle robustness sweep against the in-process SITL.

Builds a vsim_ctl_geometry_t from a .vveh, then runs VAYU_RTOS_SCENARIO=disturb
across gains x actuator transport delay and reports the envelope decay ratio.

    python3 tools/autotune/plant_sweep.py sim/vsim/racer5.vveh

READ THE METRICS THE RIGHT WAY:
  decay (late_rms/early_rms) is the STABILITY metric -- >~0.5 means the rate
  never settled, i.e. a sustained limit cycle. That is what this is for.
  peak is NOT a stability metric: it is how far the craft departs from the 21 deg
  step command, so a SOFTER loop shows a BIGGER peak while still ringing down.
  Do not read a rising peak as instability.

The geometry struct is built directly, never sliced out of a ctl frame -- the
frame carries a trailer as well as a header, and a mis-sliced struct silently
feeds vsim max_omega=0 (no thrust, no torque), which looks exactly like "the
gains do nothing".
"""
import json, os, re, struct, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BIN = os.environ.get("VAYU_SITL_RTOS_BIN",
                     os.path.join(ROOT, "build_sitl_rtos", "vayu_sitl_rtos"))
_RE = re.compile(r"peak=(\S+) early_rms=(\S+) late_rms=(\S+) decay=(\S+)")


def geom_from_vveh(vveh_path, out_path, tau=0.010):
    """Serialize vsim_ctl_geometry_t: mass, 3x3 inertia, then 4 motors
    {pos[3], axis[3], spin, k_thrust, k_moment, max_omega, tau}. 216 bytes."""
    v = json.load(open(vveh_path))
    body = struct.pack("<f", float(v["mass"]))
    body += struct.pack("<9f", *[float(x) for x in v["inertia"]])
    for m in v["motors"]:
        body += struct.pack("<3f3f5f",
                            m["pos"]["x"], m["pos"]["y"], m["pos"]["z"],
                            m["axis"]["x"], m["axis"]["y"], m["axis"]["z"],
                            float(m["spin"]), m["k_thrust"], m["k_moment"],
                            m["max_omega"], m.get("tau", tau))
    assert len(body) == 216, f"geometry must be 216 B, got {len(body)}"
    open(out_path, "wb").write(body)
    return out_path


def run(geom, axis=1, delay_ms=0, stall=0.0, **gains):
    env = dict(os.environ, VAYU_RTOS_SCENARIO="disturb", VAYU_RTOS_GEOMETRY=geom,
               VAYU_RTOS_DIST_AXIS=str(axis))
    if delay_ms:
        env["VSIM_MOTOR_DELAY_MS"] = str(delay_ms)
    if stall:
        env["VSIM_STALL_DUTY"] = str(stall)
    for k, val in gains.items():
        env["VAYU_" + k.upper()] = repr(float(val))
    p = subprocess.run([BIN], env=env, capture_output=True, text=True, timeout=120)
    m = _RE.search(p.stdout + p.stderr)
    if not m:
        raise RuntimeError(f"no disturb line\n{p.stderr[-500:]}")
    peak, early, late, decay = (float(x) for x in m.groups())
    return dict(peak=peak, early_rms=early, late_rms=late, decay=decay)


def main():
    if not os.path.exists(BIN):
        print(f"build it first: {BIN} missing (-DVAYU_SITL_RTOS_BUILD=ON)")
        return 2
    vveh = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "sim", "vsim", "racer5.vveh")
    geom = geom_from_vveh(vveh, os.path.splitext(vveh)[0] + ".geom")
    print(f"vehicle {os.path.relpath(vveh, ROOT)}\n")
    print(f"{'delay':>7} {'rate_kp':>9} {'decay':>8} {'late_rms':>9}  verdict")
    for delay in (0, 40, 80):
        for kp in (0.0025, 0.0012, 0.0006):
            r = run(geom, delay_ms=delay, stall=0.05, rate_kp=kp)
            verdict = "SUSTAINED limit cycle" if r["decay"] > 0.5 else "rings down"
            print(f"{delay:5d}ms {kp:9.4f} {r['decay']:8.3f} {r['late_rms']:9.2f}  {verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
