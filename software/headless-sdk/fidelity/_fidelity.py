"""Shared helpers for the control-loop fidelity suite.

Two jobs:
  1. verify_frame() — PROVE the plant is flying the loaded frame's geometry
     (motor positions from the GCS conf / a .vveh), NOT vsim's compiled-in X3
     defaults. It reads vsim_d's own geometry-set echo and asserts the applied
     motor positions match what the harness pushed.
  2. metric helpers + a per-maneuver 0-100 fidelity score, written as a JSON
     sidecar next to each CSV so rate_fidelity.py can aggregate the suite.

Importing this module forces SITL_LAB_DEBUG=1 so vsim_d's stderr (the geometry
echo) is captured to /tmp/vsim_d.err.
"""
import json
import math
import os
import re
import time

# vsim_d stderr (geometry echo) lands here when SITL_LAB_DEBUG is set; force it
# on for every fidelity run so verify_frame() has something to read.
os.environ.setdefault("SITL_LAB_DEBUG", "1")
VSIM_ERR = "/tmp/vsim_d.err"

from vayu_headless.paths import gcs_conf_default  # noqa: E402


def gcs_conf():
    """GCS .conf path if present (the operator's loaded vveh/vworld), else None."""
    c = gcs_conf_default()
    return c if os.path.exists(c) else None


def session_kwargs(vveh=None):
    """Frame source for SitlSession: an explicit --vveh wins, else the GCS conf
    (the default selected frame). Keeps every maneuver consistent."""
    return {"vveh": vveh} if vveh else {"conf": gcs_conf()}

# vsim_types.h MotorParams default M0 pos — the "wrong frame" sentinel.
_VSIM_DEFAULT_M0 = (0.13, 0.22)
OUT_DIR = os.path.join(os.path.dirname(__file__), "out")

_MOTOR_RE = re.compile(
    r"motor(\d)\s+pos=\(\s*([-\d.eE]+),\s*([-\d.eE]+),\s*([-\d.eE]+)\)")


def _read_vsim_motor_echo(err_path=VSIM_ERR):
    """Return the LAST set of 4 echoed motor positions vsim_d applied, or None."""
    try:
        txt = open(err_path).read()
    except OSError:
        return None
    found = {}
    for m in _MOTOR_RE.finditer(txt):
        found[int(m.group(1))] = (float(m.group(2)), float(m.group(3)),
                                  float(m.group(4)))
    if len(found) < 4:
        return None
    return [found[i] for i in range(4)]


def verify_frame(sess, label="frame", tol=2e-3, wait=2.0):
    """Assert vsim_d applied the geometry the harness pushed (sess.geometry),
    not the compiled-in defaults. Raises AssertionError on mismatch. Returns a
    dict describing the verified frame (for the scorecard)."""
    expected = getattr(sess, "geometry", {}) or {}
    if not expected:
        raise AssertionError(
            "verify_frame: NO geometry was pushed — vsim_d is running its "
            "compiled-in X3 defaults. Pass conf=gcs_conf() or vveh=<file>.")
    exp = [(float(expected["m%d_px" % i]), float(expected["m%d_py" % i]),
            float(expected["m%d_pz" % i])) for i in range(4)]

    # vsim echoes geometry at startup; poll the err file briefly for it.
    echo, t0 = None, time.time()
    while time.time() - t0 < wait:
        echo = _read_vsim_motor_echo()
        if echo:
            break
        time.sleep(0.1)
    if not echo:
        raise AssertionError(
            f"verify_frame: vsim_d printed no geometry echo in {VSIM_ERR}; "
            "cannot confirm the frame (is SITL_LAB_DEBUG set?).")

    worst = max(math.hypot(echo[i][0] - exp[i][0], echo[i][1] - exp[i][1])
                for i in range(4))
    if worst > tol:
        lines = "\n".join(
            f"    M{i}: pushed=({exp[i][0]:+.4f},{exp[i][1]:+.4f}) "
            f"applied=({echo[i][0]:+.4f},{echo[i][1]:+.4f})" for i in range(4))
        raise AssertionError(
            f"verify_frame: applied motor geometry != pushed (worst "
            f"{worst*1000:.1f} mm):\n{lines}")

    is_default = (abs(abs(echo[0][0]) - _VSIM_DEFAULT_M0[0]) < 1e-3 and
                  abs(abs(echo[0][1]) - _VSIM_DEFAULT_M0[1]) < 1e-3)
    span = max(math.hypot(p[0], p[1]) for p in echo) * 2 * 1000  # motor diag mm
    info = {"verified": True, "on_defaults": is_default,
            "mass": float(expected["mass"]),
            "motor_diag_mm": round(span, 1),
            "m0": [round(echo[0][0], 4), round(echo[0][1], 4)]}
    flag = "  <-- WARNING: looks like vsim DEFAULTS" if is_default else ""
    print(f"  [frame] VERIFIED {label}: mass={info['mass']:.3f} kg, "
          f"motor-diag~{info['motor_diag_mm']:.0f} mm, "
          f"M0=({info['m0'][0]:+.3f},{info['m0'][1]:+.3f}) m{flag}")
    return info


# AttitudeEuler is RADIANS on the wire (navlink_tx.c); truth quat_to_euler is
# degrees. Convert est to degrees so the two are directly comparable.
EST_RAD2DEG = 57.29578


def read_att(sess, quat_to_euler):
    """(truth_dict, (true_r,p,y deg), (est_r,p,y deg)) for the current sample."""
    tr = sess.truth()
    ae = sess.telem.get("AttitudeEuler")
    tru = quat_to_euler(*tr["quat"]) if tr else (0.0, 0.0, 0.0)
    est = (ae.roll * EST_RAD2DEG, ae.pitch * EST_RAD2DEG,
           ae.yaw * EST_RAD2DEG) if ae else (0.0, 0.0, 0.0)
    return tr, tru, est


# ---------------------------------------------------------------------------
# Metric helpers
# ---------------------------------------------------------------------------
def step_metrics(ts, sig, target, t_step):
    """First-order/2nd-order step descriptors for a signal stepping 0->target
    at t_step. Returns rise (10-90%), overshoot %, settle (within 5%) and
    steady-state error (last 20% mean vs target). Sign-agnostic."""
    seg = [(t, v) for t, v in zip(ts, sig) if t >= t_step]
    if not seg or abs(target) < 1e-6:
        return {}
    tt = [t - t_step for t, _ in seg]
    vv = [v for _, v in seg]
    s = 1.0 if target >= 0 else -1.0
    g = [s * v for v in vv]
    gt = s * target
    n = len(g)
    ss = sum(g[int(n * 0.8):]) / max(1, n - int(n * 0.8))

    def _cross(frac):
        thr = frac * gt
        for t, v in zip(tt, g):
            if v >= thr:
                return t
        return None
    t10, t90 = _cross(0.1), _cross(0.9)
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None
    peak = max(g)
    overshoot = max(0.0, (peak - gt) / gt * 100.0)
    settle = None
    band = 0.05 * gt
    for i in range(n):
        if all(abs(g[j] - gt) <= band for j in range(i, n)):
            settle = tt[i]
            break
    return {"rise_s": rise, "overshoot_pct": overshoot, "settle_s": settle,
            "ss_value": s * ss, "ss_err": gt - ss}


def hover_metrics(rows, idx):
    """rows: list of tuples; idx maps logical->column. Drift = max horizontal
    distance from origin; alt RMS error vs hold; attitude RMS (roll/pitch)."""
    n, e, alt = idx["n"], idx["e"], idx["alt"]
    hr, pr = idx["true_roll"], idx["true_pitch"]
    hold_alt = sum(r[alt] for r in rows[:max(1, len(rows)//10)]) / \
        max(1, len(rows)//10)
    drift = max(math.hypot(r[n], r[e]) for r in rows)
    alt_rms = math.sqrt(sum((r[alt] - hold_alt) ** 2 for r in rows) / len(rows))
    att_rms = math.sqrt(sum(r[hr]**2 + r[pr]**2 for r in rows) / len(rows))
    return {"hold_alt_m": hold_alt, "max_drift_m": drift,
            "alt_rms_m": alt_rms, "att_rms_deg": att_rms}


def est_truth_gap(rows, est_i, true_i):
    """Estimator fidelity: mean abs (est-true) and the under-read ratio
    (mean |true| / mean |est|) over samples where |true| is meaningful."""
    big = [r for r in rows if abs(r[true_i]) > 2.0]   # >2 deg of real tilt
    if not big:
        return {"mae_deg": None, "underread_ratio": None, "n": 0}
    mae = sum(abs(r[est_i] - r[true_i]) for r in big) / len(big)
    mt = sum(abs(r[true_i]) for r in big) / len(big)
    me = sum(abs(r[est_i]) for r in big) / len(big)
    ratio = (mt / me) if me > 1e-3 else None
    return {"mae_deg": mae, "underread_ratio": ratio, "n": len(big)}


# ---------------------------------------------------------------------------
# Scoring + sidecar I/O
# ---------------------------------------------------------------------------
def write_metrics(name, metrics, frame=None):
    os.makedirs(OUT_DIR, exist_ok=True)
    payload = {"maneuver": name, "metrics": metrics, "frame": frame or {}}
    payload["score"] = fidelity_score(name, metrics)
    with open(os.path.join(OUT_DIR, name + ".json"), "w") as f:
        json.dump(payload, f, indent=2)
    return payload["score"]


def _clip(x):
    return max(0.0, min(100.0, x))


def fidelity_score(name, m):
    """Heuristic 0-100 per-maneuver score. Higher = the SITL control loop
    tracks the command faithfully. Tuned to flag the known estimator under-read
    and outer-loop drift, not to be a precise grade."""
    if name.startswith("step") or name.startswith("yaw"):
        # attitude/heading step: reward fast clean tracking + low SS error.
        rise = m.get("rise_s") or 1.0
        ov = m.get("overshoot_pct") or 0.0
        sse = abs(m.get("ss_err") or 0.0)
        tgt = abs(m.get("target") or 1.0)
        return round(_clip(100 - 25 * rise - 0.5 * ov - 40 * (sse / tgt)), 1)
    if name == "hover":
        return round(_clip(100 - 8 * m.get("max_drift_m", 0)
                           - 20 * m.get("alt_rms_m", 0)
                           - 3 * m.get("att_rms_deg", 0)), 1)
    if name == "throttle":
        sse = abs(m.get("ss_err") or 0.0)
        return round(_clip(100 - 30 * sse - 0.5 * (m.get("overshoot_pct") or 0)), 1)
    if name == "tracking":
        return round(_clip(100 - 6 * m.get("mean_err_m", 0)
                           - 2 * m.get("max_err_m", 0)), 1)
    return 0.0
