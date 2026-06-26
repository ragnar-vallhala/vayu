#!/usr/bin/env python3
"""Rigorous real-airframe plant identification from the on-hardware logs.

Two independent handles on the plant P(s) = omega(s)/u(s) per axis:

  (A) low-frequency gain K  -- from the dedicated open-loop sysid (roll K=563,
      tau=20.9 ms; the gold-standard chirp). Pitch had no clean open-loop fit.

  (B) phase at the limit-cycle frequency -- the persistent oscillation is strong
      excitation. We measure P(j w_c) = Omega/U directly from (u=*_out,
      w=*_rate_curr) in each cycling window, and CROSS-CHECK it against the known
      controller via describing-function: a static-saturation limit cycle needs
      1 + N*Gc(j w_c)*P(j w_c) = 0 with N real, so
            angle(P) = -180 - angle(Gc).
      Gc is the full rate-PID (P,I + D-on-measurement w/ LPF) wrapped by the
      outer angle-P cascade, all KNOWN from the gains. Matching validates the
      measurement; the residual phase beyond a rigid-body integrator+actuator
      pole is the airframe's EXCESS lag -> an effective transport delay.

The headline: what extra lag must the real pitch plant have for the loop to
cross -180 deg at ~1.9 Hz (it limit-cycles there) -- a number the sim must
reproduce or it will never show the instability.
"""
import os, sys
import numpy as np
_CAMP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _CAMP)
import _pipeline as P
import _segment as S

W = 2 * np.pi

# --- controller gains active in these logs (compiled defaults, setup.json) ---
GAINS = {
    "pitch": dict(Kp=0.012, Ki=0.005, Kd=0.0001, td=0.004, Kpa=1.6898),
    "roll":  dict(Kp=0.012, Ki=0.00811, Kd=0.00025, td=0.004, Kpa=1.6898),
}
TAU_ACT = 0.0209   # roll actuator pole from the onhw sysid (s); best estimate


def Gc(w, g):
    """Open-loop controller transfer from the gyro rate omega back to u (the
    negative of u/omega). P,I on error (rate_sp - omega), D on measurement.
    rate_sp = Kpa*(theta_sp - theta), theta = integral(omega) => for regulation
    rate_sp = -Kpa/(jw) * omega."""
    s = 1j * w
    pi = g["Kp"] + g["Ki"] / s
    cascade = (g["Kpa"] / s + 1.0)
    d = g["Kd"] * s / (g["td"] * s + 1.0)
    return pi * cascade + d


def plant_phase_model(w, K, tau=TAU_ACT, Td=0.0):
    """Rigid-body integrator + first-order actuator + pure delay."""
    s = 1j * w
    return K / (s * (tau * s + 1.0)) * np.exp(-1j * w * Td)


LOGS = {k: os.path.join(_CAMP, v) for k, v in {
    "telem30s": "20260626-001343-telem30s/stream.bin",
    "pitch-verify": "20260625-233852-pitch-verify/pitch-verify.bin",
    "pid-openair": "20260626-014848-pid-openair/pid-openair.bin",
}.items()}


def measure(axis):
    uname = {"pitch": "pitch_out", "roll": "roll_out"}[axis]
    wname = {"pitch": "prate", "roll": "rrate"}[axis]
    g = GAINS[axis]
    rows = []
    for nm, f in LOGS.items():
        d = P.load(f); segs, _ = S.segment(d); ct = d["ct"]
        for sg in segs:
            if sg["kind"] not in ("SPOOL", "AIRBORNE") or sg["dur"] < 4:
                continue
            sl = slice(sg["i0"], sg["i1"]); t = ct[sl, 0]
            fs = len(t) / (t[-1] - t[0])
            if fs < 40:
                continue
            u = ct[sl, P.CT_I[uname]]; ww = ct[sl, P.CT_I[wname]]
            # only axes that actually oscillate carry usable excitation
            if ww.std() < 40:
                continue
            tu = np.linspace(t[0], t[-1], len(t))
            u = np.interp(tu, t, u); ww = np.interp(tu, t, ww)
            u -= u.mean(); ww -= ww.mean()
            win = np.hanning(len(u))
            U = np.fft.rfft(u * win); Om = np.fft.rfft(ww * win)
            fr = np.fft.rfftfreq(len(u), 1.0 / fs)
            k0 = np.argmax(np.abs(U) * (fr > 0.8))
            wc = W * fr[k0]
            Pmeas = Om[k0] / U[k0]
            ang = np.degrees(np.angle(Pmeas))
            # excess phase beyond a fast rigid-body model (integrator+21ms pole)
            ang_fast = np.degrees(np.angle(plant_phase_model(wc, 1.0)))
            excess = ang - ang_fast               # negative = extra lag
            Td = -np.radians(excess) / wc if wc > 0 else 0
            ang_pred = -180 - np.degrees(np.angle(Gc(wc, g)))
            rows.append((nm, fr[k0], abs(Pmeas), ang, ang_pred, ang_fast, Td * 1e3))
    return rows


for axis in ("pitch", "roll"):
    print(f"==== REAL {axis} plant @ limit cycle ====")
    print("   log              f_c     |P|   ang(P)  pred(ctrl)  fast-model  -> excess delay")
    rows = measure(axis)
    if not rows:
        print("   (no cycling windows)")
        continue
    Tds = []
    for nm, fc, mag, ang, pred, fast, Td in rows:
        Tds.append(Td)
        print(f"   {nm:14s} {fc:5.2f}Hz {mag:6.1f}  {ang:+6.0f}    {pred:+6.0f}     "
              f"{fast:+6.0f}      {Td:5.0f} ms")
    print(f"   --> median excess transport delay: {np.median(Tds):.0f} ms "
          f"(beyond integrator+{TAU_ACT*1e3:.0f}ms actuator)\n")

# where does the LOOP cross -180? (Gc*plant). sim has ~no excess delay -> stable.
print("Loop -180 crossover (Gc * plant), pitch, K=1381:")
for Td in (0.0, 0.05, 0.10, 0.124):
    ws = np.linspace(W * 0.3, W * 8, 4000)
    L = Gc(ws, GAINS["pitch"]) * plant_phase_model(ws, 1381.0, Td=Td)
    ph = np.unwrap(np.angle(L))
    idx = np.where(ph <= -np.pi)[0]
    fx = ws[idx[0]] / W if len(idx) else None
    print(f"   delay {Td*1e3:4.0f} ms -> crossover "
          + (f"{fx:.2f} Hz" if fx else "none in 0.3-8 Hz (stable)"))
