#!/usr/bin/env python3
"""Run the headless SITL (real FC logic + vsim_d physics) with the GCS conf
geometry and collect fresh sim data for real-vs-sim parity:

  1. roll + pitch CHIRP on the tuning rig (translation pinned) — logs the
     controller effort u (ControlTrace.{roll,pitch}_out) against the TRUE body
     rate (truth omega) so we can fit the sim plant gain K = wdot/u and compare
     to the real on-hardware fit (roll K=563, pitch K=1381 deg/s^2 per u).
  2. a free HOVER — logs throttle, the four motor commands, pitch_out and the
     pitch rate so we can compare hover throttle / motor differential / output
     saturation to the real logs.

Run with the package venv (has the SITL deps):
    ./.venv/bin/python <this>           # from navigator/headless-sdk
CSVs land next to this file. Fitting is done separately with system numpy.
"""
import os, sys, math, time, csv

HD = os.environ.get("HEADLESS_DIR",
                    os.path.expanduser("~/Documents/Drone/stack/vayu/navigator/headless-sdk"))
sys.path.insert(0, HD)
sys.path.insert(0, os.path.join(HD, "examples"))
from vayu_headless import SitlSession                  # noqa: E402
from _common import gcs_conf, arm_on_rig, quat_to_euler  # noqa: E402

OUT = os.path.dirname(os.path.abspath(__file__))


def chirp(axis, amp=0.3, f0=0.3, f1=6.0, secs=22.0, thr=0.5):
    uattr = {"roll": "roll_out", "pitch": "pitch_out"}[axis]
    widx = {"roll": 0, "pitch": 1}[axis]
    rows = []
    with SitlSession(rig=True, conf=gcs_conf()) as s:
        arm_on_rig(s, thr=thr)
        t0 = time.time()
        while True:
            t = time.time() - t0
            if t >= secs:
                break
            phase = 2 * math.pi * (f0 + (f1 - f0) * (t / secs) / 2.0) * t
            cmd = amp * math.sin(phase)
            s.stick(thr=thr, **{axis: cmd})
            s.set_rc(swa=2000)
            ct = s.telem.get("ControlTrace"); tr = s.truth()
            if ct and tr:
                rows.append((t, getattr(ct, uattr), tr["omega"][widx],
                             getattr(ct, "thro_out", thr),
                             tr["motor_omega"][0] if tr.get("motor_omega") else 0.0))
            time.sleep(0.01)
    p = os.path.join(OUT, f"chirp_{axis}.csv")
    with open(p, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t", "u", "omega_radps", "thro", "m0_omega"])
        w.writerows(rows)
    print(f"chirp {axis}: {len(rows)} rows -> {p}")


def hover(secs=18.0):
    """Free flight: take off and hold, log the control+actuator pipeline."""
    rows = []
    with SitlSession(rig=False, conf=gcs_conf()) as s:
        # arm + climb to a hold using the autopilot's RC sticks only
        from vayu_headless import Pilot
        pilot = Pilot(s, alt=-4.0)
        pilot.arm_takeoff()
        t0 = time.time()
        while time.time() - t0 < secs:
            t = time.time() - t0
            ct = s.telem.get("ControlTrace"); mt = s.telem.get("MotorTelemetry")
            tr = s.truth()
            if ct and mt and tr:
                cmd = list(mt.cmd)[:4]
                rows.append((t, getattr(ct, "thro_out", 0), *cmd,
                             getattr(ct, "pitch_out", 0), getattr(ct, "roll_out", 0),
                             getattr(ct, "pitch_rate_curr", 0),
                             getattr(ct, "roll_rate_curr", 0),
                             -tr["pos"][2]))
            time.sleep(0.02)
        pilot.land()
    p = os.path.join(OUT, "hover.csv")
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "thro", "m0", "m1", "m2", "m3", "pitch_out", "roll_out",
                    "prate", "rrate", "alt_up"])
        w.writerows(rows)
    print(f"hover: {len(rows)} rows -> {p}")


def tone(axis, f=0.8, amp=0.5, secs=18.0, thr=0.5):
    """Single strong tone — high SNR at one frequency for a clean K = 2*pi*f*|w/u|."""
    uattr = {"roll": "roll_out", "pitch": "pitch_out"}[axis]
    widx = {"roll": 0, "pitch": 1}[axis]
    rows = []
    with SitlSession(rig=True, conf=gcs_conf()) as s:
        arm_on_rig(s, thr=thr)
        t0 = time.time()
        while time.time() - t0 < secs:
            t = time.time() - t0
            cmd = amp * math.sin(2 * math.pi * f * t)
            s.stick(thr=thr, **{axis: cmd})
            s.set_rc(swa=2000)
            ct = s.telem.get("ControlTrace"); tr = s.truth()
            if ct and tr:
                rows.append((t, getattr(ct, uattr), tr["omega"][widx],
                             getattr(ct, "thro_out", thr),
                             tr["motor_omega"][0] if tr.get("motor_omega") else 0.0))
            time.sleep(0.01)
    p = os.path.join(OUT, f"tone_{axis}.csv")
    with open(p, "w", newline="") as f2:
        w = csv.writer(f2); w.writerow(["t", "u", "omega_radps", "thro", "m0_omega"])
        w.writerows(rows)
    print(f"tone {axis} @{f}Hz: {len(rows)} rows -> {p}")


if __name__ == "__main__":
    which = sys.argv[1:] or ["roll", "pitch", "hover"]
    if "roll" in which: chirp("roll")
    if "pitch" in which: chirp("pitch")
    if "tone_roll" in which: tone("roll")
    if "tone_pitch" in which: tone("pitch")
    if "hover" in which: hover()
