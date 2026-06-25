#!/usr/bin/env python3
"""Loop-isolation diagnostic: where does free-flight tracking break?

Commands a clean NORTH position step (0,0)->(step,0) at fixed altitude and logs,
at high rate, the FULL chain so each loop can be judged independently:

  guidance(my harness)  ->  RC emitter  ->  FC angle loop  ->  FC rate loop  -> physics
   cmd stick                RcChannels      ControlTrace        ControlTrace      truth
                                            angle_sp/curr       rate_sp/curr

A north step is driven by PITCH, so we track the pitch chain + N position.
Columns let you answer: did the FC receive the stick? did its angle loop track
its setpoint? did the estimate match truth? and did POSITION converge — or is
the outer (harness) loop the one that overshoots/wanders?
"""
import argparse
import math
import time

from vayu_headless import Pilot, SitlSession
from vayu_headless.transport.vsim import quat_to_euler

from _common import Recorder, gcs_conf


def run(step=10.0, alt=-5.0, settle=3.0, watch=14.0, csv="/tmp/diag_loops.csv",
        collision=True):
    conf = gcs_conf() if collision else None     # conf=None → no world mesh pushed
    rec = Recorder(csv, [
        "t", "tgt_n",
        "true_n", "true_e", "true_alt", "vN",
        "true_pitch", "est_pitch",
        "cmd_pitch_stick", "rc_recv_pitch",
        "ct_pitch_sp", "ct_pitch_curr", "ct_pitchrate_sp", "ct_pitchrate_curr",
        "ct_pitch_out"])
    with SitlSession(gcs=False, conf=conf) as s:
        pilot = Pilot(s, alt=alt)
        pilot.arm_takeoff(alt=alt)
        time.sleep(settle)
        t0 = time.time()
        pilot.goto([(step, 0.0)], alt=alt)         # NORTH position step
        while time.time() - t0 < watch:
            t = time.time() - t0
            tr = s.truth()
            ct = s.telem.get("ControlTrace")
            ae = s.telem.get("AttitudeEuler")
            rc = s.telem.get("RcChannels")
            if tr and ct:
                _, tp, _ = quat_to_euler(*tr["quat"])
                cmd_pitch = (s._rc[1] - 1500) / 500.0          # Pilot's emitted stick
                rc_pitch = (rc.chan[1] if rc and len(rc.chan) > 1 else 0)
                rec.add(t, step,
                        tr["pos"][0], tr["pos"][1], -tr["pos"][2], tr["vel"][0],
                        tp, getattr(ae, "pitch", 0.0),
                        cmd_pitch, rc_pitch,
                        ct.pitch_angle_sp, ct.pitch_angle_curr,
                        ct.pitch_rate_sp, ct.pitch_rate_curr, ct.pitch_out)
            time.sleep(0.02)
    rec.close()
    _analyse(rec.rows, step, csv)
    return rec.rows


def _analyse(rows, step, csv):
    if not rows:
        print("no data"); return
    # column indices
    N, VN, TP, EP = 2, 5, 6, 7
    SP, CUR = 10, 11
    ns = [r[N] for r in rows]
    settle_n = sum(r[N] for r in rows[-15:]) / 15.0          # last ~0.3s mean N
    overshoot = max(ns) - step
    # FC angle-loop tracking error (sp vs curr) over the run
    ang_err = sum(abs(r[SP] - r[CUR]) for r in rows) / len(rows)
    # estimator: est pitch vs true pitch
    est_err = sum(abs(r[EP] - r[TP]) for r in rows) / len(rows)
    print(f"\n=== diag: NORTH step to {step} m  ({csv}) ===")
    print(f"position:   reached max N={max(ns):+.1f} m, settled ~{settle_n:+.1f} m, "
          f"overshoot={overshoot:+.1f} m  (target {step})")
    print(f"FC angle:   mean |pitch_sp - pitch_curr| = {ang_err:.2f}  "
          f"(small ⇒ FC angle loop TRACKS its setpoint)")
    print(f"estimator:  mean |est_pitch - true_pitch| = {est_err:.2f}°  "
          f"(small ⇒ estimate matches truth)")
    print("verdict: if FC-angle + estimator errors are small but position "
          "overshoots/doesn't settle → the OUTER (harness) loop is the culprit.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=10.0)
    ap.add_argument("--alt", type=float, default=-5.0)
    ap.add_argument("--watch", type=float, default=14.0)
    ap.add_argument("--csv", default="/tmp/diag_loops.csv")
    ap.add_argument("--no-collision", action="store_true",
                    help="don't push the world mesh (isolate guidance from collision)")
    a = ap.parse_args()
    run(step=a.step, alt=a.alt, watch=a.watch, csv=a.csv,
        collision=not a.no_collision)
