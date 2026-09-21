#!/usr/bin/env python3
# Copyright (C) 2026 NAVRobotec Pvt Ltd
# Author: Ragnar Vallhala
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""One-shot rate-loop tune session: throttle-triggered chirp -> fit -> apply PID.

Orchestrates the three sysid tools so the single-owner UDP port (14555) is held
ONLY while talking to the FC and is RELEASED during the offline fit. Each phase
is a separate process, so the port hand-off is enforced by process exit, not by
a long-lived socket:

  phase 1  CAPTURE  sysid_excite.py --trigger-throttle T
                    -> waits for CONTROL_TRACE thro_out >= T (held ~0.3 s),
                       fires the tapered chirp, pulls the FC RAM capture to CSV,
                       then EXITS  ->  frees :14555
  phase 2  FIT      sysid_fit.py --csv ...           (offline, NO port)
                    -> plant fit omega/u = K/(s(tau s+1)) + analytic gain design;
                       its R^2 gates whether we are allowed to apply
  phase 3  APPLY    sysid_fit.py --csv ... --apply    (reacquires :14555)
                    -> pushes rate+angle CMD_SET_PID for the axis

SAFETY: throttle >= 0.4 means the FC is ARMED with props live. Excite on the rig
(raised inertia) before free flight, start at a low --amp, and keep the abort in
reach (`python3 sysid_excite.py --abort`). Default is DRY (capture+fit+print, NO
apply); pass --apply to actually push gains, and --yes to skip the confirm.

  # diagnosed pitch divergence -> identify + review pitch, do NOT push yet:
  python3 tools/sysid/tune_session.py --axis pitch --trigger 0.4 --amp 20 --dur 6

  # ... happy with K / R^2 -> push the designed gains:
  python3 tools/sysid/tune_session.py --axis pitch --trigger 0.4 --amp 20 --apply
"""
import argparse
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
EXCITE = os.path.join(HERE, "sysid_excite.py")
FIT = os.path.join(HERE, "sysid_fit.py")
PY = sys.executable


def run(cmd):
    """Run a sub-tool, stream its output live, return (rc, captured_text)."""
    print(f"\n$ {' '.join(cmd)}\n", flush=True)
    out_lines = []
    env = dict(os.environ, PYTHONUNBUFFERED="1")  # children print live, not on exit
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, bufsize=1, env=env)
    for line in p.stdout:
        sys.stdout.write(line)
        sys.stdout.flush()
        out_lines.append(line)
    p.wait()
    return p.returncode, "".join(out_lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--axis", choices=["roll", "pitch", "yaw"], default="pitch")
    ap.add_argument("--trigger", type=float, default=0.4,
                    help="fire the chirp once thro_out >= this (held ~0.3 s)")
    ap.add_argument("--trigger-timeout", type=float, default=30.0)
    ap.add_argument("--f0", type=float, default=0.5)
    ap.add_argument("--f1", type=float, default=12.0)
    ap.add_argument("--amp", type=float, default=20.0, help="chirp amplitude, deg/s")
    ap.add_argument("--dur", type=float, default=6.0)
    ap.add_argument("--csv", default="/tmp/sysid_dump.csv")
    ap.add_argument("--port", type=int, default=14555)
    ap.add_argument("--min-r2", type=float, default=0.5,
                    help="refuse to apply if the plant fit R^2 is below this")
    ap.add_argument("--apply", action="store_true",
                    help="push the designed gains (default: dry / review only)")
    ap.add_argument("--yes", action="store_true", help="skip the pre-apply confirm")
    args = ap.parse_args()

    # ---- phase 1: capture (holds :port, then frees it on exit) -------------
    rc, _ = run([PY, EXCITE, "--port", str(args.port), "--axis", args.axis,
                 "--f0", str(args.f0), "--f1", str(args.f1),
                 "--amp", str(args.amp), "--dur", str(args.dur),
                 "--trigger-throttle", str(args.trigger),
                 "--trigger-timeout", str(args.trigger_timeout),
                 "--csv", args.csv])
    if rc != 0:
        print(f"\n[tune] capture failed (rc={rc}); no fit, no apply. "
              f"If the FC is armed, disarm now.")
        return rc
    if not os.path.exists(args.csv):
        print("[tune] no CSV produced; aborting.")
        return 1

    # ---- phase 2: offline fit (no port) -----------------------------------
    rc, out = run([PY, FIT, "--csv", args.csv, "--axis", args.axis])
    if rc != 0:
        print(f"[tune] fit failed (rc={rc}); not applying.")
        return rc
    m = re.search(r"R\^2=([0-9.]+)", out)
    r2 = float(m.group(1)) if m else 0.0
    print(f"\n[tune] plant fit R^2 = {r2:.3f} (min to apply = {args.min_r2:.2f})")

    if not args.apply:
        print("[tune] DRY run — designed gains shown above, nothing pushed. "
              "Re-run with --apply to push.")
        return 0
    if r2 < args.min_r2:
        print(f"[tune] R^2 below {args.min_r2:.2f} -> fit is unreliable, REFUSING "
              f"to apply. Recapture (more amplitude / cleaner run) and retry.")
        return 1
    if not args.yes:
        ans = input(f"[tune] apply designed {args.axis} gains to the FC? [y/N] ")
        if ans.strip().lower() not in ("y", "yes"):
            print("[tune] not applying.")
            return 0

    # ---- phase 3: reacquire port + push CMD_SET_PID -----------------------
    rc, _ = run([PY, FIT, "--csv", args.csv, "--axis", args.axis,
                 "--port", str(args.port), "--apply"])
    if rc != 0:
        print(f"[tune] apply failed (rc={rc}).")
        return rc
    print(f"\n[tune] done: {args.axis} rate+angle gains pushed. NOT persisted across "
          f"battery cycles — re-apply after each boot.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
