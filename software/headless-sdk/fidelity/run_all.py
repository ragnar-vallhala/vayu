#!/usr/bin/env python3
"""Run the whole control-loop fidelity suite, then print the scorecard.

Each maneuver verifies the frame (motor geometry == the loaded vveh, not vsim
defaults) before flying, records CSV + a metrics sidecar under fidelity/out/,
and rate_fidelity.py aggregates them.

    PYTHONPATH=.:examples python fidelity/run_all.py [--vveh path]
"""
import argparse

import attitude_steps
import hover
import rate_fidelity
import throttle_step
import tracking
import yaw_rotate


def main(vveh=None):
    print(">> attitude step: roll");  attitude_steps.run("roll", vveh=vveh)
    print(">> attitude step: pitch"); attitude_steps.run("pitch", vveh=vveh)
    print(">> yaw rate");             yaw_rotate.run(vveh=vveh)
    print(">> hover hold");           hover.run(vveh=vveh)
    print(">> altitude step");        throttle_step.run(vveh=vveh)
    print(">> position tracking");    tracking.run(vveh=vveh)
    print()
    rate_fidelity.main()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--vveh", default="", help="frame .vveh to pin (default: GCS conf)")
    a = ap.parse_args()
    main(vveh=a.vveh or None)
