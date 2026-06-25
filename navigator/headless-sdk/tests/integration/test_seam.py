"""Seam guard: the FC process must see ONLY sensors + RC, never the truth.

SITL is meaningful only because the firmware is exercised honestly — sensors +
RC in, PWM + telemetry out. The ground-truth pose FIFO is the *operator's*
(Pilot) channel; if the FC ever opened it, the suite would be validating a shim
instead of the shipped firmware. This pins that contract at runtime by reading
the FC process's open file descriptors via /proc.

See memory/sitl-seam-contract.md.
"""
import glob
import os
import time

import pytest


def _fc_open_paths(pid):
    """Realpath of every file/FIFO/pty the process has open."""
    out = set()
    for fd in glob.glob(f"/proc/{pid}/fd/*"):
        try:
            out.add(os.path.realpath(os.readlink(fd)))
        except OSError:
            pass
    return out


@pytest.mark.integration
def test_fc_never_opens_truth_channel(require_binaries, gcs_conf):
    from vayu_headless import SitlSession
    if not os.path.isdir("/proc"):
        pytest.skip("seam guard needs /proc (Linux)")

    lab = SitlSession(gcs=False, conf=gcs_conf)
    try:
        pid = lab.sitl.pid
        pose = os.path.realpath(lab.paths["pose"])   # ground-truth channel
        imu = os.path.realpath(lab.paths["imu"])     # sensor channel

        # Wait for the FC to actually connect its sensor FIFO — that is the
        # readiness signal the positive control below needs (truth() flows
        # before the FC opens its IMU, so don't gate on it).
        t0, opened = time.time(), set()
        while time.time() - t0 < 10.0:
            opened = _fc_open_paths(pid)
            if imu in opened:
                break
            time.sleep(0.1)

        # HONESTY: the FC must NOT hold the truth/pose FIFO open.
        assert pose not in opened, (
            f"SEAM VIOLATION: FC (pid {pid}) has the ground-truth pose FIFO "
            f"open: {pose}. The firmware must never read physics truth.")

        # POSITIVE CONTROL: it really is consuming sensors (raw IMU), so the
        # absence above means 'blind to truth', not 'opened nothing yet'.
        assert imu in opened, (
            f"FC (pid {pid}) is not reading the IMU FIFO {imu}; the session "
            f"never came up — test would be vacuously passing.")
    finally:
        lab.close()
