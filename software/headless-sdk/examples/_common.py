"""Shared helpers for the example maneuvers.

All examples fly the SAME vehicle + world the GCS has loaded by reading its
QSettings .conf (vveh geometry + vworld + collision mesh), so a headless run
matches what you see in Navigator.
"""
import csv
import os
import time

from vayu_headless.paths import gcs_conf_default
from vayu_headless.transport.vsim import quat_to_euler  # noqa: F401  re-exported


def gcs_conf():
    """The GCS config path if it exists (else None → SDK defaults)."""
    c = gcs_conf_default()
    return c if os.path.exists(c) else None


class Recorder:
    """Tiny CSV + in-memory row recorder."""

    def __init__(self, path, header):
        self.header = header
        self.rows = []
        self._f = open(path, "w", newline="") if path else None
        self._w = csv.writer(self._f) if self._f else None
        if self._w:
            self._w.writerow(header)

    def add(self, *row):
        self.rows.append(row)
        if self._w:
            self._w.writerow(["%.5g" % v if isinstance(v, float) else v
                              for v in row])

    def close(self):
        if self._f:
            self._f.close()


def arm_on_rig(sess, thr=0.5, settle=1.5):
    """Arm the FC and hold mid-throttle. On a tuning rig (SitlSession(rig=True))
    translation is pinned, so attitude responds to stick inputs without the
    craft flying away — ideal for step/chirp/doublet system-ID."""
    sess.set_rc(swa=1000, thr=1000)          # ensure STANDBY
    time.sleep(0.3)
    sess.set_rc(swa=2000, thr=1000)          # arm gesture
    time.sleep(settle)
    sess.stick(thr=thr)                      # mid throttle, level
    sess.set_rc(swa=2000)
    time.sleep(0.5)
