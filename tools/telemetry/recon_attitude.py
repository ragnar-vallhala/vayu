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
"""Rebuild roll and pitch from a blackbox recording's raw accel and gyro.

Why this exists: the blackbox `att` stream recorded zeros for its whole life
before firmware 424b495 -- its encoder was handed the reciprocal of its scale,
so every attitude under 50 degrees quantised to 0. For any recording made
before that fix, the estimate the controller acted on is gone.

The INPUTS are not gone. The recorder logs raw accel and gyro COUNTS, not the
calibrated sample, precisely so a recording outlives the calibration that was
applied to it -- and CAL.BIN from the same card pull carries that calibration.
Together they are enough to recompute what the estimator should have seen.

    recon_attitude.py BLACKBOX.BIN[.gz] CAL.BIN 1017 1021

WHAT IT CANNOT DO: yaw is not a heading. The magnetometer reaches the hub but
not the recorder, so yaw here is integrated gyro -- it drifts without bound and
its zero is wherever the recording started. Roll and pitch are held by gravity
and do not drift; trust those.

FRAME CONVENTION is measured, not assumed. The quietest second of the
recording (by gyro magnitude) is taken as "at rest", and the sign of az there
decides whether the accel vector needs negating for the filter's reference.
Getting this wrong does not look wrong -- it produces a clean, plausible,
upside-down flight -- so it is checked against the data every run and printed.
"""
import argparse
import gzip
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hslog

G = 9.80665
CAL_PAYLOAD_FLOATS = 29  # v4: acc_off[3] si[9] gyr_off[3] mag_off[3] mag_si[9] trim[2]


def load_cal(path):
    """Unpack calib_file_header_t + bmx160_calibration_t (see driver/bmx160.h)."""
    d = open(path, "rb").read()
    magic, ver, payload = struct.unpack("<IHH", d[:8])
    f = np.array(struct.unpack("<%df" % CAL_PAYLOAD_FLOATS, d[8:8 + 4 * CAL_PAYLOAD_FLOATS]))
    return dict(magic=magic, ver=ver, payload=payload,
                acc_off=f[0:3], acc_si=f[3:12].reshape(3, 3),
                gyr_off=f[12:15], trim=f[27:29])


def lowpass(x, fs, fc):
    """One-pole applied forwards then backwards, so it adds no phase lag.

    This is post-processing, not a control loop: zero-phase is free here and a
    lagged gravity reference would smear every attitude excursion.
    """
    a = np.exp(-2.0 * np.pi * fc / fs)

    def once(v):
        out = np.empty_like(v)
        acc = v[0]
        for i, s in enumerate(v):
            acc = a * acc + (1.0 - a) * s
            out[i] = acc
        return out

    return once(once(x)[::-1])[::-1]


def mahony(t, gyr_dps, acc, kp=2.0, ki=0.02, nominal_fs=1827.0):
    """Gravity-referenced complementary filter, quaternion state.

    The accel is the only absolute reference available without a compass, so it
    corrects tilt and nothing else; yaw is pure integration.
    """
    q = np.array([1.0, 0.0, 0.0, 0.0])
    bias = np.zeros(3)
    out = np.empty((len(t), 3))
    dt_all = np.diff(t, prepend=t[0] - 1.0 / nominal_fs)
    for i in range(len(t)):
        dt = dt_all[i]
        if not (0.0 < dt < 0.05):
            dt = 1.0 / nominal_fs  # a block gap: coast at the nominal rate
        g = np.radians(gyr_dps[i])
        a = acc[i]
        n = np.linalg.norm(a)
        if n > 1e-3:
            a = a / n
            w, x, y, z = q
            v = np.array([2 * (x * z - w * y),
                          2 * (w * x + y * z),
                          w * w - x * x - y * y + z * z])
            e = np.cross(a, v)
            bias += ki * e * dt
            g = g + kp * e + bias
        w, x, y, z = q
        gx, gy, gz = g
        q = q + 0.5 * dt * np.array([-x * gx - y * gy - z * gz,
                                     w * gx + y * gz - z * gy,
                                     w * gy - x * gz + z * gx,
                                     w * gz + x * gy - y * gx])
        q /= np.linalg.norm(q)
        w, x, y, z = q
        out[i] = (np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
                  np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))),
                  np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))
    return out


def rest_window(t, gyr, fs):
    """The quietest second by mean |gyro|: the craft sitting still."""
    w = max(int(fs), 1)
    if len(t) <= w:
        return slice(0, len(t))
    gm = np.linalg.norm(gyr, axis=1)
    c = np.cumsum(np.insert(gm, 0, 0.0))
    i = int(np.argmin((c[w:] - c[:-w]) / w))
    return slice(i, i + w)


def reconstruct(hdr, streams, sess, cal, idx, acc_fc=5.0, verbose=True):
    t, imu = hslog.samples(hdr, streams, sess[idx]["blocks"], sid=1)
    t = np.asarray(t, float)
    # hslog has already applied each field's scale AND the sensor->body sign
    # map the firmware wrote into the FMT frame. What it has NOT applied is the
    # calibration -- the recorder logs counts on purpose.
    acc = np.column_stack([imu["ax"], imu["ay"], imu["az"]])
    gyr = np.column_stack([imu["gx"], imu["gy"], imu["gz"]])
    saturated = int(np.sum(np.abs(gyr) >= 1999.9))

    acc = (acc - cal["acc_off"]) @ cal["acc_si"].T
    gyr = gyr - cal["gyr_off"]

    fs = (len(t) - 1) / (t[-1] - t[0])
    sl = rest_window(t, gyr, fs)
    rest = acc[sl].mean(axis=0)
    inverted = rest[2] < 0.0
    if inverted:
        acc = -acc

    accf = np.column_stack([lowpass(acc[:, i], fs, acc_fc) for i in range(3)])
    att = mahony(t, gyr, accf, nominal_fs=fs)
    roll = att[:, 0] - cal["trim"][0]
    pitch = att[:, 1] - cal["trim"][1]
    yaw = att[:, 2]

    if verbose:
        print("\n===== session [%d] =====" % idx)
        print("  %d samples, %.1f s at %.0f Hz" % (len(t), t[-1] - t[0], fs))
        print("  rest window t=%.1fs  acc=[%+.2f %+.2f %+.2f] |a|=%.2f -> %s"
              % (t[sl.start], rest[0], rest[1], rest[2], np.linalg.norm(rest),
                 "az reads -g when level, accel negated" if inverted
                 else "az reads +g when level, accel as-is"))
        if saturated:
            print("  WARNING: %d gyro samples on the +-2000 dps rail; the true "
                  "rate there is unknown and the integration under-rotates"
                  % saturated)
        for n, v in (("roll", roll), ("pitch", pitch)):
            print("  %-5s min=%+7.1f  max=%+7.1f  sd=%5.1f deg"
                  % (n, v.min(), v.max(), v.std()))
        print("  yaw   integrated %+.0f deg over the run -- GYRO ONLY, drifts, "
              "no compass in the log" % (yaw[-1] - yaw[0]))
    return t, roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("blackbox", help="BLACKBOX.BIN or .bin.gz")
    ap.add_argument("cal", help="CAL.BIN from the same card pull")
    ap.add_argument("sessions", nargs="*", type=int,
                    help="session indices (default: every session with IMU data)")
    ap.add_argument("--csv-dir", default=None, help="write t,roll,pitch,yaw per session")
    ap.add_argument("--acc-fc", type=float, default=5.0,
                    help="gravity-reference low-pass cutoff, Hz (default 5)")
    args = ap.parse_args()

    raw = (gzip.open if args.blackbox.endswith(".gz") else open)(args.blackbox, "rb").read()
    hdr, streams, frames = hslog.decode(raw)
    sess = hslog.sessions(frames)
    cal = load_cal(args.cal)
    print("calibration v%d  board_trim=%+.2f/%+.2f deg"
          % (cal["ver"], cal["trim"][0], cal["trim"][1]))

    wanted = args.sessions
    if not wanted:
        wanted = [i for i, s in enumerate(sess)
                  if any(b.get("sid") == 1 for b in s["blocks"])]
    for idx in wanted:
        t, roll, pitch, yaw = reconstruct(hdr, streams, sess, cal, idx,
                                          acc_fc=args.acc_fc)
        if args.csv_dir:
            os.makedirs(args.csv_dir, exist_ok=True)
            p = os.path.join(args.csv_dir, "att_%d.csv" % idx)
            np.savetxt(p, np.column_stack([t, roll, pitch, yaw]),
                       delimiter=",", header="t,roll,pitch,yaw", comments="")
            print("  -> %s" % p)


if __name__ == "__main__":
    main()
