#!/usr/bin/env python3
"""Shared loader for the pitch-indi campaign: decode a VREC .bin into
time-aligned numpy series spanning the WHOLE pipeline — pilot input (RC),
estimator (attitude + vertical), control loop (ControlTrace), actuators
(MotorTelemetry), and reconstructed high-rate IMU (acc/gyr) from
ImuRaw keyframes + ImuCompressed f16 deltas. Also SystemHealth / inner_dt
for timing.

No averaging happens here — this just produces clean, time-stamped arrays.
Segmentation + metrics live in the analysis driver.
"""
import os, sys, struct
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "navlink", "sim"))
sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))
import frame
import navlink_msgs as nl

CT  = nl.ControlTrace.MSGID
ATT = nl.AttitudeEuler.MSGID
MOT = nl.MotorTelemetry.MSGID
RC  = nl.RcChannels.MSGID
VS  = nl.VerticalState.MSGID
IMR = nl.ImuRaw.MSGID
IMC = nl.ImuCompressed.MSGID
SH  = nl.SystemHealth.MSGID


def _f16(u):
    return float(np.array([u & 0xFFFF], dtype=np.uint16).view(np.float16)[0])


def walk(path):
    with open(path, "rb") as f:
        blob = f.read()
    magic = struct.unpack_from("<I", blob, 0)[0]
    assert magic == 0x56524543, f"bad VREC magic {magic:#x}"
    off = 20
    while off + 12 <= len(blob):
        t_us, dlen = struct.unpack_from("<QI", blob, off)
        off += 12
        data = blob[off:off + dlen]; off += dlen
        t = t_us / 1e6
        p = 0
        while len(data) - p >= frame.HDR_LEN + 2:
            if data[p] != 0x56 or data[p + 1] != 0x02:
                p += 1; continue
            flen = frame.HDR_LEN + data[p + 2] + 2
            if len(data) - p < flen:
                break
            d = frame.decode(data[p:p + flen]); p += flen
            if d.ok:
                yield t, d.msgid, d.payload


def load(path):
    """Return a dict of named record-arrays, each with a 't' column (seconds
    from first frame). IMU is reconstructed to its true ~48 Hz cadence."""
    ct, att, mot, rc, vs, sh = [], [], [], [], [], []
    imu = []
    vec = None
    t0 = None
    for t, mid, pl in walk(path):
        if t0 is None:
            t0 = t
        ts = t - t0
        if mid == CT:
            c = nl.ControlTrace.unpack(pl)
            ct.append((ts, c.roll_angle_sp, c.pitch_angle_sp, c.yaw_angle_sp,
                       c.roll_angle_curr, c.pitch_angle_curr, c.yaw_angle_curr,
                       c.roll_rate_sp, c.pitch_rate_sp, c.yaw_rate_sp,
                       c.roll_rate_curr, c.pitch_rate_curr, c.yaw_rate_curr,
                       c.roll_out, c.pitch_out, c.yaw_out, c.thro_out,
                       c.outer_dt, c.inner_dt))
        elif mid == ATT:
            a = nl.AttitudeEuler.unpack(pl)
            att.append((ts, a.roll, a.pitch, a.yaw,
                        a.rollspeed, a.pitchspeed, a.yawspeed))
        elif mid == MOT:
            m = nl.MotorTelemetry.unpack(pl)
            mot.append((ts, *m.cmd[:8]))
        elif mid == RC:
            r = nl.RcChannels.unpack(pl)
            rc.append((ts, *r.chan[:8], r.rssi))
        elif mid == VS:
            v = nl.VerticalState.unpack(pl)
            vs.append((ts, v.altitude, v.climb_rate, v.vertical_accel,
                       v.baro_altitude, v.agl, v.valid))
        elif mid == SH:
            h = nl.SystemHealth.unpack(pl)
            sh.append((ts, h.tx_overflow, h.imu_drop, h.log_wrap, h.cpu_load))
        elif mid == IMR:
            m = nl.ImuRaw.unpack(pl)
            vec = [*m.acc, *m.gyr, *m.mag, m.temp]
            imu.append((ts, *vec[:6]))            # acc xyz, gyr xyz
        elif mid == IMC and vec is not None:
            m = nl.ImuCompressed.unpack(pl)
            d = [_f16(x) for x in m.delta]
            vec = [vec[i] + d[i] for i in range(10)]
            imu.append((ts, *vec[:6]))

    def arr(rows, ncol):
        return np.array(rows, float) if rows else np.zeros((0, ncol))

    return {
        "ct":  arr(ct, 19),
        "att": arr(att, 7),
        "mot": arr(mot, 9),
        "rc":  arr(rc, 10),
        "vs":  arr(vs, 7),
        "sh":  arr(sh, 5),
        "imu": arr(imu, 7),
    }


# column index helpers --------------------------------------------------------
CT_COLS = ["t", "roll_sp", "pitch_sp", "yaw_sp", "roll", "pitch", "yaw",
           "rrate_sp", "prate_sp", "yrate_sp", "rrate", "prate", "yrate",
           "roll_out", "pitch_out", "yaw_out", "thr", "outer_dt", "inner_dt"]
CT_I = {n: i for i, n in enumerate(CT_COLS)}
ATT_COLS = ["t", "roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"]
ATT_I = {n: i for i, n in enumerate(ATT_COLS)}
VS_COLS = ["t", "altitude", "climb_rate", "vertical_accel", "baro_alt", "agl", "valid"]
VS_I = {n: i for i, n in enumerate(VS_COLS)}
IMU_COLS = ["t", "ax", "ay", "az", "gx", "gy", "gz"]
IMU_I = {n: i for i, n in enumerate(IMU_COLS)}


if __name__ == "__main__":
    d = load(sys.argv[1])
    for k, v in d.items():
        print(f"{k:5s} shape {v.shape}")
