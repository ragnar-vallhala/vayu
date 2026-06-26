#!/usr/bin/env python3
"""Direct transport-delay estimate for the pitch loop.

For the rate plant  omega_dot = b * cmd(t - tau), the angular ACCELERATION
responds to the command with only the transport delay tau (no integrator
phase). So cross-correlating the applied command against d(rate)/dt gives tau
directly. We use the real post-mix pitch differential from the motor outputs
(anti-saturation means the PID 'u' is NOT what the motors deliver), and also
report u-vs-accel for comparison.
"""
import os, sys, struct
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "navlink", "sim"))
sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))
import frame, navlink_msgs as nl  # noqa: E402
CT, MT = nl.ControlTrace.MSGID, nl.MotorTelemetry.MSGID

def walk(path):
    blob = open(path, "rb").read()
    assert struct.unpack_from("<I", blob, 0)[0] == 0x56524543
    off = 20
    while off + 12 <= len(blob):
        t_us, dlen = struct.unpack_from("<QI", blob, off); off += 12
        data = blob[off:off+dlen]; off += dlen
        p = 0
        while len(data) - p >= frame.HDR_LEN + 2:
            if data[p] != 0x56 or data[p+1] != 0x02: p += 1; continue
            flen = frame.HDR_LEN + data[p+2] + 2
            if len(data) - p < flen: break
            d = frame.decode(data[p:p+flen]); p += flen
            if d.ok: yield t_us/1e6, d.msgid, d.payload

def bandpass(x, fs, f0, bw=1.2):
    """Narrow FFT band-pass around f0 to isolate the limit-cycle sinusoid."""
    X = np.fft.rfft(x - x.mean()); fr = np.fft.rfftfreq(len(x), 1/fs)
    X[(fr < f0-bw) | (fr > f0+bw)] = 0
    return np.fft.irfft(X, len(x))

def lag_ms(a, b, fs, maxms=400):
    """Lag of b behind a, in ms, via cross-correlation (search +-maxms)."""
    a = a - a.mean(); b = b - b.mean()
    cc = np.correlate(b, a, "full"); mid = len(a) - 1
    win = int(maxms/1000*fs)
    seg = cc[mid-win:mid+win+1]
    k = np.argmax(np.abs(seg)) - win
    sign = np.sign(seg[np.argmax(np.abs(seg))])
    return k/fs*1e3, sign

def main():
    path = sys.argv[1]
    ct, mt = [], []
    for t, mid, pl in walk(path):
        if mid == CT:
            c = nl.ControlTrace.unpack(pl)
            ct.append((t, c.pitch_rate_curr, c.pitch_out, c.thro_out))
        elif mid == MT:
            m = nl.MotorTelemetry.unpack(pl)
            ct_mt = (t,) + tuple(m.cmd[:4]); mt.append(ct_mt)
    a = np.array(ct, float); t0 = a[0,0]
    t = a[:,0]-t0; rate, u, thr = a[:,1], a[:,2], a[:,3]
    mm = np.array(mt, float); mt_t = mm[:,0]-t0

    fs = 50.0
    tu = np.arange(t[0], t[-1], 1/fs)
    R = np.interp(tu, t, rate)
    U = np.interp(tu, t, u)
    TH = np.interp(tu, t, thr)
    # real applied pitch differential: mix_pitch = [+1,-1,-1,+1] (FR,RR,RL,FL)
    M = np.stack([np.interp(tu, mt_t, mm[:,1+i]) for i in range(4)], 1)
    dpitch = (M[:,0] - M[:,1] - M[:,2] + M[:,3])   # front - rear

    act = TH > 0.25
    f0 = float(sys.argv[2]) if len(sys.argv) > 2 else 1.79
    Rb = bandpass(R, fs, f0)[act]
    Ub = bandpass(U, fs, f0)[act]
    Db = bandpass(dpitch, fs, f0)[act]
    accel = np.gradient(Rb, 1/fs)   # d(rate)/dt of the band-passed rate

    per = 1000.0/f0
    print(f"limit-cycle f0={f0:.2f}Hz  period={per:.0f}ms  (90deg={per/4:.0f}ms, 180deg={per/2:.0f}ms)")
    print(f"active samples: {act.sum()}\n")

    # b (effectiveness) sign+magnitude: accel = b * dpitch(t)  (zero-lag regression)
    b = np.linalg.lstsq(Db[:,None], accel[:,None], rcond=None)[0][0,0]
    print(f"effectiveness fit  accel = b*dpitch :  b = {b:8.1f} deg/s^2 per unit  (sign {'+' if b>0 else '-'})")

    for name, cmd in [("PID u", Ub), ("motor dpitch", Db)]:
        l_acc, s_acc = lag_ms(cmd, accel, fs)
        l_rate, s_rate = lag_ms(cmd, Rb, fs)
        print(f"\n{name}:")
        print(f"  cmd -> accel   lag {l_acc:+6.0f} ms  ({l_acc/per*360:+.0f} deg, corr sign {s_acc:+.0f})   <- transport delay")
        print(f"  cmd -> rate    lag {l_rate:+6.0f} ms  ({l_rate/per*360:+.0f} deg)   (expect ~+90deg from integrator + delay)")

    # roll baseline for contrast
    print("\n(reference) within-cycle: rate vs accel should be ~90deg by definition")
    lr,_ = lag_ms(Rb, accel, fs)
    print(f"  rate -> accel  lag {lr:+.0f} ms ({lr/per*360:+.0f} deg)")

if __name__ == "__main__":
    main()
