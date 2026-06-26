#!/usr/bin/env python3
"""Deeper pitch-oscillation decomposition: is 1.77 Hz in the rate SETPOINT
(outer/estimator) or only the MEASUREMENT (inner)? Plus angle-loop consistency,
motor saturation pattern, and outer-loop timing."""
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

def dom(x, fs, fmin=0.5):
    x = x - x.mean(); w = np.hanning(len(x))
    F = np.abs(np.fft.rfft(x*w)); fr = np.fft.rfftfreq(len(x), 1/fs)
    F[fr < fmin] = 0
    return fr[np.argmax(F)], F.max()

def main():
    path = sys.argv[1]
    ct, mt = [], []
    for t, mid, pl in walk(path):
        if mid == CT:
            c = nl.ControlTrace.unpack(pl)
            ct.append((t, c.pitch_angle_sp, c.pitch_angle_curr, c.pitch_rate_sp,
                       c.pitch_rate_curr, c.pitch_out, c.thro_out, c.outer_dt, c.inner_dt))
        elif mid == MT:
            m = nl.MotorTelemetry.unpack(pl)
            mt.append((t,) + tuple(m.cmd[:4]))
    a = np.array(ct, float)
    t = a[:,0]-a[0,0]; fs = len(t)/(t[-1]-t[0])
    asp, acur, rsp, rcur, pout, thr, odt, idt = [a[:,i] for i in range(1,9)]

    # restrict to the active (throttle-up) part so idle zeros don't wash out stats
    act = thr > 0.25
    print(f"active samples (thr>0.25): {act.sum()}/{len(a)}  fs~{fs:.1f}Hz")
    print(f"outer_dt: median {np.median(odt)*1e3:.2f}ms  mean {odt.mean()*1e3:.2f}ms  "
          f"max {odt.max()*1e3:.2f}ms  std {odt.std()*1e3:.2f}ms  (decim ratio ~{np.median(odt)/np.median(idt):.0f})")

    def rep(name, x):
        f0,_ = dom(x[act], fs)
        print(f"  {name:18s} RMS {x[act].std():7.1f}  |max| {np.abs(x[act]).max():7.1f}  dom {f0:.2f}Hz")
    print("\nPITCH chain (active window):")
    rep("angle_sp", asp); rep("angle_curr", acur)
    rep("rate_sp (outer->)", rsp); rep("rate_curr (gyro)", rcur)
    rep("rate_out u", pout)

    # Is the setpoint itself oscillating? ratio of rate_sp osc to rate_curr osc.
    _, psp = dom(rsp[act], fs); _, pcur = dom(rcur[act], fs)
    print(f"\nrate_sp osc / rate_curr osc (1.77Hz band power ratio): {psp/pcur:.2f}")

    # Angle-loop law check: rate_sp ?= Kp*(angle_sp-angle_curr). Fit Kp.
    err = asp - acur
    m = act & (np.abs(err) > 1)
    kp = np.linalg.lstsq(err[m,None], rsp[m,None], rcond=None)[0][0,0]
    resid = rsp[m] - kp*err[m]
    print(f"angle-loop fit: rate_sp ~= {kp:.3f}*(angle_sp-angle_curr)   "
          f"resid RMS {resid.std():.1f} deg/s ({100*resid.std()/rsp[m].std():.0f}% unexplained)")

    # Lag between setpoint and measurement (cross-correlation, samples->ms).
    xs = np.interp(np.linspace(t[0],t[-1],int((t[-1]-t[0])*fs)), t, rsp-rsp.mean())
    xc = np.interp(np.linspace(t[0],t[-1],int((t[-1]-t[0])*fs)), t, rcur-rcur.mean())
    cc = np.correlate(xc, xs, "full"); lag = (np.argmax(cc)-(len(xs)-1))/fs*1e3
    print(f"rate_curr lags rate_sp by ~{lag:.0f} ms  (period of 1.77Hz = 565ms; 180deg = 283ms)")

    # Motors
    if mt:
        mm = np.array(mt, float); mt_t = mm[:,0]-mm[0,0]; M = mm[:,1:5]
        ma = np.interp(t, mt_t, M[:,0])  # align active mask by time roughly
        print("\nMOTOR cmd (active window, M1..M4 = FR,RR,RL,FL):")
        for i in range(4):
            mi = np.interp(t, mt_t, M[:,i])[act]
            print(f"  M{i+1}: mean {mi.mean():.3f}  min {mi.min():.3f}  max {mi.max():.3f}  "
                  f"floor(<=0.06): {100*np.mean(mi<=0.06):.0f}%  rail(>=0.99): {100*np.mean(mi>=0.99):.0f}%")

    # Sample of worst window so the phase is visible.
    bw = int(fs); best=-1; bi=0
    for i in range(0,len(rcur)-bw, bw//4):
        s=rcur[i:i+bw].std()
        if s>best: best,bi=s,i
    print(f"\nworst 1s @ t+{t[bi]:.1f}s — sampled every ~80ms:")
    print("   t     angSP angCUR  rSP    rCUR   u")
    for j in range(bi, bi+bw, max(1,int(fs*0.08))):
        print(f"  {t[j]:5.2f}  {asp[j]:6.1f}{acur[j]:7.1f}{rsp[j]:7.0f}{rcur[j]:7.0f}{pout[j]:7.2f}")

if __name__ == "__main__":
    main()
