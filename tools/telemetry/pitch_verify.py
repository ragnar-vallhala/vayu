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
"""Verify the saturation-limit-cycle hypothesis from the existing log.
Checks: (1) outer loop innocent, (2) no transport delay across time,
(3) actuator clipping active, (4) applied pitch differential saturates vs
demand, (5) railing is momentum-driven, (6) amplitude bounded/sustained
(limit cycle, not divergence), (7) throttle/floor trend, (8) roll contrast."""
import os, sys, struct
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "navlink", "sim"))
sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))
import frame, navlink_msgs as nl  # noqa: E402
CT, MT = nl.ControlTrace.MSGID, nl.MotorTelemetry.MSGID
FLOOR = 0.005

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

def bp(x, fs, f0, bw=1.2):
    X = np.fft.rfft(x-x.mean()); fr = np.fft.rfftfreq(len(x),1/fs)
    X[(fr<f0-bw)|(fr>f0+bw)] = 0
    return np.fft.irfft(X, len(x))

def lag_ms(a, b, fs, maxms=400):
    a=a-a.mean(); b=b-b.mean()
    cc=np.correlate(b,a,"full"); mid=len(a)-1; w=int(maxms/1000*fs)
    seg=cc[mid-w:mid+w+1]; k=np.argmax(np.abs(seg))-w
    return k/fs*1e3

def main():
    path=sys.argv[1]
    ct,mt=[],[]
    for t,mid,pl in walk(path):
        if mid==CT:
            c=nl.ControlTrace.unpack(pl)
            ct.append((t,c.pitch_angle_sp,c.pitch_angle_curr,c.pitch_rate_sp,
                       c.pitch_rate_curr,c.pitch_out,c.roll_out,c.thro_out))
        elif mid==MT:
            m=nl.MotorTelemetry.unpack(pl); mt.append((t,)+tuple(m.cmd[:4]))
    a=np.array(ct,float); t0=a[0,0]; t=a[:,0]-t0
    asp,acur,rsp,rcur,u,uroll,thr=[a[:,i] for i in range(1,8)]
    mm=np.array(mt,float); mt_t=mm[:,0]-t0
    fs=50.0; tu=np.arange(t[0],t[-1],1/fs)
    def rs(x,xt=t): return np.interp(tu,xt,x)
    A_sp,A_cur,R_sp,R_cur,U,Uroll,TH=map(rs,[asp,acur,rsp,rcur,u,uroll,thr])
    M=np.stack([np.interp(tu,mt_t,mm[:,1+i]) for i in range(4)],1)
    Dapp=(M[:,0]-M[:,1]-M[:,2]+M[:,3])/4.0      # applied pitch diff (= u if unclipped)
    act=TH>0.30                                  # full-authority region (HW gate=0.30)
    f0=float(sys.argv[2]) if len(sys.argv)>2 else 1.79; per=1000/f0
    print(f"=== VERIFY (active=full-authority thr>0.30, n={act.sum()}, f0={f0}Hz) ===\n")

    # 1. outer loop innocent
    err=A_sp-A_cur; m=act&(np.abs(err)>1)
    kp=np.linalg.lstsq(err[m,None],R_sp[m,None],rcond=None)[0][0,0]
    resid=(R_sp[m]-kp*err[m]).std()
    print(f"[1] OUTER LOOP: angle_sp |max|={np.abs(A_sp[act]).max():.1f}deg (commanded ~0)")
    print(f"    rate_sp = {kp:.3f}*(angle_sp-angle_curr), resid {resid:.1f} deg/s "
          f"({100*resid/R_sp[m].std():.0f}% unexplained)  -> outer is a clean follower\n")

    # 2. no delay across 5 time chunks
    print("[2] TRANSPORT DELAY (motor-diff -> accel; 90deg=no delay):")
    for i in range(5):
        s=slice(int(i*len(tu)/5),int((i+1)*len(tu)/5))
        sub=act.copy(); mask=np.zeros_like(act); mask[s]=True; sub&=mask
        if sub.sum()<60: print(f"    chunk {i}: (idle)"); continue
        Db=bp(Dapp,fs,f0); Rb=bp(R_cur,fs,f0); ac=np.gradient(Rb,1/fs)
        l=lag_ms(Db[sub],ac[sub],fs)
        print(f"    t{i*6}-{(i+1)*6}s: {l:+5.0f}ms = {l/per*360:+.0f}deg")
    print()

    # 3. clipping active
    railu=np.mean(np.abs(U[act])>=0.99)*100
    nfloor=np.mean((M[act]<=FLOOR+1e-4).any(1))*100
    railm=np.mean((M[act]>=0.99).any(1))*100
    print(f"[3] SATURATION: PID |u|>=0.99 in {railu:.0f}% of samples;  "
          f"a motor at floor in {nfloor:.0f}%;  a motor railed-high in {railm:.0f}%\n")

    # 4. applied differential saturates vs demand
    print("[4] APPLIED vs DEMANDED pitch differential (Dapp should track u until it clips):")
    for lo,hi in [(0,.1),(.1,.3),(.3,.5),(.5,.8),(.8,1.01)]:
        mm2=act&(np.abs(U)>=lo)&(np.abs(U)<hi)
        if mm2.sum()>10:
            ratio=np.median(np.abs(Dapp[mm2])/np.abs(U[mm2]))
            print(f"    |u| in [{lo:.1f},{hi:.1f}): n={mm2.sum():4d}  median|Dapp| {np.median(np.abs(Dapp[mm2])):.3f}  "
                  f"applied/demanded {ratio:.2f}")
    print(f"    -> headroom to floor at thr~0.33 is ~{0.33-FLOOR:.2f}; above that, demand is clipped\n")

    # 5. railing is momentum-driven (rate error dominated by rate_curr not rate_sp)
    railed=act&(np.abs(U)>=0.99)
    print(f"[5] MOMENTUM-RAILING (railed samples): median|rate_curr|={np.median(np.abs(R_cur[railed])):.0f} "
          f"vs median|rate_sp|={np.median(np.abs(R_sp[railed])):.0f} deg/s  "
          f"-> error set by body momentum, not setpoint; lower kp can't unrail\n")

    # 6. amplitude bounded/sustained (limit cycle, not divergence)
    env=np.abs(bp(R_cur,fs,f0))
    win=int(fs*0.6)
    pk=np.array([env[i:i+win].max() for i in range(0,len(env)-win,win)])
    tt=np.arange(len(pk))*0.6
    pkA=pk[(tt>3)&(tt<27)]
    print(f"[6] LIMIT CYCLE: per-0.6s peak pitch-rate over active span: "
          f"mean {pkA.mean():.0f}, std {pkA.std():.0f} deg/s (trend "
          f"{np.polyfit(np.arange(len(pkA)),pkA,1)[0]:+.1f}/step)  -> bounded & sustained, not diverging\n")

    # 7. throttle / floor trend
    print("[7] THROTTLE TREND (within captured 0.30-0.354 only):")
    for lo,hi in [(.30,.32),(.32,.34),(.34,.36)]:
        mm2=(TH>=lo)&(TH<hi)
        if mm2.sum()>20:
            fl=np.mean((M[mm2]<=FLOOR+1e-4).any(1))*100
            print(f"    thr[{lo:.2f},{hi:.2f}): n={mm2.sum():4d}  pitchRMS {R_cur[mm2].std():.0f}  floor% {fl:.0f}")
    print("    (throttle never exceeded 0.354 -> CANNOT test the hover-throttle prediction from this log)\n")

    # 8. roll contrast
    print(f"[8] ROLL CONTRAST: roll |u|>=0.99 in {np.mean(np.abs(Uroll[act])>=0.99)*100:.0f}% "
          f"(pitch {railu:.0f}%);  roll_out RMS {Uroll[act].std():.3f} vs pitch {U[act].std():.3f}")

if __name__=="__main__":
    main()
