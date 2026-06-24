#!/usr/bin/env python3
"""Generate analysis plots for the combined on-hardware tuning archive.

Decodes the three archived .bin logs (via ../parse_log.py) + the roll sysid CSV
and writes PNGs into ./plots/. Bespoke to this archive (3-log combined report).

Usage (from repo root):
    python3 docs/journal/log-analysis/20260622-012844-onhw-tune/make_plots.py
"""
import importlib.util, os, csv, ast, math, bisect
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
LA = os.path.dirname(HERE)
spec = importlib.util.spec_from_file_location("pl", os.path.join(LA, "parse_log.py"))
pl = importlib.util.module_from_spec(spec); spec.loader.exec_module(pl)

plt.rcParams.update({"figure.dpi": 120, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "figure.autolayout": True})

LOGS = {
  "rig_235115": os.path.join(HERE, "data/rig_telem_235115.bin"),
  "rig_235304": os.path.join(HERE, "data/rig_telem_235304_sysid-run.bin"),
  "ff_001043":  os.path.join(HERE, "freeflight/freeflight_001043_yaw-departure.bin"),
}
TITLE = {"rig_235115":"rig 23:51 (ANGLE)", "rig_235304":"rig 23:53 (ANGLE→ACRO)",
         "ff_001043":"free-flight 00:10 (gated)"}
COL = {"rig_235115":"#1f77b4", "rig_235304":"#d62728", "ff_001043":"#2ca02c"}
OUT = os.path.join(HERE, "plots"); os.makedirs(OUT, exist_ok=True)

def load(path):
    _, recs = pl.read_records(path)
    return pl.Collector().run(recs)
COLS = {k: load(v) for k, v in LOGS.items()}

def series(col, name, attr):
    out=[]
    for t,m,_ in col.msgs.get(name,[]):
        out.append((t, getattr(m, attr)))
    if not out: return np.array([]), np.array([])
    t=np.array([x[0] for x in out],float); v=np.array([x[1] for x in out],float)
    return (t-t[0])/1e6, v

def motors(col):
    T=[]; M=[]
    for t,m,_ in col.msgs.get("MotorTelemetry",[]):
        c=list(m.cmd)[:4]
        if len(c)==4: T.append(t); M.append(c)
    if not T: return np.array([]), np.zeros((0,4))
    T=np.array(T,float); return (T-T[0])/1e6, np.array(M)

def corr(a,b):
    a=np.asarray(a,float); b=np.asarray(b,float); n=min(len(a),len(b))
    a,b=a[:n],b[:n]
    if n<3 or a.std()==0 or b.std()==0: return float('nan')
    return float(np.corrcoef(a,b)[0,1])

def ctrl(col, attr):
    return series(col,"ControlTrace",attr)

# ---------- 01: pitch limit cycle (rig_235304) ----------
def p01():
    c=COLS["rig_235304"]
    t,pr=ctrl(c,"pitch_rate_curr"); _,rr=ctrl(c,"roll_rate_curr"); _,th=ctrl(c,"thro_out")
    fig,ax=plt.subplots(figsize=(9,3.4))
    ax.plot(t,pr,color="#d62728",lw=0.7,label="pitch rate (°/s)")
    ax.plot(t,rr,color="#1f77b4",lw=0.6,alpha=0.7,label="roll rate (°/s)")
    ax.set_ylabel("body rate (°/s)"); ax.set_xlabel("time (s)")
    ax2=ax.twinx(); ax2.plot(t,th,color="#555",lw=1.0,ls="--",label="throttle"); ax2.set_ylabel("throttle"); ax2.set_ylim(0,1); ax2.grid(False)
    ax.set_title("rig 23:53 — pitch limit cycle (~1.4 Hz, peaks ±322°/s) under throttle; roll stays quiet (rig-constrained)")
    l1,la1=ax.get_legend_handles_labels(); l2,la2=ax2.get_legend_handles_labels()
    ax.legend(l1+l2,la1+la2,loc="upper left",fontsize=7,ncol=3)
    fig.savefig(os.path.join(OUT,"01_pitch_limit_cycle.png")); plt.close(fig)

# ---------- 02: mixer-sign correlation (the yaw-sign confirmation) ----------
def mix_corr(col):
    t,m=motors(col)
    if len(t)==0: return {}
    tc,ro=ctrl(col,"roll_out"); _,po=ctrl(col,"pitch_out"); _,yo=ctrl(col,"yaw_out")
    n=len(tc)
    def rs(v): return np.array([v[min(len(v)-1,int(i*len(v)/n))] for i in range(n)])
    Mr=[rs(m[:,i]) for i in range(4)]
    roll=-Mr[0]-Mr[1]+Mr[2]+Mr[3]; pitch=Mr[0]-Mr[1]-Mr[2]+Mr[3]; yaw=Mr[0]-Mr[1]+Mr[2]-Mr[3]
    return {"roll":corr(ro,roll),"pitch":corr(po,pitch),"yaw":corr(yo,yaw)}
def p02():
    data={k:mix_corr(COLS[k]) for k in LOGS}
    axes=["roll","pitch","yaw"]; x=np.arange(3); w=0.25
    fig,ax=plt.subplots(figsize=(8,3.6))
    for i,k in enumerate(LOGS):
        vals=[data[k].get(a,0) for a in axes]
        ax.bar(x+(i-1)*w,vals,w,label=TITLE[k],color=COL[k])
    ax.axhline(0,color="k",lw=0.8)
    ax.set_xticks(x); ax.set_xticklabels([a+"\n(motor-diff vs *_out)" for a in axes])
    ax.set_ylabel("correlation"); ax.set_ylim(-1,1)
    ax.set_title("Mixer-sign check: + = motors realise commanded torque, − = INVERTED\n"
                 "yaw flips −0.7 (rig, default sign) → +0.75 (free-flight, after spin=[-1,1,-1,1] fix)")
    ax.legend(fontsize=7,loc="upper left")
    ax.annotate("yaw sign BACKWARDS\n(positive feedback)",(2,-0.55),fontsize=7,ha="center",va="top",color="#b00")
    fig.savefig(os.path.join(OUT,"02_mixer_sign.png")); plt.close(fig)

# ---------- 03: motor balance (left-heavy bias) ----------
def p03():
    fig,axs=plt.subplots(1,2,figsize=(9,3.4))
    for ax,k in zip(axs,["rig_235115","rig_235304"]):
        t,m=motors(COLS[k]); mask=m.max(axis=1)>0.1; A=m[mask]
        means=A.mean(axis=0)
        names=["M1\nFR","M2\nRR","M3\nRL","M4\nFL"]; cols=["#1f77b4","#1f77b4","#d62728","#d62728"]
        ax.bar(names,means,color=cols)
        L=means[2]+means[3]; R=means[0]+means[1]
        ax.set_title(f"{TITLE[k]}\nLEFT {L:.2f} vs RIGHT {R:.2f}  (L−R {L-R:+.2f})",fontsize=8)
        ax.set_ylabel("mean cmd (0..1)"); ax.set_ylim(0,0.7)
        ax.axhline(means.mean(),color="k",ls=":",lw=0.8)
    fig.suptitle("Left pair (red, M3+M4) runs +24–26% hotter than right (blue) — standing left-low correction",fontsize=9)
    fig.savefig(os.path.join(OUT,"03_motor_balance.png")); plt.close(fig)

# ---------- 04: motor timeline + saturation (rig_235304) ----------
def p04():
    t,m=motors(COLS["rig_235304"])
    fig,ax=plt.subplots(figsize=(9,3.4))
    nm=["M1 FR","M2 RR","M3 RL","M4 FL"]; cols=["#1f77b4","#9467bd","#d62728","#ff7f0e"]
    for i in range(4): ax.plot(t,m[:,i],lw=0.6,color=cols[i],label=nm[i])
    ax.axhline(0.95,color="k",ls="--",lw=0.8,alpha=0.6)
    nsat=int((m[:,2]>0.95).sum()); ax.text(t[-1]*0.5,0.96,f"M3 saturates {nsat} frames @1.0",fontsize=7,color="#b00")
    ax.set_ylabel("motor cmd (0..1)"); ax.set_xlabel("time (s)"); ax.set_ylim(0,1.05)
    ax.set_title("rig 23:53 — M3 (rear-left) / M4 (front-left) hit the 1.0 rail; right pair has headroom")
    ax.legend(fontsize=7,ncol=4,loc="lower right")
    fig.savefig(os.path.join(OUT,"04_motor_saturation.png")); plt.close(fig)

# ---------- 05: roll sysid capture ----------
def p05():
    p=os.path.join(HERE,"data/sysid_roll_capture.csv")
    if not os.path.exists(p): return
    t=[];u=[];g=[]
    with open(p) as f:
        for r in csv.DictReader(f): t.append(float(r["t_s"]));u.append(float(r["u"]));g.append(float(r["gyro_dps"]))
    t=np.array(t);u=np.array(u);g=np.array(g)
    fig,ax=plt.subplots(figsize=(9,3.4))
    ax.plot(t,g,color="#d62728",lw=0.7,label="roll rate (°/s)")
    ax2=ax.twinx(); ax2.plot(t,u,color="#1f77b4",lw=0.6,alpha=0.7,label="u (rate-PID out)"); ax2.set_ylabel("u (command)"); ax2.grid(False)
    ax.set_ylabel("roll rate (°/s)"); ax.set_xlabel("time (s)")
    ax.set_title("Roll system-ID chirp (0.5→12 Hz): plant fit K=563 (°/s)/u, τ=20.9 ms, R²=0.83")
    l1,la1=ax.get_legend_handles_labels(); l2,la2=ax2.get_legend_handles_labels()
    ax.legend(l1+l2,la1+la2,loc="upper right",fontsize=7)
    fig.savefig(os.path.join(OUT,"05_sysid_capture.png")); plt.close(fig)

# ---------- 06: accel scale (|a| vs g) ----------
def accmag(col):
    out=[]
    for t,m,_ in col.msgs.get("ImuRaw",[]):
        a=list(m.acc); g=list(m.gyr)
        if len(a)==3 and (len(g)<3 or max(abs(x) for x in g)<5):
            out.append(math.sqrt(sum(x*x for x in a)))
    return np.array(out)
def p06():
    fig,ax=plt.subplots(figsize=(8,3.4))
    for k in LOGS:
        a=accmag(COLS[k])
        if len(a): ax.hist(a,bins=30,alpha=0.5,color=COL[k],label=f"{TITLE[k]}  μ={a.mean():.2f} ({100*(a.mean()-9.807)/9.807:+.1f}%)")
    ax.axvline(9.807,color="k",lw=1.2,ls="--",label="g = 9.807")
    ax.set_xlabel("|accel| at rest (m/s²)"); ax.set_ylabel("count")
    ax.set_title("Accel scale error roughly halved by calibration: +7.8% (06-17) → +2.5–5.1% now")
    ax.legend(fontsize=7)
    fig.savefig(os.path.join(OUT,"06_accel_scale.png")); plt.close(fig)

# ---------- 07: mag field-magnitude spread ----------
def magmag(col):
    out=[]
    for t,m,_ in col.msgs.get("ImuRaw",[]):
        v=list(m.mag)
        if len(v)==3: out.append(math.sqrt(sum(x*x for x in v)))
    return np.array(out)
def p07():
    fig,ax=plt.subplots(figsize=(8,3.4))
    for k in LOGS:
        v=magmag(COLS[k])
        if len(v):
            sp=100*(v.max()-v.min())/v.mean()
            ax.hist(v,bins=30,alpha=0.5,color=COL[k],label=f"{TITLE[k]}  spread {sp:.0f}% of mean")
    ax.set_xlabel("|mag| (µT)"); ax.set_ylabel("count")
    ax.set_title("Mag spread tightened post-cal (220%→64% of mean) but residual hard-iron remains\n"
                 "(a calibrated mag would be a single narrow peak)")
    ax.legend(fontsize=7)
    fig.savefig(os.path.join(OUT,"07_mag_spread.png")); plt.close(fig)

# ---------- 08: kernel (CPU + stack + ipc anomaly) ----------
def cpu_load(col):
    pg=[(t,m) for t,m,_ in col.msgs.get("PerfGlobal",[])]
    loads=[]
    for i in range(1,len(pg)):
        dc=pg[i][1].cpu_cycles_lo-pg[i-1][1].cpu_cycles_lo
        di=pg[i][1].idle_cycles_lo-pg[i-1][1].idle_cycles_lo
        if dc>0 and 0<=di<=dc: loads.append(100*(1-di/dc))
    ipct=pg[-1][1].ipc_timeouts if pg else 0
    return (np.mean(loads) if loads else 0), ipct
def p08():
    fig,(ax1,ax2)=plt.subplots(1,2,figsize=(9,3.4))
    ks=list(LOGS); cpu=[]; ipc=[]
    for k in ks:
        c,i=cpu_load(COLS[k]); cpu.append(c); ipc.append(i)
    ax1.bar([TITLE[k] for k in ks],cpu,color=[COL[k] for k in ks])
    ax1.set_ylabel("CPU load (%)"); ax1.set_ylim(0,70); ax1.set_title("CPU ~50% (was 63% on 06-17)")
    for ax in (ax1,): ax.tick_params(axis="x",labelsize=6.5,rotation=12)
    ax2.bar([TITLE[k] for k in ks],ipc,color=[COL[k] for k in ks])
    ax2.set_ylabel("ipc_timeouts (cumulative)"); ax2.set_title("IPC timeouts: 0 steady, 7099 in 001043\n(calib-under-load contention)")
    ax2.tick_params(axis="x",labelsize=6.5,rotation=12)
    fig.savefig(os.path.join(OUT,"08_kernel.png")); plt.close(fig)

for fn in (p01,p02,p03,p04,p05,p06,p07,p08):
    try: fn(); print("ok",fn.__name__)
    except Exception as e: print("FAIL",fn.__name__,repr(e))
print("plots ->",OUT)
