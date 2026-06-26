#!/usr/bin/env python3
"""Figures for the pitch-cascade-oscillation archive.

Decodes the raw VREC capture in this dir (recorded live off the FC with
tools/telemetry/udp_telem_sniff.py --raw-bin) through the generated NavLink v2
codec and writes the oscillation PNGs into ./plots/. Bespoke to this archive.

    python3 docs/journal/log-analysis/20260625-231706-pitch-osc/make_plots.py
"""
import os, struct, math, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "../../../.."))
sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))
import navlink_msgs as nm

OUT = os.path.join(HERE, "plots"); os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({"figure.dpi": 120, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3})
SYNC, VER, HDR = 0x56, 0x02, 10


def decode(path):
    data = open(path, "rb").read()
    assert struct.unpack_from("<I", data, 0)[0] == 0x56524543
    off, t0, rows = 20, None, []
    while off + 12 <= len(data):
        t_us, ln = struct.unpack_from("<QI", data, off); off += 12
        chunk = data[off:off+ln]; off += ln
        if t0 is None: t0 = t_us
        p = 0
        while len(chunk) - p >= HDR + 2:
            if chunk[p] != SYNC or chunk[p+1] != VER:
                p += 1; continue
            plen = chunk[p+2]; flen = HDR + plen + 2
            if len(chunk) - p < flen: break
            mid = chunk[p+7] | chunk[p+8] << 8 | chunk[p+9] << 16
            pay = chunk[p+HDR:p+HDR+plen]; p += flen
            cls = nm.MSGID_TO_CLASS.get(mid)
            if cls is None: continue
            try: rows.append(((t_us - t0)/1e6, cls.__name__, cls.unpack(pay)))
            except Exception: pass
    return rows


rows = decode(os.path.join(HERE, "pitch-osc.bin"))
ct = [(t, o) for t, n, o in rows if n == "ControlTrace"]
mt = [(t, o) for t, n, o in rows if n == "MotorTelemetry"]
t = np.array([x[0] for x in ct])
sp = np.array([o.pitch_angle_sp for _, o in ct])
cur = np.array([o.pitch_angle_curr for _, o in ct])
rate = np.array([o.pitch_rate_curr for _, o in ct])
out = np.array([o.pitch_out for _, o in ct])
fs = 1.0 / np.median(np.diff(t))

# ---- 01: the limit cycle (full + zoom on the worst 8 s) -----------------
seg_std = [(np.std(rate[(t >= s) & (t < s+4)]), s) for s in np.arange(t[0], t[-1]-4, 1)]
ws = max(seg_std)[1]                      # worst-4s window start
z0, z1 = max(t[0], ws - 1), min(t[-1], ws + 7)

fig, axs = plt.subplots(3, 1, figsize=(13, 9), sharex=False)
a = axs[0]
a.plot(t, cur, lw=0.8, color="#d62728", label="pitch angle (actual)")
a.plot(t, sp, lw=1.0, color="#1f77b4", label="pitch angle setpoint")
a.axvspan(z0, z1, color="k", alpha=0.06, label="zoom below")
a.set_ylabel("deg"); a.set_title(
    f"Pitch cascade limit cycle — actual swings {cur.min():.0f}…{cur.max():.0f}° "
    f"({cur.max()-cur.min():.0f}° pk-pk) with setpoint ≈0", fontweight="bold")
a.legend(fontsize=8, loc="upper right")

a = axs[1]
m = (t >= z0) & (t <= z1)
a.plot(t[m], cur[m], lw=1.2, color="#d62728", label="pitch angle (deg)")
a.plot(t[m], rate[m]/10, lw=0.9, color="#2ca02c", alpha=0.8,
       label="pitch rate /10 (deg/s)")
a.axhline(0, color="k", lw=0.6)
a.set_ylabel("deg  /  (deg/s)/10")
a.set_title(f"Zoom {z0:.0f}–{z1:.0f}s — sustained ~1 Hz oscillation", fontweight="bold")
a.legend(fontsize=8, loc="upper right")

a = axs[2]
a.plot(t[m], out[m], lw=1.0, color="#9467bd")
a.axhline(1, color="k", ls="--", lw=0.8); a.axhline(-1, color="k", ls="--", lw=0.8)
sat = 100*np.mean(np.abs(out) > 0.98)
a.fill_between(t[m], 0.98, 1.0, color="r", alpha=0.15)
a.fill_between(t[m], -1.0, -0.98, color="r", alpha=0.15)
a.set_ylim(-1.15, 1.15); a.set_ylabel("pitch_out (±1)"); a.set_xlabel("s")
a.set_title(f"Controller output railed ±1.0 — {sat:.0f}% of armed samples saturated",
            fontweight="bold")
fig.tight_layout(); fig.savefig(os.path.join(OUT, "01_pitch_cascade.png")); plt.close(fig)

# ---- 02: spectrum (confirm the ~1 Hz peak) ------------------------------
x = cur - cur.mean()
n = len(x)
win = np.hanning(n)
X = np.abs(np.fft.rfft(x*win)) / n
f = np.fft.rfftfreq(n, d=1.0/fs)
band = (f > 0.2) & (f < 10)
fpk = f[band][np.argmax(X[band])]
fig, a = plt.subplots(1, 1, figsize=(10, 4))
a.plot(f, X, color="#d62728")
a.axvline(fpk, color="#1f77b4", lw=1.2, ls="--", label=f"peak {fpk:.2f} Hz")
a.set_xlim(0, 8); a.set_xlabel("Hz"); a.set_ylabel("amplitude (deg)")
a.set_title(f"Pitch-angle spectrum — dominant limit-cycle at {fpk:.2f} Hz",
            fontweight="bold")
a.legend(); fig.tight_layout()
fig.savefig(os.path.join(OUT, "02_pitch_spectrum.png")); plt.close(fig)

print(f"samples={n}  fs={fs:.1f}Hz  pitch pk-pk={cur.max()-cur.min():.1f}deg  "
      f"rate std={rate.std():.0f}dps  sat={sat:.1f}%  fpeak={fpk:.2f}Hz")
print("wrote plots/01_pitch_cascade.png plots/02_pitch_spectrum.png")
