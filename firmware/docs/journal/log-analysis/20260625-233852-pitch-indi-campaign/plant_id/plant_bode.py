#!/usr/bin/env python3
"""Plant-ID figure: real vs sim plant phase, the delay->crossover law, and what
each sim modification actually produces. -> ../plots/10_plant_id.png"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "plots")
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({"figure.dpi": 120, "font.size": 9, "axes.grid": True, "grid.alpha": 0.3})
W = 2 * np.pi

# controller Gc(jw) from gyro (pitch gains, compiled defaults)
g = dict(Kp=0.012, Ki=0.005, Kd=0.0001, td=0.004, Kpa=1.6898)
def Gc(w):
    s = 1j * w
    return (g["Kp"] + g["Ki"] / s) * (g["Kpa"] / s + 1) + g["Kd"] * s / (g["td"] * s + 1)
def plant(w, K, tau=0.0209, Td=0.0):
    s = 1j * w
    return K / (s * (tau * s + 1)) * np.exp(-1j * w * Td)

fig, axs = plt.subplots(1, 3, figsize=(10.5, 3.7))

# (1) plant phase: real measured vs sim model
f = np.linspace(0.3, 6, 500); w = W * f
axs[0].plot(f, np.degrees(np.angle(plant(w, 1.0, tau=0.0125, Td=0.0))), "g-",
            label="SIM plant (int + 12.5ms pole)")
axs[0].plot(f, np.degrees(np.angle(plant(w, 1.0, tau=0.0209, Td=0.109))), "r-",
            label="REAL plant (+109ms delay)")
real_pts = [(2.05, -176), (1.80, -175), (1.81, -172), (1.77, -172), (1.73, -172), (1.70, -171)]
axs[0].scatter([p[0] for p in real_pts], [p[1] for p in real_pts], c="r", s=30,
               zorder=5, label="REAL measured @ cycle")
axs[0].axhline(-180, color="k", ls=":", lw=1)
axs[0].axvspan(1.7, 2.05, color="r", alpha=0.07)
axs[0].set_xlabel("frequency (Hz)"); axs[0].set_ylabel("plant phase ∠P (deg)")
axs[0].set_title("Plant phase: sim has no transport delay", fontsize=9)
axs[0].legend(fontsize=6.5, loc="lower left"); axs[0].set_ylim(-200, -60)

# (2) loop -180 crossover frequency vs added transport delay
Tds = np.linspace(0, 0.18, 200); fx = []
ws = np.linspace(W * 0.3, W * 9, 6000)
for Td in Tds:
    ph = np.unwrap(np.angle(Gc(ws) * plant(ws, 1381.0, Td=Td)))
    idx = np.where(ph <= -np.pi)[0]
    fx.append(ws[idx[0]] / W if len(idx) else np.nan)
axs[1].plot(Tds * 1e3, fx, "b-")
axs[1].axhspan(1.7, 2.05, color="r", alpha=0.12, label="observed real cycle")
axs[1].axvline(109, color="r", ls="--", lw=1); axs[1].axvline(0, color="g", ls="--", lw=1)
axs[1].text(4, 7, "sim\n(0ms→stable)", color="g", fontsize=7)
axs[1].text(112, 6, "real\n109ms", color="r", fontsize=7)
axs[1].set_xlabel("actuator transport delay (ms)")
axs[1].set_ylabel("loop −180° crossover (Hz)")
axs[1].set_title("Delay sets the cycle frequency", fontsize=9)
axs[1].legend(fontsize=7); axs[1].set_ylim(0, 9)

# (3) what each sim mod produces (measured)
mods = ["SIM\nbaseline", "SIM\n+real K\n(inertia)", "SIM\n+τ pole\n120ms", "REAL", "TARGET\n+delay"]
freq = [0, 18.2, 0, 1.9, 1.9]
rms = [6, 1498, 8, 170, 170]
colors = ["#2ca02c", "#7a0177", "#ff7f0e", "#d62728", "#1f77b4"]
ax = axs[2]
b = ax.bar(mods, rms, color=colors)
ax.set_yscale("log"); ax.set_ylabel("body-rate RMS (deg/s, log)")
for i, (fr_, r_) in enumerate(zip(freq, rms)):
    ax.text(i, r_ * 1.3, f"{fr_:.0f}Hz" if fr_ else "stable", ha="center", fontsize=6.5)
ax.set_title("High K → 18Hz (wrong); delay → 2Hz (right)", fontsize=9)
ax.tick_params(axis="x", labelsize=6.5)

fig.suptitle("Real-plant ID: the limit cycle is a ~100 ms actuator transport delay the sim lacks "
             "(gain alone gives the wrong 18 Hz mode)", fontsize=9.5)
fig.tight_layout(rect=[0, 0, 1, 0.93])
fig.savefig(os.path.join(OUT, "10_plant_id.png")); plt.close(fig)
print("wrote", os.path.join(OUT, "10_plant_id.png"))
