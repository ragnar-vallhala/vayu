#!/usr/bin/env python3
"""Plot the measured sim-vs-real parity (fresh headless runs) -> ../plots/09_sim_parity.png."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "plots")
plt.rcParams.update({"figure.dpi": 120, "font.size": 9, "axes.grid": True, "grid.alpha": 0.3})

# measured (fresh headless runs) vs real on-hardware
K_sim = {"roll": 187, "pitch": 199}
K_real = {"roll": 563, "pitch": 1381}
hover_sim, hover_real = 0.252, 0.45
# rate RMS: sim hover (calm) / real active windows / sim with parity inertia (diverges)
rate = {"SIM hover": 6, "REAL active": 120, "SIM @parity-I": 3500}

fig, axs = plt.subplots(1, 3, figsize=(9.6, 3.5))
# 1) effectiveness K
x = np.arange(2); w = 0.36
axs[0].bar(x - w / 2, [K_sim["roll"], K_sim["pitch"]], w, color="#2ca02c", label="SIM (measured)")
axs[0].bar(x + w / 2, [K_real["roll"], K_real["pitch"]], w, color="#d62728", label="REAL (sysid)")
for i, ax_ in enumerate(["roll", "pitch"]):
    axs[0].text(i, K_real[ax_] + 30, f"{K_real[ax_]/K_sim[ax_]:.1f}×", ha="center", fontsize=8, color="#d62728")
axs[0].set_xticks(x); axs[0].set_xticklabels(["roll", "pitch"])
axs[0].set_ylabel("control effectiveness K  (deg/s² per u)")
axs[0].set_title("Plant gain: real is 3–7× the sim", fontsize=9); axs[0].legend(fontsize=7)
# 2) hover throttle
axs[1].bar(["SIM", "REAL"], [hover_sim, hover_real], color=["#2ca02c", "#d62728"])
axs[1].axhline(0.5, color="k", ls=":", lw=0.8)
axs[1].set_ylabel("hover throttle"); axs[1].set_ylim(0, 0.6)
axs[1].set_title("Thrust margin: sim over-powered", fontsize=9)
# 3) what happens when you give the sim real effectiveness
cols = ["#2ca02c", "#d62728", "#7a0177"]
axs[2].bar(list(rate.keys()), list(rate.values()), color=cols)
axs[2].set_yscale("log"); axs[2].set_ylabel("body-rate RMS (deg/s, log)")
axs[2].set_title("Parity inertia → same firmware diverges", fontsize=9)
axs[2].tick_params(axis="x", labelsize=7, rotation=10)
fig.suptitle("Sim↔real parity (fresh headless SITL): the sim flies because its plant is "
             "3–7× too weak; modeling real effectiveness destabilizes the same firmware",
             fontsize=9.5)
fig.tight_layout(rect=[0, 0, 1, 0.94])
fig.savefig(os.path.join(OUT, "09_sim_parity.png")); plt.close(fig)
print("wrote", os.path.join(OUT, "09_sim_parity.png"))
