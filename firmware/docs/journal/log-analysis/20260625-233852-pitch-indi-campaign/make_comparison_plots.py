#!/usr/bin/env python3
"""Quantitative figures for reference-autopilots-comparison.md.

Writes plots/cmp_latency.png (gyro->PWM latency budget) and
plots/cmp_resource.png (CPU + heap headroom for the proposed changes).
Numbers trace §8 and §10 of the comparison doc. Regenerate: python3 make_comparison_plots.py
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PLOTS = os.path.join(HERE, "plots")
os.makedirs(PLOTS, exist_ok=True)

C = {  # stable colours per latency stage
    "sensor+bus":   "#9ecae1",
    "gyro filter":  "#fb6a4a",
    "scheduling":   "#fdae6b",
    "compute":      "#bdbdbd",
    "output stage": "#74c476",
}

# ---- Figure 1: gyro -> PWM latency budget (typical, ms), traces §8 ----------
stages = list(C.keys())
# rows: vayu, PX4, ArduPilot  (typical ms per stage)
data = {
    "vayu\n(F4 1 kHz)":      [1.10, 0.60, 0.60, 0.05, 1.95],
    "PX4\n(event chain)":    [0.65, 5.60, 0.10, 0.30, 0.05],
    "ArduPilot\n(400 Hz)":   [1.50, 13.5, 1.25, 0.30, 1.30],
}
labels = list(data.keys())
vals = np.array([data[k] for k in labels])
totals = vals.sum(axis=1)

fig, ax = plt.subplots(figsize=(9.2, 3.4))
left = np.zeros(len(labels))
y = np.arange(len(labels))[::-1]  # vayu on top
for j, st in enumerate(stages):
    ax.barh(y, vals[:, j], left=left, color=C[st], label=st, edgecolor="white", height=0.6)
    left += vals[:, j]
for yi, tot in zip(y, totals):
    ax.text(tot + 0.25, yi, f"{tot:.1f} ms", va="center", ha="left", fontweight="bold", fontsize=10)
ax.set_yticks(y)
ax.set_yticklabels(labels, fontsize=10)
ax.set_xlabel("gyro sample → motor-command latency  (ms, typical)")
ax.set_xlim(0, 20.5)
ax.set_title("Where the gyro→PWM latency goes — lowest total ≠ best-flying", fontsize=11, fontweight="bold")
ax.legend(ncol=5, fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.22), framealpha=0.95)
ax.spines[["top", "right"]].set_visible(False)
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "cmp_latency.png"), dpi=150, bbox_inches="tight")
plt.close(fig)

# ---- Figure 2: resource headroom for the proposed changes, traces §10 -------
fig, (axc, axm) = plt.subplots(1, 2, figsize=(9.2, 3.2))

# CPU: share of the 84 MHz core
axc.bar(["EKF\n(existing)", "all proposed\nchanges"], [17.5, 2.0],
        color=["#969696", "#74c476"], edgecolor="white", width=0.6)
axc.set_ylabel("% of 84 MHz core")
axc.set_ylim(0, 20)
axc.set_title("CPU: the changes are tiny vs the EKF", fontsize=10, fontweight="bold")
for x, v, lab in [(0, 17.5, "~17.5%"), (1, 2.0, "~1.5–2.5%")]:
    axc.text(x, v + 0.4, lab, ha="center", fontweight="bold", fontsize=9)
axc.spines[["top", "right"]].set_visible(False)

# RAM: the 56 KB heap, and where the changes land
seg = [("task stacks + IPC", 19.5, "#969696"),
       ("proposed (v_malloc)", 4.0, "#74c476"),
       ("free heap", 32.5, "#e5e5e5")]
bottom = 0
for name, kb, col in seg:
    axm.bar(["56 KB heap"], [kb], bottom=bottom, color=col, edgecolor="white", width=0.5, label=f"{name} ({kb:.0f} KB)")
    bottom += kb
axm.set_ylabel("KB")
axm.set_ylim(0, 60)
axm.set_title("Heap: changes carve from free, .bss stays flat", fontsize=10, fontweight="bold")
axm.legend(fontsize=7.5, loc="upper right", framealpha=0.9)
axm.spines[["top", "right"]].set_visible(False)
axm.text(0.5, -0.16, "Placed on the abundant heap (not static .bss) → the boot-HardFault "
         "invariant is never approached.", transform=axm.transAxes, ha="center",
         fontsize=7.5, style="italic", color="#444")
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "cmp_resource.png"), dpi=150, bbox_inches="tight")
plt.close(fig)

print("wrote plots/cmp_latency.png, plots/cmp_resource.png")

# ---- Figure 3: time-base / scheduling models (graphviz), traces §1 ----------
import subprocess

TIMEBASE_DOT = r'''
digraph timebase {
  rankdir=TB; bgcolor=white;
  node [shape=box, style="rounded,filled", fontname="DejaVu Sans", fontsize=11, margin="0.12,0.07"];
  edge [fontname="DejaVu Sans", fontsize=9];

  subgraph cluster_v {
    label="vayu — fixed 1 kHz loop, free-running (constant dt)";
    fontname="DejaVu Sans"; fontsize=12; color="#888888"; style=rounded; fontcolor="#222222";
    v1 [label="IMU ~2 kHz\n(higher-prio task)", fillcolor="#dbeafe"];
    v2 [label="rate task: wait 1 kHz tick,\ndrain freshest sample", fillcolor="#dbeafe"];
    v3 [label="mix → PWM", fillcolor="#dcfce7"];
    v1 -> v2 [label="overwrite ring", style=dashed];
    v2 -> v3;
  }
  subgraph cluster_p {
    label="PX4 — uORB event chain, one max-prio thread (measured dt)";
    fontname="DejaVu Sans"; fontsize=12; color="#888888"; style=rounded; fontcolor="#222222";
    p1 [label="gyro publish", fillcolor="#fee2e2"];
    p2 [label="ang. vel", fillcolor="#fef9c3"];
    p3 [label="rate ctrl", fillcolor="#fef9c3"];
    p4 [label="allocation", fillcolor="#fef9c3"];
    p5 [label="DShot out", fillcolor="#dcfce7"];
    p1 -> p2 [label="callback"]; p2 -> p3 [label="pub"]; p3 -> p4 [label="pub"]; p4 -> p5 [label="pub"];
  }
  subgraph cluster_a {
    label="ArduPilot — gyro-paced loop (measured dt)";
    fontname="DejaVu Sans"; fontsize=12; color="#888888"; style=rounded; fontcolor="#222222";
    a1 [label="gyro sample", fillcolor="#fee2e2"];
    a2 [label="fast_loop @400 Hz\n(or rate thread ≤8 kHz)", fillcolor="#fef9c3"];
    a3 [label="motors", fillcolor="#dcfce7"];
    a1 -> a2 [label="wait_for_sample:\nreleases loop + gives dt"]; a2 -> a3;
  }
}
'''

SAT_DOT = r'''
digraph sat {
  rankdir=TB; bgcolor=white;
  node [shape=box, style="rounded,filled", fontname="DejaVu Sans", fontsize=11, margin="0.14,0.09"];
  edge [fontname="DejaVu Sans", fontsize=9, color="#555555"];

  D [label="demanded motor mix exceeds [0,1]\n(headroom < demand)", fillcolor="#eeeeee", shape=box, style="filled,rounded"];

  V [label="vayu\nscale the WHOLE attitude\ndifferential down — keep throttle", fillcolor="#fde8e8"];
  P [label="PX4 airmode\ncut thrust (up or down),\nthen sacrifice yaw", fillcolor="#eef6ff"];
  A [label="ArduPilot\nspend yaw headroom,\nthen rpy_scale the RPY vector", fillcolor="#eef6ff"];

  VR [label="roll/pitch LOST\n→ 2 Hz limit cycle, no hover", fillcolor="#ffd0d0", color="#c0392b"];
  PR [label="roll/pitch PRESERVED", fillcolor="#cdebcd", color="#27ae60"];
  AR [label="roll/pitch PRESERVED", fillcolor="#cdebcd", color="#27ae60"];

  D -> V; D -> P; D -> A;
  V -> VR; P -> PR; A -> AR;
}
'''

for dot_src, out in [(TIMEBASE_DOT, "cmp_timebase.png"), (SAT_DOT, "cmp_saturation.png")]:
    p = subprocess.run(["dot", "-Tpng", "-Gdpi=150", "-o", os.path.join(PLOTS, out)],
                       input=dot_src.encode(), capture_output=True)
    if p.returncode != 0:
        raise SystemExit(f"dot failed for {out}: {p.stderr.decode()}")
print("wrote plots/cmp_timebase.png, plots/cmp_saturation.png")
