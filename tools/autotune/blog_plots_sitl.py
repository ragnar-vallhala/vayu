#!/usr/bin/env python3
"""SITL step-response figures for the autotuning blog: roll doublet (seed vs
tuned) and the yaw heading-hold demo. Launches one headless SITL stack."""
import json, os, sys, time
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT  = os.path.join(REPO, "docs", "blog", "pid-autotuning", "images")
os.makedirs(OUT, exist_ok=True)
sys.path.insert(0, HERE)
import autotune as AT
from sitl import SitlStack

plt.rcParams.update({"figure.facecolor": "white", "axes.grid": True,
                     "grid.alpha": 0.3, "font.size": 10})

geom = json.load(open(os.path.join(REPO, "logs", "autotune_vehicle.json")))
world = json.load(open(os.path.join(REPO, "logs", "autotune_world.json")))

def roll_step(st, gains):
    """Clean rising step: hold level briefly, then command +21 deg and hold."""
    AT.apply_gains(st, gains, False)
    st.reset(seed=12345); st.set_testrig(True, tether_k=30.0)
    st.set_rc(roll=1500, pitch=1500, yaw=1500, thr=1000, ch6=1000)
    st.clear_samples(); st.wait_level(); st.arm()
    st.set_rc(thr=1500); time.sleep(0.5)
    st.clear_samples()
    time.sleep(0.6)                 # baseline at 0 deg before the step
    st.set_rc(roll=1800)            # STEP UP to ~+21 deg, then hold
    time.sleep(5.0)                 # long hold so rise + settle are fully visible
    s = st.snapshot(); st.disarm()
    t0 = s[0][0]
    return ([d[0] - t0 for d in s],
            [d[1]["roll_angle_sp"] for d in s],
            [d[1]["roll_angle_curr"] for d in s])

def yaw_rate_step(st, gains):
    """Inner rate-loop step: command a yaw RATE and watch the measured body rate
    catch the setpoint. Yaw is rate-controlled, so the stick IS the rate command."""
    AT.apply_gains(st, gains, True)          # tune_yaw=True -> apply yaw rate gains
    st.reset(seed=12345); st.set_testrig(True, tether_k=0.0)
    st.set_rc(roll=1500, pitch=1500, yaw=1500, thr=1000, ch6=1000)
    st.clear_samples(); st.wait_level(); st.arm()
    st.set_rc(thr=1500); time.sleep(0.5)
    st.clear_samples()
    time.sleep(0.5)                          # baseline at 0 deg/s
    st.set_rc(yaw=1750)                       # step the commanded yaw RATE up
    time.sleep(2.5)
    st.set_rc(yaw=1500)                        # back to 0 -> rate must return
    time.sleep(2.0)
    s = st.snapshot(); st.disarm()
    t0 = s[0][0]
    return ([d[0] - t0 for d in s],
            [d[1]["yaw_rate_sp"] for d in s],
            [d[1]["yaw_rate_curr"] for d in s])

def yaw_hold(st, gains):
    AT.apply_gains(st, gains, False)
    st.reset(seed=12345); st.set_testrig(True, tether_k=0.0)
    st.set_rc(roll=1500, pitch=1500, yaw=1500, thr=1000, ch6=1000)  # stabilise
    st.clear_samples(); st.wait_level(); st.arm()
    st.set_rc(thr=1500); time.sleep(0.4)
    st.clear_samples()
    st.set_rc(yaw=1700); time.sleep(1.2)      # command yaw
    st.set_rc(yaw=1500); time.sleep(2.0)      # release -> must HOLD heading
    s = st.snapshot(); st.disarm()
    t0 = s[0][0]
    return ([d[0] - t0 for d in s],
            [d[1]["yaw_angle_curr"] for d in s],
            [d[1]["yaw_rate_curr"] for d in s])

def main():
    st = SitlStack(REPO, geometry=geom, world=world, quiet=True, rates=(1000, 8000, 120))
    st.tether_k = 30.0; st.start()
    try:
        seed  = [0.007, 0.002, 0.0005, 2.0, 0.0]
        tuned = [0.0005, 0.0033333, 0.0, 4.0, 0.0]   # buzz-free autotuned pick
        ts, sp_s, cur_s = roll_step(st, seed)
        tt, sp_t, cur_t = roll_step(st, tuned)

        fig, ax = plt.subplots(figsize=(9, 4.5))
        ax.plot(tt, sp_t, color="#555", ls="--", lw=1.6, label="commanded (step to +21°)")
        ax.plot(ts, cur_s, color="#E06C75", lw=2, label="seed gains (hand-picked)")
        ax.plot(tt, cur_t, color="#98C379", lw=2, label="autotuned (buzz-free)")
        ax.axhline(0, color="#ccc", lw=0.8)
        ax.set_xlabel("time (s)"); ax.set_ylabel("roll angle (deg)")
        ax.set_title("Roll step response: command rises 0 → 21°, autotuned tracks it faster")
        ax.legend(loc="lower right"); fig.tight_layout()
        png = os.path.join(OUT, "step_roll.png"); fig.savefig(png, dpi=130); plt.close(fig)
        print("wrote", png)

        # Inner rate-loop catchup: commanded yaw rate vs measured body rate.
        yaw_tuned = [0.0005, 0.0033333, 0.0, 4.0, 0.0, 0.018, 0.0, 0.0, 0.0]
        rt, rsp, rcur = yaw_rate_step(st, yaw_tuned)
        fig, ax = plt.subplots(figsize=(9, 4.5))
        ax.plot(rt, rsp, color="#555", ls="--", lw=1.6, label="commanded rate (setpoint)")
        ax.plot(rt, rcur, color="#61AFEF", lw=2, label="measured body rate (gyro)")
        ax.axhline(0, color="#ccc", lw=0.8)
        ax.set_xlabel("time (s)"); ax.set_ylabel("yaw rate (deg/s)")
        ax.set_title("Inner rate loop: the measured body rate catches the commanded rate")
        ax.legend(loc="upper right"); fig.tight_layout()
        png = os.path.join(OUT, "step_rate.png"); fig.savefig(png, dpi=130); plt.close(fig)
        print("wrote", png)

        ty, yaw, yr = yaw_hold(st, tuned)
        fig, ax1 = plt.subplots(figsize=(9, 4.5))
        ax1.plot(ty, yaw, color="#61AFEF", lw=2, label="yaw heading (deg)")
        ax1.axvline(1.2, color="#888", ls=":", lw=1)
        ax1.text(1.25, min(yaw), "stick released", color="#666", fontsize=9)
        ax1.set_xlabel("time (s)"); ax1.set_ylabel("yaw heading (deg)", color="#61AFEF")
        ax1.tick_params(axis="y", colors="#61AFEF")
        ax2 = ax1.twinx()
        ax2.plot(ty, yr, color="#E5C07B", lw=1.5, alpha=0.8, label="yaw rate (deg/s)")
        ax2.set_ylabel("yaw rate (deg/s)", color="#E5C07B")
        ax2.tick_params(axis="y", colors="#E5C07B")
        ax1.set_title("Yaw is rate-controlled in stabilise mode: holds heading on release (no snap-back to 0)")
        fig.tight_layout()
        png = os.path.join(OUT, "yaw_hold.png"); fig.savefig(png, dpi=130); plt.close(fig)
        print("wrote", png)
    finally:
        st.stop()
    print("SITL figures done")

if __name__ == "__main__":
    main()
