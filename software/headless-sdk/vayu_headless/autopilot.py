"""Pilot — continuous outer-loop guidance (takeoff / goto / land / station-keep).

Always writes RC STICKS only (never position) except the explicit takeoff/land
respawn, so the flight stays genuine firmware-in-the-loop physics. Carved
verbatim from tools/sim_host/sitl_lab.py.
"""
import threading
import time

from .transport.vsim import quat_to_euler


DEFAULT_GAINS = dict(kp_z=0.05, ki_z=0.02, kd_z=0.05, hover=0.36,
                     kp_h=0.06, kd_h=0.55, tilt=0.30, vmax=3.0)


class Pilot:
    """Continuous outer-loop guidance that runs for the LIFE of a session (not
    one blocking flight). Once airborne it never stops commanding: it always
    station-keeps at its current target, so the craft hovers under control
    between commands and the GCS keeps showing a live, stable aircraft. Flight
    commands (takeoff/goto/land) just mutate the shared target; the 50 Hz loop
    picks them up. This is what makes 'attach the GCS once, fly many runs' work
    — nothing is torn down between flights."""

    def __init__(self, lab, alt=-5.0, gains=None, reach=2.0):
        self.lab = lab
        gp = dict(DEFAULT_GAINS)
        if gains:
            gp.update(gains)
        self.gp = gp
        self.reach = reach
        self.alt = alt
        self.wps = [(0.0, 0.0)]
        self.wp = 0
        self.iz = 0.0
        self.armed = False
        self.active = False              # guidance engaged (hovering/flying)
        self.last = {}
        self._lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    # -- commands (thread-safe; mutate the shared target) --------------------
    def arm_takeoff(self, alt=None):
        if alt is not None:
            self.alt = alt
        # Disarm and STOP guidance BEFORE respawning. If we reset_pose while the
        # craft is still flying (and the Pilot is still commanding sticks), the
        # teleport to the ground leaves the estimator with the old in-air
        # attitude/velocity; the FC then sees a huge error and trips the
        # bank-angle FAILSAFE, wedging it on the ground. Settle level first.
        with self._lock:
            self.active = False
        self.lab.set_rc(swa=1000, thr=1000)          # disarm → STANDBY
        time.sleep(0.3)
        self.lab.reset_pose((0, 0, -0.05))           # respawn level on the ground
        time.sleep(0.8)                              # let the estimator settle level
        self.lab.set_rc(swa=2000, thr=1000)          # arm gesture (low throttle)
        time.sleep(0.4)
        with self._lock:
            self.wps = [(0.0, 0.0)]
            self.wp = 0
            self.iz = 0.0
            self.armed = True
            self.active = True                        # loop now flies it up + holds

    def goto(self, wps, alt=None):
        with self._lock:
            if alt is not None:
                self.alt = alt
            self.wps = [tuple(map(float, p)) for p in wps] or [(0.0, 0.0)]
            self.wp = 0

    def reached_last(self):
        with self._lock:
            if not self.last:
                return False
            tx, ty = self.wps[-1]
            dx, dy = tx - self.last.get("x", 0), ty - self.last.get("y", 0)
            return self.wp >= len(self.wps) - 1 and (dx * dx + dy * dy) < self.reach * self.reach

    def land(self):
        """Descend to the ground at the current spot, then disarm."""
        with self._lock:
            self.alt = -0.15                          # sink toward ground
        for _ in range(120):                          # ~2.4 s descent
            with self._lock:
                z = self.last.get("z", 0.0)
            if z > -0.3:
                break
            time.sleep(0.02)
        with self._lock:
            self.active = False
            self.armed = False
        self.lab.set_rc(swa=1000, thr=1000)           # disarm
        time.sleep(0.3)

    # -- 50 Hz guidance loop -------------------------------------------------
    def _loop(self):
        dt = 0.02
        nt = time.time()
        while not self.lab._stop.is_set():
            if self.active:
                tr = self.lab.truth()
                if tr:
                    self._step(tr, dt)
            nt += dt
            s = nt - time.time()
            if s > 0:
                time.sleep(s)
            else:
                nt = time.time()

    def _step(self, tr, dt):
        gp = self.gp
        x, y, z = tr["pos"]
        vN, vE, vD = tr["vel"]
        roll, pitch, yaw = quat_to_euler(*tr["quat"])
        with self._lock:
            alt = self.alt
            # advance waypoint if within reach
            tx, ty = self.wps[self.wp]
            eN, eE = tx - x, ty - y
            if (eN * eN + eE * eE) < self.reach * self.reach \
                    and self.wp < len(self.wps) - 1:
                self.wp += 1
                tx, ty = self.wps[self.wp]
                eN, eE = tx - x, ty - y
            # altitude hold (NED z down-positive; alt<0 is up)
            ez = z - alt
            self.iz = max(-0.3, min(0.3, self.iz + ez * dt))
            thr = gp["hover"] + gp["kp_z"] * ez + gp["ki_z"] * self.iz \
                - gp["kd_z"] * (-vD)
            thr = max(0.0, min(1.0, thr))
            # position → speed-capped velocity → damped tilt
            vdes_n = max(-gp["vmax"], min(gp["vmax"], 0.6 * eN))
            vdes_e = max(-gp["vmax"], min(gp["vmax"], 0.6 * eE))
            des_pitch = max(-gp["tilt"], min(gp["tilt"], gp["kd_h"] * (vdes_n - vN)))
            des_roll = max(-gp["tilt"], min(gp["tilt"], gp["kd_h"] * (vdes_e - vE)))
            self.last = dict(x=x, y=y, z=z, vN=vN, vE=vE, vD=vD,
                             roll=roll, pitch=pitch, yaw=yaw, wp=self.wp, thr=thr)
        self.lab.stick(roll=des_roll, pitch=des_pitch, thr=thr, yaw=0.0)
        self.lab.set_rc(swa=2000)                     # keep armed each tick


