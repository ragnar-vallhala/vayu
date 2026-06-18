"""Pilot — continuous outer-loop guidance (takeoff / goto / land / station-keep).

Always writes RC STICKS only (never position) except the explicit takeoff/land
respawn, so the flight stays genuine firmware-in-the-loop physics. Carved
verbatim from tools/sim_host/sitl_lab.py.
"""
import threading
import time

from .transport.vsim import quat_to_euler


DEFAULT_GAINS = dict(
    # altitude PID (NED z) — unchanged, hover hold is already tight.
    kp_z=0.05, ki_z=0.02, kd_z=0.05, hover=0.36,
    # horizontal cascade: position P (kp_pos) + velocity feedforward -> desired
    # velocity (vmax cap); velocity error * kd_h + position integral (ki_pos,
    # clamped i_lim) -> commanded tilt (tilt cap). tilt raised from the old 0.30
    # (~2.7 deg, ~0.5 m/s^2 max accel — the real sluggishness) to 0.6 (~5.5 deg).
    kp_pos=0.9, kd_h=0.85, ki_pos=0.10, i_lim=3.0, tilt=0.6, vmax=2.0)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def guidance_outputs(gp, ez, iz, vD, eN, eE, vN, vE, dt,
                     vff_n=0.0, vff_e=0.0, iN=0.0, iE=0.0):
    """Pure cascade-guidance step (no I/O, no state) — unit-testable.

    Inputs: gains `gp`; altitude error `ez` (NED z − target, +ve = below target);
    altitude integrator `iz`; down-velocity `vD`; horizontal position errors
    `eN/eE`; horizontal velocities `vN/vE`; timestep `dt`; per-axis velocity
    feedforward `vff_n/vff_e` (the moving setpoint's own velocity); horizontal
    position integrators `iN/iE`.
    Returns (throttle[0..1], des_roll, des_pitch, iz, iN, iE). yaw held at 0.
    """
    iz = _clamp(iz + ez * dt, -0.3, 0.3)
    thr = gp["hover"] + gp["kp_z"] * ez + gp["ki_z"] * iz - gp["kd_z"] * (-vD)
    thr = _clamp(thr, 0.0, 1.0)

    # position error (+ path feedforward) -> desired velocity, capped.
    vdes_n = _clamp(gp["kp_pos"] * eN + vff_n, -gp["vmax"], gp["vmax"])
    vdes_e = _clamp(gp["kp_pos"] * eE + vff_e, -gp["vmax"], gp["vmax"])

    # position integral (anti-windup applied below).
    iN_in, iE_in = iN, iE
    iN = _clamp(iN + eN * dt, -gp["i_lim"], gp["i_lim"])
    iE = _clamp(iE + eE * dt, -gp["i_lim"], gp["i_lim"])

    # velocity error * kd_h + position integral -> commanded tilt, capped.
    raw_pitch = gp["kd_h"] * (vdes_n - vN) + gp["ki_pos"] * iN
    raw_roll = gp["kd_h"] * (vdes_e - vE) + gp["ki_pos"] * iE
    des_pitch = _clamp(raw_pitch, -gp["tilt"], gp["tilt"])
    des_roll = _clamp(raw_roll, -gp["tilt"], gp["tilt"])

    # conditional-integration anti-windup: when the tilt is saturated, HOLD the
    # integrator at its prior value (don't accumulate) so it can't overshoot on
    # arrival.
    if des_pitch != raw_pitch:
        iN = iN_in
    if des_roll != raw_roll:
        iE = iE_in
    return thr, des_roll, des_pitch, iz, iN, iE


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
        self.iN = 0.0                    # horizontal position integrators
        self.iE = 0.0
        self._prev_tgt = None            # for velocity feedforward (moving sp)
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
            self.iN = self.iE = 0.0
            self._prev_tgt = None
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
            # velocity feedforward: the setpoint's OWN velocity, from the
            # per-tick motion of the (possibly moving) target. A waypoint switch
            # jumps the target for one tick — vmax-clamped so that can't spike.
            vff_n = vff_e = 0.0
            if self._prev_tgt is not None:
                vff_n = _clamp((tx - self._prev_tgt[0]) / dt,
                               -gp["vmax"], gp["vmax"])
                vff_e = _clamp((ty - self._prev_tgt[1]) / dt,
                               -gp["vmax"], gp["vmax"])
            self._prev_tgt = (tx, ty)
            # altitude hold (NED z down-positive; alt<0 is up) + position cascade
            ez = z - alt
            thr, des_roll, des_pitch, self.iz, self.iN, self.iE = \
                guidance_outputs(gp, ez, self.iz, vD, eN, eE, vN, vE, dt,
                                 vff_n, vff_e, self.iN, self.iE)
            self.last = dict(x=x, y=y, z=z, vN=vN, vE=vE, vD=vD,
                             roll=roll, pitch=pitch, yaw=yaw, wp=self.wp, thr=thr)
        self.lab.stick(roll=des_roll, pitch=des_pitch, thr=thr, yaw=0.0)
        self.lab.set_rc(swa=2000)                     # keep armed each tick


