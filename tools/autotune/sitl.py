"""Headless SITL stack manager for the PID autotuner.

Launches vsim_d (physics) + vayu_sitl (firmware) sharing one VSIM_FIFO_SUFFIX,
drives RC over a FIFO, reads control-telemetry off the firmware's UART2 pty,
sends CMD_SET_PID / ARM, and runs the vsim_d ctl channel (reset / test-rig /
rates). One stack is launched once and reused across many rollouts.

Wiring (all paths suffixed with VSIM_FIFO_SUFFIX):
  /tmp/vsim_ctl<sfx>        autotuner -> vsim_d   (reset/testrig/rates)
  /tmp/vsim_pose<sfx>       vsim_d   -> autotuner (pose, optional)
  /tmp/vayu_uart2_pty<sfx>  advert file -> the firmware telemetry/command pty
  $VAYU_UART_RC_PATH        autotuner -> firmware (RC CSV frames)
"""

import os
import time
import threading
import subprocess
import termios
import collections

import protocol as P

# RC channel layout (FS-i6 Mode 2): roll, pitch, throttle, yaw, arm, [ch6=acro].
RC_NEUTRAL = [1500, 1500, 1000, 1500, 1000, 1000]


class SitlStack:
    def __init__(self, repo_root, suffix=None, rates=(1000, 8000, 120), quiet=True,
                 geometry=None, world=None, mixer_geometry=None):
        self.root = os.path.abspath(repo_root)
        self.suffix = suffix or f"_at{os.getpid()}"
        self.rates = rates                      # (imu_hz, physics_hz, pose_hz)
        self.quiet = quiet
        self.geometry = geometry                # dict: mass/inertia/motors, or None
        # Firmware-mixer layout (un-rotated): the index-based roll/pitch mix
        # signs must be invariant to a cosmetic body rotation (e.g. rx=180 to
        # right an upside-down mesh), so the mixer takes this un-rotated layout
        # while physics uses `geometry` (rotated). Falls back to `geometry`.
        # Mirrors GeometryEditorWidget::mixerConfig() / SitlStack mixerGeometry.
        self.mixer_geometry = mixer_geometry
        self.world = world                      # dict: gravity/drag/..., or None
        self.tether_k = 0.0                     # >0: soft rig (estimator-aware)
        self.vsim_bin = os.path.join(self.root, "build_vsim", "vsim_d")
        self.sitl_bin = os.path.join(self.root, "build_sitl", "vayu_sitl")
        self.rc_path = f"/tmp/vayu_at_rc{self.suffix}"
        self.ctl_path = f"/tmp/vsim_ctl{self.suffix}"
        self.advert_path = f"/tmp/vayu_uart2_pty{self.suffix}"

        self._procs = []
        self._ctl_fd = -1
        self._pty_fd = -1
        self._rc_master = -1
        self._rc_slave = -1
        self._rc = list(RC_NEUTRAL)
        self._rc_lock = threading.Lock()
        self._dec = P.NavlinkDecoder()
        self._samples = collections.deque(maxlen=20000)
        self._rc_log = collections.deque(maxlen=20000)   # (t, rc copy) excitation
        self._sample_lock = threading.Lock()
        self._last_state = None             # latest sys_state_t from telemetry
        self._last_state_t = 0.0            # monotonic time of that reading
        self._last_mode = None              # latest effective flight mode (0/1)
        self._last_mode_src = None          # 0=RC, 1=GCS
        self._stop = threading.Event()
        self._threads = []

    # -- lifecycle ----------------------------------------------------------
    def start(self):
        for p in (self.vsim_bin, self.sitl_bin):
            if not os.path.exists(p):
                raise FileNotFoundError(f"missing binary: {p} (build it first)")
        # RC must look like a serial tty: the firmware calls tcgetattr() on it,
        # which fails on a plain FIFO. Use a PTY — firmware opens the slave, we
        # write RC frames to the master.
        self._rc_master, self._rc_slave = os.openpty()
        rc_slave = os.ttyname(self._rc_slave)     # path the firmware opens
        self._make_raw(self._rc_master)
        self._make_raw(self._rc_slave)

        env = dict(os.environ)
        env["VSIM_FIFO_SUFFIX"] = self.suffix
        env["VAYU_UART_RC_PATH"] = rc_slave
        out = subprocess.DEVNULL if self.quiet else None

        # vsim_d first so it creates the pwm/imu/pose/ctl FIFOs the firmware opens.
        self._procs.append(subprocess.Popen([self.vsim_bin], env=env, stderr=out, stdout=out))
        self._wait_path(self.ctl_path, 5.0)
        self._ctl_fd = os.open(self.ctl_path, os.O_RDWR | os.O_NONBLOCK)
        self.send_ctl(P.ctl_rates(*self.rates))
        if self.geometry:                       # tune the actual airframe
            g = self.geometry
            self.send_ctl(P.ctl_geometry(g["mass"], g["inertia"], g["motors"]))
        if self.world:                          # tune the actual environment (drag!)
            w = self.world
            self.send_ctl(P.ctl_world(
                gravity=w.get("gravity", 9.81), ground_z=w.get("ground_z", 0.0),
                restitution=w.get("restitution", 0.3),
                linear_drag=w.get("linear_drag", 0.10),
                angular_drag=w.get("angular_drag", 0.005),
                ground_right_gain=w.get("ground_right_gain", 8.0),
                ground_right_damp=w.get("ground_right_damp", 3.0)))

        # firmware host: opens RC FIFO, creates the UART2 pty, boots to STANDBY.
        self._procs.append(subprocess.Popen([self.sitl_bin], env=env, stderr=out, stdout=out))

        self._start_rc_writer()
        self._open_pty()
        self._start_pty_reader()
        # Gate on the firmware actually booting (telemetry flowing + a system
        # state reported) instead of a blind sleep. A slow/loaded cold start
        # used to leave the first wait_level() sample-starved, failing every
        # arm attempt ("arm not confirmed") before the host was even ready.
        self.wait_ready(timeout=8.0)
        # §10.5 command gate: the FC rejects EVERY command (set_pid, geometry,
        # flight mode) with TEMPORARILY_REJECTED until the GCS has disciplined
        # its clock at least once. Discipline it now, before the first command
        # below — otherwise the harness silently runs the loaded/default plant
        # (the bug that made every prior sweep inert). The GCS does this via its
        # time-sync handshake; we mirror it with a single REQUEST.
        self.sync_clock()
        if self.geometry:
            # Set the FIRMWARE mixer from the UN-ROTATED layout (mixer_geometry
            # if given, else geometry): the index-based roll/pitch mix signs are
            # a wiring property and must not flip under a cosmetic body rotation.
            mix = self.mixer_geometry or self.geometry
            self.send_navlink(P.set_motor_geometry_command(mix["motors"]))
            time.sleep(0.1)
        return self

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        for fd in (self._ctl_fd, self._pty_fd, self._rc_master, self._rc_slave):
            if fd >= 0:
                try: os.close(fd)
                except OSError: pass
        for p in self._procs:
            p.terminate()
        for p in self._procs:
            try: p.wait(timeout=2.0)
            except subprocess.TimeoutExpired: p.kill()

    def __enter__(self): return self.start()
    def __exit__(self, *a): self.stop()

    # -- vsim_d ctl ---------------------------------------------------------
    def send_ctl(self, frame: bytes):
        if self._ctl_fd >= 0:
            self._write_all(self._ctl_fd, frame)

    def reset(self, pos=(0, 0, -0.05), seed=0):
        # seed!=0 => deterministic sensor-noise reset in vsim_d (repeatable cost).
        self.send_ctl(P.ctl_reset(pos=pos, seed=seed))

    def set_testrig(self, on, pos=(0, 0, -0.05), tether_k=0.0):
        self.send_ctl(P.ctl_testrig(on, pos=pos, tether_k=tether_k))

    # -- firmware command uplink (over the pty) -----------------------------
    def send_navlink(self, pkt: bytes):
        if self._pty_fd >= 0:
            self._write_all(self._pty_fd, pkt)

    def sync_clock(self, seq=1):
        """Send a TIME_SYNC REQUEST so the FC's §10.5 command gate opens
        (time_sync_is_synced() -> commands ACCEPTED instead of
        TEMPORARILY_REJECTED). Idempotent; safe to re-send."""
        self.send_navlink(P.time_sync_command(seq))
        time.sleep(0.15)

    def set_pid(self, controller, axis, kp, ki, kd, kff):
        self.send_navlink(P.set_pid_command(controller, axis, kp, ki, kd, kff))

    def set_gyro_lpf(self, axis, rc):
        self.send_navlink(P.set_gyro_lpf_command(axis, rc))

    def set_flight_mode(self, mode):
        """0=stabilise/angle, 1=acro, 2=release to RC switch."""
        self.send_navlink(P.set_flight_mode_command(mode))

    # -- RC -----------------------------------------------------------------
    def set_rc(self, roll=None, pitch=None, thr=None, yaw=None, arm=None, ch6=None):
        with self._rc_lock:
            if roll is not None: self._rc[0] = int(roll)
            if pitch is not None: self._rc[1] = int(pitch)
            if thr is not None: self._rc[2] = int(thr)
            if yaw is not None: self._rc[3] = int(yaw)
            if arm is not None: self._rc[4] = int(arm)
            if ch6 is not None: self._rc[5] = int(ch6)

    def wait_state(self, mask, timeout=2.0):
        """Block until telemetry reports a system state matching `mask` (a
        sys_state_t bit value), or timeout. Returns the matched state, or None.
        Requires a *fresh* reading (after this call started) so a stale ARMED
        from a prior rollout can't satisfy it."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self._last_state is not None and self._last_state_t >= t0:
                if self._last_state & mask:
                    return self._last_state
            time.sleep(0.01)
        return None

    def arm(self, timeout=2.0):
        """Arm and CONFIRM the firmware actually reached ARMED before returning.
        Replaces a blind sleep that let silent arm failures produce phantom
        no-response rollouts. Returns True iff ARMED was observed in telemetry.

        First confirms STANDBY with the arm switch LOW: a prior divergent rollout
        (common mid line-search in the `structured` optimizer, which probes
        aggressive gains) can latch the firmware in FAILSAFE, and it only arms
        from STANDBY. Dropping the switch + a clean STANDBY edge clears that, so
        a good gain set after a bad probe doesn't get misrecorded as a failure."""
        # Arm switch low first → clear any latched FAILSAFE, settle into STANDBY.
        self.set_rc(roll=1500, pitch=1500, thr=1000, yaw=1500, arm=1000)
        self.wait_state(P.SYSTEM_STATE_STANDBY, timeout=1.5)
        # Arm requires throttle < 1100 in STANDBY with the arm switch high.
        self.set_rc(roll=1500, pitch=1500, thr=1000, yaw=1500, arm=2000)
        return self.wait_state(P.SYSTEM_STATE_ARMED, timeout) is not None

    def disarm(self):
        self.set_rc(thr=1000, arm=1000)
        # Confirm we left ARMED so the next arm sees a clean STANDBY->ARMED edge
        # (which is what hard-resets the firmware rate-PID integrators).
        self.wait_state(P.SYSTEM_STATE_STANDBY, timeout=1.0)

    # -- telemetry ----------------------------------------------------------
    def clear_samples(self):
        with self._sample_lock:
            self._samples.clear()

    def snapshot(self):
        with self._sample_lock:
            return list(self._samples)

    def rc_snapshot(self):
        """[(t, [roll,pitch,thr,yaw,arm,ch6]), ...] — the commanded RC, so the
        plotter can show the excitation that drove a rollout."""
        with self._sample_lock:
            return list(self._rc_log)

    def wait_level(self, deg=6.0, timeout=2.5):
        """Wait until the estimator reports near-level (|roll|,|pitch| < deg).
        After a divergent rollout the attitude estimator is 'degraded' and the
        firmware refuses to arm (arm_preconditions_met); a reset levels the sim
        but the estimator needs a moment to re-converge. Returns True if level."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self._sample_lock:
                last = self._samples[-1][1] if self._samples else None
            if last is not None:
                if (abs(last["roll_angle_curr"]) < deg
                        and abs(last["pitch_angle_curr"]) < deg):
                    return True
            time.sleep(0.02)
        return False

    def wait_ready(self, timeout=8.0):
        """Block until the firmware host is up: telemetry samples are flowing
        AND a system state has been reported (booted to STANDBY). Replaces the
        old blind 1 s startup sleep so a slow or CPU-loaded cold start (e.g.
        launched alongside the GCS) can't sample-starve the first arm. Returns
        True if telemetry started; proceeds on a brief grace if state lags."""
        t0 = time.monotonic()
        first_sample_t = None
        while time.monotonic() - t0 < timeout:
            with self._sample_lock:
                have_sample = bool(self._samples)
            if have_sample:
                if first_sample_t is None:
                    first_sample_t = time.monotonic()
                if self._last_state is not None:
                    return True
                # Telemetry is flowing; give the state field a short grace
                # then proceed rather than block the whole timeout.
                if time.monotonic() - first_sample_t > 1.0:
                    return True
            time.sleep(0.02)
        return first_sample_t is not None

    def collect(self, duration, hover_thr=None):
        """Return [(t, ct_dict), ...] captured over `duration` seconds (t monotonic)."""
        if hover_thr is not None:
            self.set_rc(thr=hover_thr)
        self.clear_samples()
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration:
            time.sleep(0.01)
        with self._sample_lock:
            return list(self._samples)

    # -- internals ----------------------------------------------------------
    @staticmethod
    def _write_all(fd, data):
        """Write every byte, retrying on EAGAIN. A single os.write() on a
        non-blocking fd can short-write or raise BlockingIOError when the pipe
        buffer is full, silently truncating a SET_PID / ctl command and leaving
        the eval running on stale gains. Loop until the whole frame is out."""
        mv = memoryview(data)
        deadline = time.monotonic() + 0.5
        while mv:
            try:
                n = os.write(fd, mv)
                mv = mv[n:]
            except BlockingIOError:
                if time.monotonic() > deadline:
                    return False
                time.sleep(0.001)
            except OSError:
                return False
        return True

    @staticmethod
    def _make_raw(fd):
        try:
            attrs = termios.tcgetattr(fd)
            attrs[3] &= ~(termios.ICANON | termios.ECHO | termios.ISIG)
            attrs[0] &= ~(termios.ICRNL | termios.INLCR)
            attrs[1] &= ~termios.OPOST
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except termios.error:
            pass

    @staticmethod
    def _wait_path(path, timeout):
        t0 = time.monotonic()
        while not os.path.exists(path):
            if time.monotonic() - t0 > timeout:
                raise TimeoutError(f"timed out waiting for {path}")
            time.sleep(0.02)

    def _open_pty(self):
        self._wait_path(self.advert_path, 5.0)
        with open(self.advert_path) as f:
            slave = f.read().strip()
        self._wait_path(slave, 5.0)
        fd = os.open(slave, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(fd)
            attrs[3] &= ~(termios.ICANON | termios.ECHO | termios.ISIG)
            attrs[0] &= ~(termios.ICRNL | termios.INLCR)
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except termios.error:
            pass
        self._pty_fd = fd

    def _start_rc_writer(self):
        def run():
            while not self._stop.is_set():
                with self._rc_lock:
                    rc = list(self._rc)
                    line = ",".join(str(c) for c in rc) + "\n"
                if not self._write_all(self._rc_master, line.encode()):
                    time.sleep(0.05); continue   # a torn RC line mis-parses
                with self._sample_lock:          # log the commanded excitation
                    self._rc_log.append((time.monotonic(), rc))
                time.sleep(0.02)             # ~50 Hz
        self._spawn(run)

    def _start_pty_reader(self):
        def run():
            while not self._stop.is_set():
                try:
                    data = os.read(self._pty_fd, 4096)
                except BlockingIOError:
                    time.sleep(0.003); continue
                except OSError:
                    time.sleep(0.01); continue
                if not data:
                    time.sleep(0.003); continue
                now = time.monotonic()
                for ct in self._dec.feed(data):
                    if "sys_state" in ct:           # system-state, not a CT sample
                        self._last_state = int(ct["sys_state"])
                        self._last_state_t = now
                        continue
                    if "flight_mode" in ct:         # flight-mode status, not a CT sample
                        self._last_mode = ct["flight_mode"]
                        self._last_mode_src = ct["flight_mode_src"]
                        continue
                    with self._sample_lock:
                        self._samples.append((now, ct))
        self._spawn(run)

    def _spawn(self, fn):
        t = threading.Thread(target=fn, daemon=True)
        t.start()
        self._threads.append(t)
