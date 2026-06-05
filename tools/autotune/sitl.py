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
    def __init__(self, repo_root, suffix=None, rates=(400, 2000, 120), quiet=True,
                 geometry=None):
        self.root = os.path.abspath(repo_root)
        self.suffix = suffix or f"_at{os.getpid()}"
        self.rates = rates                      # (imu_hz, physics_hz, pose_hz)
        self.quiet = quiet
        self.geometry = geometry                # dict: mass/inertia/motors, or None
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
        self._sample_lock = threading.Lock()
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

        # firmware host: opens RC FIFO, creates the UART2 pty, boots to STANDBY.
        self._procs.append(subprocess.Popen([self.sitl_bin], env=env, stderr=out, stdout=out))

        self._start_rc_writer()
        self._open_pty()
        self._start_pty_reader()
        time.sleep(1.0)                          # let the loops spin up
        if self.geometry:
            # Also set the FIRMWARE mixer from the same motor layout so the
            # control mix matches the physics (keeps the loop stable for any
            # quad layout, not just the firmware's default numbering).
            self.send_navlink(P.set_motor_geometry_command(self.geometry["motors"]))
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
            try: os.write(self._ctl_fd, frame)
            except OSError: pass

    def reset(self, pos=(0, 0, -0.05)):
        self.send_ctl(P.ctl_reset(pos=pos))

    def set_testrig(self, on, pos=(0, 0, -0.05)):
        self.send_ctl(P.ctl_testrig(on, pos=pos))

    # -- firmware command uplink (over the pty) -----------------------------
    def send_navlink(self, pkt: bytes):
        if self._pty_fd >= 0:
            try: os.write(self._pty_fd, pkt)
            except OSError: pass

    def set_pid(self, controller, axis, kp, ki, kd, kff):
        self.send_navlink(P.set_pid_command(controller, axis, kp, ki, kd, kff))

    def set_gyro_lpf(self, axis, rc):
        self.send_navlink(P.set_gyro_lpf_command(axis, rc))

    # -- RC -----------------------------------------------------------------
    def set_rc(self, roll=None, pitch=None, thr=None, yaw=None, arm=None, ch6=None):
        with self._rc_lock:
            if roll is not None: self._rc[0] = int(roll)
            if pitch is not None: self._rc[1] = int(pitch)
            if thr is not None: self._rc[2] = int(thr)
            if yaw is not None: self._rc[3] = int(yaw)
            if arm is not None: self._rc[4] = int(arm)
            if ch6 is not None: self._rc[5] = int(ch6)

    def arm(self):
        # Arm requires throttle < 1100 in STANDBY with the arm switch high.
        self.set_rc(roll=1500, pitch=1500, thr=1000, yaw=1500, arm=2000)
        time.sleep(0.6)

    def disarm(self):
        self.set_rc(thr=1000, arm=1000)
        time.sleep(0.3)

    # -- telemetry ----------------------------------------------------------
    def clear_samples(self):
        with self._sample_lock:
            self._samples.clear()

    def snapshot(self):
        with self._sample_lock:
            return list(self._samples)

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
                    line = ",".join(str(c) for c in self._rc) + "\n"
                try:
                    os.write(self._rc_master, line.encode())
                except OSError:
                    time.sleep(0.05); continue
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
                for ct in self._dec.feed(data):
                    with self._sample_lock:
                        self._samples.append((time.monotonic(), ct))
        self._spawn(run)

    def _spawn(self, fn):
        t = threading.Thread(target=fn, daemon=True)
        t.start()
        self._threads.append(t)
