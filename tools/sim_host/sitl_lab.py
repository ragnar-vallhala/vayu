#!/usr/bin/env python3
"""sitl_lab.py — programmatic closed-loop SITL harness for the REAL firmware.

The point: drive and test the *actual* flight-controller logic (the same
estimator, angle/rate cascade, mixer, arming and telemetry that run on the FC)
with precise, repeatable, scripted inputs — instead of flying by hand on an RC
stick. Nothing here re-implements control; it only stubs the sensors and RC and
lets physics handle the actuation response.

Topology (all headless, isolated by a per-run VSIM_FIFO_SUFFIX):

    sitl_lab.py ──RC CSV (pty)──▶ vayu_sitl  ──PWM (fifo)──▶ vsim_d
         ▲                        (real FC)                  (physics)
         │                            │                          │
         └──NavLink telem (UART2 pty)─┘     IMU (fifo) ◀──────────┘
         └──────────── pose ground truth (fifo) ◀────────────────┘

So we DRIVE: RC sticks/arm/mode (→ real arm logic + attitude setpoints), plus
env/disturbances via vsim_d ctl (wind, world). We READ: physics ground truth
(vsim_d pose) AND the FC's own NavLink telemetry (its estimate/state/etc.),
decoded with the generated navlink codec.

Example:
  VSIM_BIN_PATH=tools/vsim/build/vsim_d \
  VAYU_SITL_BIN=tools/sim_host/build_sitl/vayu_sitl \
  python3 tools/sim_host/sitl_lab.py --csv /tmp/roll_step.csv
"""
import argparse
import math
import os
import struct
import subprocess
import sys
import threading
import time

# --- locate + import the generated NavLink codec ----------------------------
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_ROOT, "navlink", "sim"))
sys.path.insert(0, os.path.join(_ROOT, "navlink", "generated", "python"))
import frame as nlframe          # noqa: E402  navlink/sim/frame.py
import navlink_msgs as nlmsg     # noqa: E402  generated codec

# msgid -> message class (for decoding telemetry payloads).
MSG_BY_ID = {c.MSGID: c for c in vars(nlmsg).values()
             if isinstance(c, type) and hasattr(c, "MSGID") and hasattr(c, "unpack")}

# --- vsim_d wire bits (subset of tools/vsim/tests/sim_drive.py) -------------
MAGIC, VERSION = 0x4D495356, 3
FRAME_POSE = 3
CTL_RESET, CTL_SET_WORLD, CTL_SET_TESTRIG, CTL_SET_WIND = 1, 6, 12, 14
HDR = struct.Struct("<IHHII")
POSE = struct.Struct("<II3f4f3f3f4f4f3fffffff")   # v3 body, 128 B
assert HDR.size == 16 and POSE.size == 128


def _vhdr(typ, plen, seq=0):
    return HDR.pack(MAGIC, VERSION, typ, plen, seq)


def _ctl(subtype, body):
    body = body[:256].ljust(256, b"\x00")
    payload = struct.pack("<II", subtype, 0) + body
    return _vhdr(4, len(payload)) + payload


def _reset(pos=(0, 0, -0.05), seed=1):
    return _ctl(CTL_RESET, struct.pack("<3f4f3f3fI", *pos, 1, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, seed))


def _world(gravity=9.81, ground_z=50.0, lin_drag=0.10):
    # ground_z far below so a rig/airborne craft never clamps unexpectedly.
    return _ctl(CTL_SET_WORLD, struct.pack("<7f", gravity, ground_z, 0.0,
                                           lin_drag, 0.005, 40.0, 6.0))


def _testrig(enable, pos=(0, 0, -1.0), tether_k=0.0):
    return _ctl(CTL_SET_TESTRIG, struct.pack("<i3ff", 1 if enable else 0,
                                             pos[0], pos[1], pos[2], tether_k))


def _wind(steady=(0, 0, 0), turb=0.0, enable=True):
    return _ctl(CTL_SET_WIND, struct.pack("<3f4fi", steady[0], steady[1],
                steady[2], 0.0, 0.0, turb, 1.0, 1 if enable else 0))


def quat_to_euler(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(v) for v in (roll, pitch, yaw)]


class SitlLab:
    """Spawns vsim_d + vayu_sitl, streams RC, reads truth + FC telemetry."""

    def __init__(self, rig=False, wind=None, turb=0.0, rc_hz=50):
        self.suffix = f"_lab{os.getpid()}"
        self.env = dict(os.environ, VSIM_FIFO_SUFFIX=self.suffix)
        self.paths = {n: f"/tmp/vsim_{n}{self.suffix}" for n in
                      ("pwm", "imu", "pose", "ctl")}
        self.uart_advert = f"/tmp/vayu_uart2_pty{self.suffix}"
        self._stop = threading.Event()
        # RC frame: roll,pitch,throttle,yaw,SwA(arm),aux  (us; centres 1500)
        self._rc = [1500, 1500, 1000, 1500, 1000, 1500]
        self._rc_lock = threading.Lock()
        self.rc_hz = rc_hz
        self.telem = {}            # latest decoded msg by class name
        self.telem_counts = {}
        self._truth = None

        vsim_bin = os.environ.get("VSIM_BIN_PATH", "tools/vsim/build/vsim_d")
        sitl_bin = os.environ.get("VAYU_SITL_BIN",
                                  "tools/sim_host/build_sitl/vayu_sitl")
        for b in (vsim_bin, sitl_bin):
            if not os.path.exists(b):
                raise FileNotFoundError(f"missing binary: {b}")

        # 1) physics daemon
        self.vsim = subprocess.Popen([vsim_bin], env=self.env,
                                     stderr=subprocess.DEVNULL)
        self._await(lambda: all(os.path.exists(p) for p in self.paths.values()),
                    "vsim_d FIFOs")
        self.ctl_fd = os.open(self.paths["ctl"], os.O_RDWR | os.O_NONBLOCK)
        self.pose_fd = os.open(self.paths["pose"], os.O_RDWR | os.O_NONBLOCK)

        # configure plant before the FC boots
        os.write(self.ctl_fd, _reset())
        os.write(self.ctl_fd, _world())
        if rig:
            os.write(self.ctl_fd, _testrig(True))
        if wind or turb:
            os.write(self.ctl_fd, _wind(wind or (0, 0, 0), turb, True))

        # 2) RC pty + real firmware host
        self.rc_master, rc_slave = os.openpty()
        self.env["VAYU_UART_RC_PATH"] = os.ttyname(rc_slave)
        self.sitl = subprocess.Popen([sitl_bin], env=self.env,
                                     stderr=subprocess.DEVNULL)
        threading.Thread(target=self._rc_thread, daemon=True).start()

        # 3) FC telemetry: wait for the advertised UART2 pty slave path
        self._await(lambda: os.path.exists(self.uart_advert), "UART2 advert")
        with open(self.uart_advert) as f:
            self.uart_path = f.read().strip()
        self.uart_fd = os.open(self.uart_path, os.O_RDWR | os.O_NONBLOCK)
        threading.Thread(target=self._telem_thread, daemon=True).start()

    # -- lifecycle -----------------------------------------------------------
    def _await(self, cond, what, timeout=8.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if cond():
                return
            time.sleep(0.02)
        raise TimeoutError(f"timed out waiting for {what}")

    def close(self):
        self._stop.set()
        for p in (getattr(self, "sitl", None), getattr(self, "vsim", None)):
            if p:
                p.terminate()
                try:
                    p.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    p.kill()
        for p in self.paths.values():
            try:
                os.unlink(p)
            except OSError:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    # -- RC input (drive the real arm logic + setpoints) ---------------------
    def _rc_thread(self):
        period = 1.0 / self.rc_hz
        while not self._stop.is_set():
            with self._rc_lock:
                line = ",".join(str(int(v)) for v in self._rc) + "\n"
            try:
                os.write(self.rc_master, line.encode())
            except OSError:
                pass
            time.sleep(period)

    def set_rc(self, roll=None, pitch=None, thr=None, yaw=None, swa=None):
        with self._rc_lock:
            for i, v in enumerate((roll, pitch, thr, yaw, swa)):
                if v is not None:
                    self._rc[i] = v

    def stick(self, roll=0.0, pitch=0.0, thr=0.0, yaw=0.0):
        """Normalised inputs: roll/pitch/yaw in [-1,1], thr in [0,1]."""
        us = lambda c: int(1500 + 500 * max(-1.0, min(1.0, c)))
        self.set_rc(roll=us(roll), pitch=us(pitch), yaw=us(yaw),
                    thr=int(1000 + 1000 * max(0.0, min(1.0, thr))))

    def arm(self, settle=1.2):
        """SwA up + low throttle → real FC arm transition."""
        self.set_rc(swa=2000, thr=1000, roll=1500, pitch=1500, yaw=1500)
        time.sleep(settle)

    def disarm(self):
        self.set_rc(swa=1000, thr=1000)

    # -- read FC telemetry (NavLink over UART2) ------------------------------
    def _telem_thread(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                data = os.read(self.uart_fd, 65536)
            except (BlockingIOError, OSError):
                data = b""
            if data:
                buf += data
                self._parse_frames(buf)
            else:
                time.sleep(0.002)

    def _parse_frames(self, buf):
        i = 0
        while i < len(buf):
            if buf[i] != nlframe.SYNC:
                i += 1
                continue
            if i + nlframe.HDR_LEN + 2 > len(buf):
                break
            plen = buf[i + 2]
            total = nlframe.HDR_LEN + plen + 2
            if i + total > len(buf):
                break
            d = nlframe.decode(bytes(buf[i:i + total]))
            if d.ok:
                cls = MSG_BY_ID.get(d.msgid)
                if cls:
                    name = cls.__name__
                    self.telem[name] = cls.unpack(d.payload)
                    self.telem_counts[name] = self.telem_counts.get(name, 0) + 1
                i += total
            else:
                i += 1                # resync on the next SYNC byte
        del buf[:i]

    # -- read physics ground truth (vsim_d pose) -----------------------------
    def truth(self):
        chunk = b""
        try:
            while True:
                part = os.read(self.pose_fd, 65536)
                if not part:
                    break
                chunk += part
        except BlockingIOError:
            pass
        pos = 0
        while pos + HDR.size <= len(chunk):
            magic, ver, t, plen, seq = HDR.unpack_from(chunk, pos)
            if magic != MAGIC:
                pos += 1
                continue
            if pos + HDR.size + plen > len(chunk):
                break
            if t == FRAME_POSE:
                p = POSE.unpack_from(chunk, pos + HDR.size)
                self._truth = {
                    "pos": p[2:5], "quat": p[5:9], "vel": p[9:12],
                    "omega": p[12:15], "motor_omega": p[15:19],
                    "wind": p[23:26],
                }
            pos += HDR.size + plen
        return self._truth


def demo(args):
    """Arm the real FC on a rig, command a roll-angle step, record the actual
    controller's tracking response (ground truth) + FC telemetry to CSV."""
    with SitlLab(rig=True, wind=tuple(args.wind), turb=args.turb) as lab:
        print(f"suffix={lab.suffix}  rc={lab.env['VAYU_UART_RC_PATH']}  "
              f"uart2={lab.uart_path}")
        lab.arm(settle=1.5)
        lab.stick(thr=0.5)                     # mid throttle, level
        rows = []
        t0 = time.time()
        csv = open(args.csv, "w") if args.csv else None
        if csv:
            csv.write("t,cmd_roll,roll,pitch,yaw,wx,wy,wz,nav_state\n")
        while time.time() - t0 < args.secs:
            t = time.time() - t0
            cmd_roll = 0.4 if 1.0 <= t < 2.5 else 0.0   # roll-angle step+return
            lab.stick(roll=cmd_roll, thr=0.5)
            tr = lab.truth()
            hb = lab.telem.get("Heartbeat")
            nav = getattr(hb, "nav_state", -1) if hb else -1
            if tr:
                roll, pitch, yaw = quat_to_euler(*tr["quat"])
                wx, wy, wz = tr["omega"]
                rows.append((t, cmd_roll, roll, pitch, yaw, wx, wy, wz, nav))
                if csv:
                    csv.write("%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%d\n"
                              % (t, cmd_roll, roll, pitch, yaw, wx, wy, wz, nav))
            time.sleep(0.02)
        if csv:
            csv.close()

        print(f"recorded {len(rows)} samples")
        print("FC telemetry decoded:",
              ", ".join(f"{k}×{v}" for k, v in sorted(lab.telem_counts.items()))
              or "(none)")
        if rows:
            peak = max(rows, key=lambda r: abs(r[2]))
            print(f"  commanded roll-step 0.4 (~ {0.4*30:.0f}° at 30°/full)")
            print(f"  peak true roll reached: {peak[2]:+.1f}°  "
                  f"(at t={peak[0]:.2f}s)")
            last = rows[-1]
            print(f"  final attitude: roll={last[2]:+.1f}° pitch={last[3]:+.1f}° "
                  f"yaw={last[4]:+.1f}°  nav_state={last[8]}")
        if args.csv:
            print(f"  CSV -> {args.csv}")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--wind", type=float, nargs=3, default=[0, 0, 0],
                    metavar=("N", "E", "D"))
    ap.add_argument("--turb", type=float, default=0.0)
    ap.add_argument("--csv", type=str, default="")
    sys.exit(demo(ap.parse_args()))

