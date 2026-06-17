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
import tty

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


CTL_SET_GEOMETRY = 5


def _read_gcs_conf(path):
    """Parse the [simulator] geometry\\* and world\\* keys from the GCS's
    QSettings .conf so the headless run flies the SAME vveh/vworld."""
    g, w = {}, {}
    sect = None
    try:
        for ln in open(path):
            ln = ln.strip()
            if ln.startswith("[") and ln.endswith("]"):
                sect = ln[1:-1]
                continue
            if sect != "simulator" or "=" not in ln:
                continue
            k, v = ln.split("=", 1)
            if k.startswith("geometry\\"):
                g[k[len("geometry\\"):]] = v
            elif k.startswith("world\\"):
                w[k[len("world\\"):]] = v
    except OSError:
        pass
    return g, w


def _geometry_frame(g):
    """Pack vsim_ctl_geometry_t from the parsed geometry dict (54 floats)."""
    f = lambda k, d=0.0: float(g.get(k, d))
    body = struct.pack("<f", f("mass", 1.0))
    body += struct.pack("<9f", *[f("I%d" % i) for i in range(9)])
    for i in range(4):
        p = "m%d_" % i
        body += struct.pack("<3f", f(p + "px"), f(p + "py"), f(p + "pz"))
        body += struct.pack("<3f", f(p + "ax"), f(p + "ay"), f(p + "az", 1.0))
        body += struct.pack("<5f", f(p + "spin", 1.0), f(p + "kt", 1.522e-5),
                            f(p + "km", 2.44e-7), f(p + "wmax", 1200.0),
                            f(p + "tau", 0.0125))
    return _ctl(CTL_SET_GEOMETRY, body)


def quat_to_euler(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(v) for v in (roll, pitch, yaw)]


class SitlLab:
    """Spawns vsim_d + vayu_sitl, streams RC, reads truth + FC telemetry."""

    # Default advert file the Navigator toolbar reads to auto-offer a SITL pty.
    GCS_ADVERT = "/tmp/vayu_uart2_pty"

    def __init__(self, rig=False, wind=None, turb=0.0, rc_hz=50, gcs=False,
                 attach=False, conf=None):
        # attach=True → run on the DEFAULT /tmp/vsim_* paths so the GCS's
        # "Attach Ext" can read /tmp/vsim_pose and render this run.
        self.suffix = "" if attach else f"_lab{os.getpid()}"
        self.gcs = gcs
        self.gcs_master = None
        self.gcs_path = None
        self._advertised = False
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
        # Match the GCS's selected vehicle + world (vveh/vworld) if given.
        g, w = _read_gcs_conf(conf) if conf else ({}, {})
        if g:
            os.write(self.ctl_fd, _geometry_frame(g))
        if w:
            os.write(self.ctl_fd, _world(
                gravity=float(w.get("gravity", 9.81)),
                ground_z=float(w.get("ground_z", 0.0)),
                lin_drag=float(w.get("linear_drag", 0.10))))
        else:
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

        # 4) Optional GCS tap: a second pty the Navigator can connect to. The
        # harness stays the SOLE reader of the FC pty (so its own decode keeps
        # working) and transparently forwards bytes both ways, so the operator
        # can monitor/record live in the GCS while the harness drives + checks.
        if self.gcs:
            self.gcs_master, gcs_slave = os.openpty()
            self.gcs_path = os.ttyname(gcs_slave)
            # RAW line discipline: binary NavLink telemetry must pass through
            # untouched (no NL/CR translation, no echo) — exactly what the FC's
            # own UART2 pty does via cfmakeraw. Cooked mode corrupts framing.
            tty.setraw(self.gcs_master)
            os.close(gcs_slave)                       # GCS reopens it by path
            os.set_blocking(self.gcs_master, False)
            # Outbound buffer for FC→GCS: a non-blocking pty master takes
            # partial writes, so a dedicated drainer preserves the remainder
            # (truncating mid-frame would corrupt framing — that's what dropped
            # all but the low-rate streams in the first cut).
            self._gcs_out = bytearray()
            self._gcs_out_lock = threading.Lock()
            # Advertise so the Navigator toolbar auto-offers it (SITL UART2).
            try:
                with open(self.GCS_ADVERT, "w") as f:
                    f.write(self.gcs_path + "\n")
                self._advertised = True
            except OSError:
                pass
            threading.Thread(target=self._gcs_rx_thread, daemon=True).start()
            threading.Thread(target=self._gcs_tx_thread, daemon=True).start()

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
        # Remove our advert only if it still points at us (don't clobber a real
        # vayu_sitl's advert written after ours).
        if self._advertised:
            try:
                with open(self.GCS_ADVERT) as f:
                    if f.read().strip() == self.gcs_path:
                        os.unlink(self.GCS_ADVERT)
            except OSError:
                pass
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
                if self.gcs_master is not None:        # queue raw bytes → GCS
                    with self._gcs_out_lock:
                        self._gcs_out += data
                        # Cap if the GCS stalls/isn't attached: drop oldest so we
                        # bound memory (costs one resync on the GCS, not silence).
                        if len(self._gcs_out) > (1 << 18):
                            del self._gcs_out[:len(self._gcs_out) - (1 << 17)]
            else:
                time.sleep(0.002)

    def _gcs_tx_thread(self):
        """Drain the FC→GCS buffer, honouring partial non-blocking writes so a
        frame is never truncated mid-stream (preserve the remainder, retry)."""
        while not self._stop.is_set():
            with self._gcs_out_lock:
                chunk = bytes(self._gcs_out)
            if not chunk:
                time.sleep(0.003)
                continue
            try:
                n = os.write(self.gcs_master, chunk)
            except (BlockingIOError, OSError):
                n = 0                                   # buffer full / GCS absent
            if n:
                with self._gcs_out_lock:
                    del self._gcs_out[:n]
            if n < len(chunk):
                time.sleep(0.003)                       # let the GCS drain

    def _gcs_rx_thread(self):
        """Forward GCS→FC bytes (heartbeats, time-sync, commands) to the FC pty."""
        while not self._stop.is_set():
            try:
                data = os.read(self.gcs_master, 65536)
            except (BlockingIOError, OSError):
                data = b""
            if data:
                try:
                    os.write(self.uart_fd, data)
                except OSError:
                    pass
            else:
                time.sleep(0.004)

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

    def fly_course(self, waypoints, alt=-3.0, secs=60.0, csv=None, reach=2.0,
                   gains=None):
        """Outer guidance: hold altitude + chase waypoints, feeding the FC's
        RC attitude setpoints from GROUND-TRUTH pose (the FC has no position
        loop in sim). waypoints = [(N,E), ...] in metres; alt is NED z (<0=up).
        Returns the recorded trajectory rows."""
        gp = dict(kp_z=0.05, ki_z=0.02, kd_z=0.05, hover=0.36,
                  kp_h=0.10, kd_h=0.30, tilt=0.45)
        if gains:
            gp.update(gains)
        self.arm(settle=1.5)
        rows, wp = [], 0
        iz = 0.0
        csvf = open(csv, "w") if csv else None
        if csvf:
            csvf.write("t,x,y,z,vN,vE,vD,roll,pitch,yaw,wp,thr\n")
        t0 = time.time()
        next_t = t0
        dt = 0.02
        while time.time() - t0 < secs:
            tr = self.truth()
            if tr:
                x, y, z = tr["pos"]
                vN, vE, vD = tr["vel"]
                roll, pitch, yaw = quat_to_euler(*tr["quat"])
                # Altitude: NED z is down-positive; target alt<0 (up). ez>0 ⇒
                # below target ⇒ climb ⇒ more throttle. -vD is climb rate.
                ez = z - alt
                iz = max(-0.3, min(0.3, iz + ez * dt))
                thr = gp["hover"] + gp["kp_z"] * ez + gp["ki_z"] * iz \
                    - gp["kd_z"] * (-vD)
                thr = max(0.0, min(1.0, thr))
                tx, ty = waypoints[wp]
                eN, eE = tx - x, ty - y
                if (eN * eN + eE * eE) < reach * reach and wp < len(waypoints) - 1:
                    wp += 1
                # yaw≈0 ⇒ +N via pitch stick, +E via roll stick (signs verified
                # headless against the FC's stick→angle→motion convention).
                des_pitch = max(-gp["tilt"], min(gp["tilt"],
                                gp["kp_h"] * eN - gp["kd_h"] * vN))
                des_roll = max(-gp["tilt"], min(gp["tilt"],
                               gp["kp_h"] * eE - gp["kd_h"] * vE))
                self.stick(roll=des_roll, pitch=des_pitch, thr=thr, yaw=0.0)
                t = time.time() - t0
                rows.append((t, x, y, z, vN, vE, vD, roll, pitch, yaw, wp, thr))
                if csvf:
                    csvf.write("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                               "%.2f,%.2f,%.2f,%d,%.3f\n"
                               % (t, x, y, z, vN, vE, vD, roll, pitch, yaw, wp, thr))
            next_t += dt
            s = next_t - time.time()
            if s > 0:
                time.sleep(s)
        if csvf:
            csvf.close()
        return rows


def demo(args):
    """Arm the real FC on a rig, command a roll-angle step, record the actual
    controller's tracking response (ground truth) + FC telemetry to CSV."""
    with SitlLab(rig=True, wind=tuple(args.wind), turb=args.turb,
                 gcs=args.gcs) as lab:
        print(f"suffix={lab.suffix}  rc={lab.env['VAYU_UART_RC_PATH']}  "
              f"uart2={lab.uart_path}")
        if args.gcs:
            print(f"\n  ▶ MONITOR IN GCS: in Navigator, Connect → port "
                  f"'{lab.gcs_path}' (auto-listed as 'SITL UART2') @ 115200.")
            print(f"    Live telemetry is bridged there while this run drives + "
                  f"records.\n")
            if args.gcs_wait > 0:
                print(f"  waiting {args.gcs_wait:.0f}s for you to connect the GCS…")
                time.sleep(args.gcs_wait)
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


def flight(args):
    """Attach-mode flight: run on default FIFOs so the GCS 'Attach Ext' renders
    it; fly a waypoint course under harness guidance for `secs`."""
    conf = args.conf or os.path.expanduser("~/.config/Vayu/Vayu GCS.conf")
    # Course: list of (N,E) waypoints. Default = a box at the target altitude.
    if args.course:
        wps = [tuple(float(v) for v in p.split(",")) for p in args.course.split(";")]
    else:
        d = args.box
        wps = [(d, 0), (d, d), (0, d), (0, 0), (d, 0)]
    with SitlLab(attach=True, gcs=args.gcs, conf=conf,
                 wind=tuple(args.wind), turb=args.turb) as lab:
        print(f"vsim_d on DEFAULT /tmp/vsim_* (pose=/tmp/vsim_pose)")
        print(f"  ▶ In Navigator: click 'Attach Ext' to render this flight, and "
              f"Connect → 'SITL UART2' for telemetry.")
        if args.gcs:
            print(f"    (telemetry bridge pty: {lab.gcs_path})")
        if args.gcs_wait > 0:
            print(f"  waiting {args.gcs_wait:.0f}s for you to attach + connect…")
            time.sleep(args.gcs_wait)
        print(f"  flying {len(wps)} waypoints @ alt {args.alt} m for {args.secs:.0f}s…")
        rows = lab.fly_course(wps, alt=args.alt, secs=args.secs, csv=args.csv)
        if rows:
            xs = [r[1] for r in rows]; ys = [r[2] for r in rows]; zs = [r[3] for r in rows]
            print(f"  flew {len(rows)} steps; "
                  f"N∈[{min(xs):+.1f},{max(xs):+.1f}] "
                  f"E∈[{min(ys):+.1f},{max(ys):+.1f}] "
                  f"alt∈[{-max(zs):+.1f},{-min(zs):+.1f}] m; "
                  f"reached wp {rows[-1][10]}/{len(wps)-1}")
        print("FC telemetry:",
              ", ".join(f"{k}×{v}" for k, v in sorted(lab.telem_counts.items())) or "(none)")
        if args.csv:
            print(f"  CSV → {args.csv}")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--attach", action="store_true",
                    help="run on default FIFOs + fly a course (GCS 'Attach Ext' renders it)")
    ap.add_argument("--alt", type=float, default=-3.0, help="hold altitude, NED z (<0=up)")
    ap.add_argument("--box", type=float, default=6.0, help="default-course box side [m]")
    ap.add_argument("--course", type=str, default="",
                    help='waypoints "N,E;N,E;..." (overrides --box)')
    ap.add_argument("--conf", type=str, default="",
                    help="GCS .conf to match vveh/vworld (default: ~/.config/Vayu/...)")
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--wind", type=float, nargs=3, default=[0, 0, 0],
                    metavar=("N", "E", "D"))
    ap.add_argument("--turb", type=float, default=0.0)
    ap.add_argument("--csv", type=str, default="")
    ap.add_argument("--gcs", action="store_true",
                    help="bridge FC telemetry to a pty the Navigator GCS can "
                         "monitor/record live (auto-advertised)")
    ap.add_argument("--gcs-wait", type=float, default=0.0,
                    help="seconds to pause after setup so you can connect the GCS")
    args = ap.parse_args()
    sys.exit(flight(args) if args.attach else demo(args))

