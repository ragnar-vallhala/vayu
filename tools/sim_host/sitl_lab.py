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
import socket
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
        # Thrust axis: vsim lift is body -Z (F = axis*thrust, motor_model.cpp).
        # The GCS conf stores az in a frame where +1 is "up", which is -Z in
        # vsim's NED — pushing it verbatim thrusts DOWNWARD and jams the craft
        # into the ground. A standard quad's rotors all lift up, so force -Z.
        body += struct.pack("<3f", 0.0, 0.0, -1.0)
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

    # Default pose FIFO the GCS "Attach Ext" reads (SimWorker startAttach).
    GCS_POSE = "/tmp/vsim_pose"

    def __init__(self, rig=False, wind=None, turb=0.0, rc_hz=50, gcs=False,
                 attach=False, conf=None):
        # ALWAYS run vsim_d/vayu_sitl on PRIVATE (suffixed) FIFOs+advert: the
        # harness must be the sole reader of the pose FIFO and the FC pty. When
        # gcs=True it re-broadcasts pose to the DEFAULT /tmp/vsim_pose (a fan-out
        # FIFO the harness owns) so the GCS "Attach Ext" renders it — the same
        # publish pattern as the UART2 telemetry bridge. (A FIFO is single-reader:
        # if the GCS opened the real pose FIFO directly it would split the byte
        # stream with the harness and corrupt every frame — hence the re-broadcast.)
        self.suffix = f"_lab{os.getpid()}"
        self.attach = attach
        self.gcs = gcs
        self.gcs_master = None
        self.gcs_path = None
        self.gcs_pose_fd = None
        self._pose_advertised = False
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

        # configure plant before the FC boots (spawn just above ground; the
        # ground-clamp fix in physics_core holds the airframe level while
        # resting so it now takes off upright instead of tumbling).
        os.write(self.ctl_fd, _reset(pos=(0, 0, -0.05)))
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

        # 2) RC pty + real firmware host.
        # In attach mode the FC advert path (where vayu_sitl writes its UART2
        # slave) is the SAME file the GCS bridge later advertises itself at.
        # A stale advert from a previous run points at a now-deleted /dev/pts/N,
        # so delete it FIRST and only accept the path vayu_sitl writes THIS run
        # — otherwise we open a dead pty and never drain the FC's UART2 (the FC's
        # blocking pty write then backs up and all telemetry stalls).
        try:
            os.unlink(self.uart_advert)
        except OSError:
            pass
        self.rc_master, rc_slave = os.openpty()
        self.env["VAYU_UART_RC_PATH"] = os.ttyname(rc_slave)
        _errto = (open("/tmp/sitl.err", "w") if os.environ.get("SITL_LAB_DEBUG")
                  else subprocess.DEVNULL)
        self.sitl = subprocess.Popen([sitl_bin], env=self.env, stderr=_errto)
        threading.Thread(target=self._rc_thread, daemon=True).start()

        # 3) FC telemetry: wait for the (fresh) advertised UART2 pty slave path,
        # and require the path it names to actually exist before opening it.
        def _fc_pty_ready():
            try:
                with open(self.uart_advert) as f:
                    p = f.read().strip()
            except OSError:
                return False
            if p and os.path.exists(p):
                self.uart_path = p
                return True
            return False
        self._await(_fc_pty_ready, "UART2 advert")
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

            # GCS-facing pose fan-out FIFO. The harness owns it (creates it and
            # holds it O_RDWR so writes never SIGPIPE while the GCS is detached);
            # the GCS "Attach Ext" opens it O_RDONLY. _pose_thread relays every
            # frame here verbatim, so the GCS renders the SAME pose the harness
            # guides on. Replace any stale node (e.g. a real-FIFO from a prior run).
            try:
                if os.path.exists(self.GCS_POSE):
                    os.unlink(self.GCS_POSE)
                os.mkfifo(self.GCS_POSE)
                self.gcs_pose_fd = os.open(self.GCS_POSE, os.O_RDWR | os.O_NONBLOCK)
                self._pose_advertised = True
            except OSError:
                self.gcs_pose_fd = None

        # Pose relay: the SOLE reader of vsim_d's (private) pose FIFO. Updates
        # the cached ground truth for guidance AND fans frames out to the GCS.
        threading.Thread(target=self._pose_thread, daemon=True).start()

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
        if self._pose_advertised:                # remove our GCS pose fan-out FIFO
            try:
                os.unlink(self.GCS_POSE)
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
        self._telem_bytes = 0
        while not self._stop.is_set():
            try:
                data = os.read(self.uart_fd, 65536)
            except (BlockingIOError, OSError):
                data = b""
            if data:
                self._telem_bytes += len(data)
                buf += data
                # A malformed frame must never kill this thread: if it dies, the
                # FC's blocking UART2 write backs up and ALL telemetry stalls.
                try:
                    self._parse_frames(buf)
                except Exception:                 # noqa: BLE001
                    del buf[:]
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

    # -- read physics ground truth (vsim_d pose) + fan out to the GCS --------
    def _pose_thread(self):
        """Sole reader of vsim_d's (private) pose FIFO: parse the latest frame
        into the cached ground truth AND relay every byte verbatim to the
        GCS-facing fan-out FIFO so 'Attach Ext' renders the same pose."""
        pbuf = bytearray()        # frame-parse buffer (ground truth)
        obuf = bytearray()        # outbound relay buffer (to the GCS)
        while not self._stop.is_set():
            try:
                data = os.read(self.pose_fd, 65536)
            except (BlockingIOError, OSError):
                data = b""
            if data:
                pbuf += data
                self._parse_pose(pbuf)
                if self.gcs_pose_fd is not None:
                    obuf += data
                    if len(obuf) > (1 << 18):        # bound if GCS detached
                        del obuf[:len(obuf) - (1 << 17)]
            if self.gcs_pose_fd is not None and obuf:
                try:
                    n = os.write(self.gcs_pose_fd, bytes(obuf))
                except (BlockingIOError, OSError):
                    n = 0                            # FIFO full / GCS not reading
                if n:
                    del obuf[:n]
            if not data:
                time.sleep(0.002)

    def _parse_pose(self, buf):
        pos = 0
        while pos + HDR.size <= len(buf):
            magic, ver, t, plen, seq = HDR.unpack_from(buf, pos)
            if magic != MAGIC:
                pos += 1
                continue
            if pos + HDR.size + plen > len(buf):
                break
            if t == FRAME_POSE and plen >= POSE.size:
                p = POSE.unpack_from(buf, pos + HDR.size)
                self._truth = {
                    "pos": p[2:5], "quat": p[5:9], "vel": p[9:12],
                    "omega": p[12:15], "motor_omega": p[15:19],
                    "wind": p[23:26],
                }
            pos += HDR.size + plen
        del buf[:pos]

    def truth(self):
        """Latest ground-truth pose (updated by _pose_thread)."""
        return self._truth

    def reset_pose(self, pos):
        """Re-spawn the airframe at pos (NED) with zero velocity."""
        os.write(self.ctl_fd, _reset(pos=tuple(pos)))

    def takeoff(self, hover=0.5, climb=0.7):
        """Clean ground takeoff (the physics ground-clamp now holds the airframe
        level while resting, so it lifts off upright instead of tumbling):
        spawn on the ground, arm at low throttle, then ramp to a climb and
        settle at hover."""
        self.reset_pose((0, 0, -0.05))
        time.sleep(0.3)
        self.set_rc(swa=1000, thr=1000)            # ensure disarmed → STANDBY
        time.sleep(0.3)
        self.set_rc(swa=2000, thr=1000)            # arm gesture (low throttle)
        time.sleep(0.4)
        self.stick(thr=climb); self.set_rc(swa=2000)  # break ground
        time.sleep(0.6)
        self.stick(thr=hover); self.set_rc(swa=2000)  # settle to ~hover

    def fly_course(self, waypoints, alt=-3.0, secs=60.0, csv=None, reach=2.0,
                   gains=None):
        """Outer guidance: hold altitude + chase waypoints, feeding the FC's
        RC attitude setpoints from GROUND-TRUTH pose (the FC has no position
        loop in sim). waypoints = [(N,E), ...] in metres; alt is NED z (<0=up).
        Returns the recorded trajectory rows."""
        gp = dict(kp_z=0.05, ki_z=0.02, kd_z=0.05, hover=0.36,
                  kp_h=0.06, kd_h=0.55, tilt=0.30, vmax=3.0)
        if gains:
            gp.update(gains)
        self.takeoff(hover=gp["hover"])
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
                # Cascade: position error → speed-capped desired velocity →
                # damped tilt. The velocity cap + damping stop the orbiting that
                # a raw position-P term produced. yaw≈0 ⇒ +N via pitch, +E via
                # roll (signs verified headless vs the FC's stick→motion map).
                vdes_n = max(-gp["vmax"], min(gp["vmax"], 0.6 * eN))
                vdes_e = max(-gp["vmax"], min(gp["vmax"], 0.6 * eE))
                des_pitch = max(-gp["tilt"], min(gp["tilt"],
                                gp["kd_h"] * (vdes_n - vN)))
                des_roll = max(-gp["tilt"], min(gp["tilt"],
                               gp["kd_h"] * (vdes_e - vE)))
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
        self.lab.reset_pose((0, 0, -0.05))
        time.sleep(0.3)
        self.lab.set_rc(swa=1000, thr=1000)          # ensure disarmed → STANDBY
        time.sleep(0.3)
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


SOCK_PATH = "/tmp/sitl_lab.sock"


def _parse_course(s):
    return [tuple(float(v) for v in p.split(",")) for p in s.split(";") if p]


def serve(args):
    """Persistent session: boot physics + the real FC + the GCS bridge ONCE,
    keep the craft under continuous guidance, and accept flight commands over a
    Unix socket. Attach the GCS a single time (pose 'Attach Ext' + 'SITL UART2');
    every `do` command then flies against that same live session."""
    conf = args.conf or os.path.expanduser("~/.config/Vayu/Vayu GCS.conf")
    lab = SitlLab(attach=True, gcs=True, conf=conf,
                  wind=tuple(args.wind), turb=args.turb)
    pilot = Pilot(lab, alt=args.alt)
    print(f"vsim_d on DEFAULT /tmp/vsim_* (pose=/tmp/vsim_pose)")
    print(f"  ▶ In Navigator (ONCE): click 'Attach Ext' to render, and "
          f"Connect → 'SITL UART2' for telemetry.")
    print(f"    (telemetry bridge pty: {lab.gcs_path})")
    print(f"  control socket: {SOCK_PATH}")
    if args.gcs_wait > 0:
        print(f"  waiting {args.gcs_wait:.0f}s for you to attach + connect…")
        time.sleep(args.gcs_wait)

    if os.path.exists(SOCK_PATH):
        os.unlink(SOCK_PATH)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCK_PATH)
    srv.listen(8)
    cmd_lock = threading.Lock()
    print("  session ready — send commands with: sitl_lab.py do '<cmd>'")

    def status():
        hb = lab.telem.get("Heartbeat")
        nav = getattr(hb, "nav_state", -1) if hb else -1
        L = pilot.last
        tc = ",".join(f"{k}×{v}" for k, v in sorted(lab.telem_counts.items()))
        return (f"nav={nav} armed={pilot.armed} active={pilot.active} "
                f"pos=({L.get('x',0):+.2f},{L.get('y',0):+.2f}) "
                f"alt={-L.get('z',0):+.2f}m vD={L.get('vD',0):+.2f} "
                f"att=(r{L.get('roll',0):+.0f},p{L.get('pitch',0):+.0f},"
                f"y{L.get('yaw',0):+.0f}) wp={L.get('wp',0)}/{len(pilot.wps)-1} "
                f"thr={L.get('thr',0):.2f} | telem[{tc}]")

    def dispatch(line):
        parts = line.split()
        if not parts:
            return "ok " + status()
        cmd, rest = parts[0], parts[1:]
        if cmd in ("status", "st"):
            return "ok " + status()
        if cmd == "takeoff":
            alt = float(rest[0]) if rest else None
            with cmd_lock:
                pilot.arm_takeoff(alt=alt)
            return "ok takeoff; " + status()
        if cmd in ("goto", "fly"):
            # fly <course> [alt] [timeout]; goto = same but non-blocking
            course = _parse_course(rest[0]) if rest else [(0.0, 0.0)]
            alt = float(rest[1]) if len(rest) > 1 else None
            timeout = float(rest[2]) if len(rest) > 2 else 30.0
            with cmd_lock:
                if not pilot.armed:
                    pilot.arm_takeoff(alt=alt)
                pilot.goto(course, alt=alt)
            if cmd == "goto":
                return "ok goto set; " + status()
            t0 = time.time()                          # fly = block until reached
            while time.time() - t0 < timeout:
                if pilot.reached_last():
                    return f"ok reached in {time.time()-t0:.1f}s; " + status()
                time.sleep(0.1)
            return f"ok timeout {timeout:.0f}s; " + status()
        if cmd == "wait":
            time.sleep(float(rest[0]) if rest else 1.0)
            return "ok " + status()
        if cmd == "alt":
            with pilot._lock:
                pilot.alt = float(rest[0])
            return "ok " + status()
        if cmd == "land":
            with cmd_lock:
                pilot.land()
            return "ok landed; " + status()
        if cmd == "rc":                               # raw stick: r p t y (disables guidance)
            with pilot._lock:
                pilot.active = False
            r, p, t, yv = (float(rest[i]) if i < len(rest) else 0.0 for i in range(4))
            lab.stick(roll=r, pitch=p, thr=t, yaw=yv)
            lab.set_rc(swa=2000)
            return "ok rc; " + status()
        if cmd in ("quit", "stop", "shutdown"):
            return "BYE"
        return f"err unknown command: {cmd}"

    def handle(conn):
        try:
            data = b""
            while b"\n" not in data:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                data += chunk
            line = data.decode(errors="replace").strip()
            resp = dispatch(line)
            conn.sendall((resp + "\n").encode())
            if resp == "BYE":
                lab._stop.set()
        except Exception as e:                        # noqa: BLE001
            try:
                conn.sendall(f"err {e}\n".encode())
            except OSError:
                pass
        finally:
            conn.close()

    try:
        while not lab._stop.is_set():
            srv.settimeout(0.5)
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            threading.Thread(target=handle, args=(conn,), daemon=True).start()
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()
        try:
            os.unlink(SOCK_PATH)
        except OSError:
            pass
        lab.close()
        print("session closed")
    return 0


def client(args):
    """Send one command to a running `serve` session and print the reply."""
    cmd = " ".join(args.do)
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        s.connect(SOCK_PATH)
    except (FileNotFoundError, ConnectionRefusedError):
        print(f"err: no session at {SOCK_PATH} — start one with: "
              f"sitl_lab.py --serve", file=sys.stderr)
        return 1
    s.sendall((cmd + "\n").encode())
    buf = b""
    while b"\n" not in buf:
        chunk = s.recv(4096)
        if not chunk:
            break
        buf += chunk
    s.close()
    print(buf.decode(errors="replace").strip())
    return 0


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
    ap.add_argument("--serve", action="store_true",
                    help="persistent session: boot physics+FC+GCS-bridge ONCE, "
                         "keep the craft under continuous guidance, accept "
                         "flight commands over a socket (attach the GCS just once)")
    ap.add_argument("--do", nargs=argparse.REMAINDER,
                    help="send one command to a running --serve session, e.g. "
                         "--do fly 8,0\\;8,8\\;0,8\\;0,0 -5 40 (see commands below)")
    args = ap.parse_args()
    if args.do is not None:
        sys.exit(client(args))
    if args.serve:
        sys.exit(serve(args))
    sys.exit(flight(args) if args.attach else demo(args))

