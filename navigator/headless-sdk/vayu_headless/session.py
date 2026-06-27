"""SitlSession — process lifecycle + I/O for headless SITL.

Spawns vsim_d + the real firmware host, streams RC, decodes FC telemetry, reads
ground truth, and (optionally) bridges pose + telemetry to the GCS. Carved
verbatim from sim/host/sitl_lab.py; the pure helpers it uses live in the
sibling modules.
"""
import os
import struct
import subprocess
import sys
import threading
import time
import tty

from . import paths as _paths
from .transport import vsim as _vsim
from .transport import navlink as _navlink
from .transport import rc as _rc
from . import config as _config
from . import world as _world_mod

# Aliases so the carved class body reads exactly as before.
nlframe = _navlink.nlframe
MSG_BY_ID = _navlink.MSG_BY_ID
MAGIC, VERSION, FRAME_POSE = _vsim.MAGIC, _vsim.VERSION, _vsim.FRAME_POSE
HDR, POSE = _vsim.HDR, _vsim.POSE
quat_to_euler = _vsim.quat_to_euler
_vhdr, _ctl = _vsim.vhdr, _vsim.ctl
_reset, _world = _vsim.reset, _vsim.world
_testrig, _wind, _world_mesh = _vsim.testrig, _vsim.wind, _vsim.world_mesh
_read_gcs_conf, _geometry_frame = _config.read_gcs_conf, _config.geometry_frame
_geometry_from_vveh = _config.geometry_from_vveh
_build_world_mesh = _world_mod.build_world_mesh


class SitlLab:
    """Spawns vsim_d + vayu_sitl, streams RC, reads truth + FC telemetry."""

    # GCS-facing singletons (resolved in paths.py).
    GCS_ADVERT = _paths.GCS_ADVERT      # Navigator auto-offers this as "SITL UART2"
    GCS_POSE = _paths.GCS_POSE          # GCS "Attach Ext" reads this

    def __init__(self, rig=False, wind=None, turb=0.0, rc_hz=50, gcs=False,
                 attach=False, conf=None, vveh=None):
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
        self.paths = _paths.fifo_paths(self.suffix)
        self.uart_advert = _paths.uart_advert(self.suffix)
        self._stop = threading.Event()
        # RC frame: roll,pitch,throttle,yaw,SwA(arm),aux  (us; centres 1500)
        self._rc = [1500, 1500, 1000, 1500, 1000, 1500]
        self._rc_lock = threading.Lock()
        self.rc_hz = rc_hz
        self.telem = {}            # latest decoded msg by class name
        self.telem_counts = {}
        self._truth = None

        vsim_bin = _paths.vsim_bin()
        sitl_bin = _paths.sitl_bin()
        for b in (vsim_bin, sitl_bin):
            if not os.path.exists(b):
                raise FileNotFoundError(f"missing binary: {b}")

        # 1) physics daemon
        _vsim_err = (open("/tmp/vsim_d.err", "w")
                     if os.environ.get("SITL_LAB_DEBUG") else subprocess.DEVNULL)
        self.vsim = subprocess.Popen([vsim_bin], env=self.env, stderr=_vsim_err)
        self._await(lambda: all(os.path.exists(p) for p in self.paths.values()),
                    "vsim_d FIFOs")
        self.ctl_fd = os.open(self.paths["ctl"], os.O_RDWR | os.O_NONBLOCK)
        self.pose_fd = os.open(self.paths["pose"], os.O_RDWR | os.O_NONBLOCK)

        # configure plant before the FC boots (spawn just above ground; the
        # ground-clamp fix in physics_core holds the airframe level while
        # resting so it now takes off upright instead of tumbling).
        os.write(self.ctl_fd, _reset(pos=(0, 0, -0.05)))
        # Match the GCS's selected vehicle + world (vveh/vworld) if given.
        # An explicit vveh= overrides the conf as the geometry source (the .vveh
        # is the authoritative frame definition; the conf can drift from it).
        g, w = _read_gcs_conf(conf) if conf else ({}, {})
        if vveh:
            g = _geometry_from_vveh(vveh)
        self.geometry = g          # expose the pushed geometry for verification
        if g:
            os.write(self.ctl_fd, _geometry_frame(g))
        if w:
            os.write(self.ctl_fd, _world(
                gravity=float(w.get("gravity", 9.81)),
                ground_z=float(w.get("ground_z", 0.0)),
                lin_drag=float(w.get("linear_drag", 0.10))))
        else:
            os.write(self.ctl_fd, _world())
        # Push the world COLLISION mesh (rings/obstacles) the same way the GCS
        # does, so the headless craft physically collides with it instead of
        # flying through. Built from the GCS's selected world mesh via the
        # GCS's own loader/BVH builder (render + collision stay in lockstep).
        self.world_mesh_bin = _paths.world_mesh_bin(self.suffix)
        wm = _build_world_mesh(w, self.world_mesh_bin) if w else None
        if wm:
            os.write(self.ctl_fd, _world_mesh(*wm))
            print(f"  [world-mesh] {wm[2]} tris, {wm[3]} nodes → vsim_d "
                  f"(rest={wm[4]}, 2-sided={wm[5]})")
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
                self._pose_out = bytearray()
                self._pose_out_lock = threading.Lock()
                self._pose_advertised = True
                threading.Thread(target=self._pose_tx_thread, daemon=True).start()
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
        wmb = getattr(self, "world_mesh_bin", None)   # remove the BVH blob (vsim_d gone)
        if wmb:
            try:
                os.unlink(wmb)
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
        _navlink.parse_frames(buf, self.telem, self.telem_counts)

    # -- read physics ground truth (vsim_d pose) + fan out to the GCS --------
    def _pose_thread(self):
        """CRITICAL PATH: the SOLE reader of vsim_d's (private) pose FIFO.
        Parses the latest frame into the cached ground truth and QUEUES raw
        bytes for the GCS relay — it NEVER writes to the GCS FIFO itself. The
        relay is a separate thread (_pose_tx_thread) precisely so a slow,
        detaching or reconnecting GCS reader can't stall this drain: if it did,
        vsim_d's blocking pose write would back up and FREEZE the physics step
        (observed: attaching then detaching 'Attach Ext' wedged the whole sim)."""
        pbuf = bytearray()
        while not self._stop.is_set():
            try:
                data = os.read(self.pose_fd, 65536)
            except (BlockingIOError, OSError):
                data = b""
            if data:
                pbuf += data
                self._parse_pose(pbuf)
                if self.gcs_pose_fd is not None:
                    with self._pose_out_lock:
                        self._pose_out += data
                        if len(self._pose_out) > (1 << 18):   # bound if GCS gone
                            del self._pose_out[:len(self._pose_out) - (1 << 17)]
            else:
                time.sleep(0.002)

    def _pose_tx_thread(self):
        """Best-effort relay of queued pose frames to the GCS fan-out FIFO.
        Partial-write-safe; backs off (never spins) when the GCS is detached so
        a full FIFO can't peg a core — and crucially never touches _pose_thread."""
        while not self._stop.is_set():
            with self._pose_out_lock:
                chunk = bytes(self._pose_out)
            if not chunk:
                time.sleep(0.005)
                continue
            try:
                n = os.write(self.gcs_pose_fd, chunk)
            except (BlockingIOError, OSError):
                n = 0
            if n:
                with self._pose_out_lock:
                    del self._pose_out[:n]
            if n < len(chunk):
                time.sleep(0.01)               # GCS slow/detached — back off

    def _parse_pose(self, buf):
        t = _vsim.parse_pose(buf)
        if t is not None:
            self._truth = t

    def truth(self):
        """Latest ground-truth pose (updated by _pose_thread)."""
        return self._truth

    def reset_pose(self, pos):
        """Re-spawn the airframe at pos (NED) with zero velocity."""
        os.write(self.ctl_fd, _reset(pos=tuple(pos)))

    # Continuous outer-loop guidance (takeoff/goto/land) lives in
    # vayu_headless.autopilot.Pilot, which all callers use.

# Canonical name; SitlLab kept as a back-compat alias.
SitlSession = SitlLab
