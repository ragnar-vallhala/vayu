#!/usr/bin/env python3
"""sitl_lab.py — thin CLI shim over the vayu_headless SDK (Phase 1).

The library (SitlSession/Pilot + wire layers) now lives in the vayu_headless
package (software/headless-sdk). This file keeps the serve/do/attach CLI until
Phase 2 moves it into vayu_headless.cli. See software/headless-sdk/PLAN.md.
"""
import argparse
import os
import socket
import sys
import threading
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_ROOT, "software", "headless-sdk"))
from vayu_headless.session import SitlLab, SitlSession       # noqa: E402,F401
from vayu_headless.autopilot import Pilot, DEFAULT_GAINS     # noqa: E402,F401
from vayu_headless.transport.vsim import quat_to_euler       # noqa: E402


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

