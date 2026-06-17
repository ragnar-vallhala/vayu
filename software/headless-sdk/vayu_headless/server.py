"""Persistent-session control server + command protocol.

Boots ONCE (SitlSession + Pilot), then accepts flight commands over a Unix
socket so the GCS attaches a single time. Two wire encodings share one command
model (Phase 3):
  - text  : "fly 8,0;8,8 -5 40"  ->  "ok reached in 23.1s; nav=4 ..."  (back-compat)
  - JSON  : {"v":1,"cmd":"fly","course":[[8,0],[8,8]],"alt":-5,"timeout":40}
            ->  {"ok":true,"status":{...}}

`parse_command` and `course_from` are pure (unit-tested without a daemon); the
SessionServer binds them to a live SitlSession/Pilot.
"""
import json
import socket
import threading
import time

PROTOCOL_VERSION = 1


def course_from(val):
    """Accept a course as "N,E;N,E;..." (text) or [[N,E],...] (JSON) → [(N,E)]."""
    if isinstance(val, str):
        return [tuple(float(v) for v in p.split(",")) for p in val.split(";") if p]
    return [tuple(float(v) for v in pt) for pt in val]


def parse_command(raw):
    """Parse a request line into a normalised command dict {"cmd": ..., ...}.
    Pure: no I/O, no session. Raises ValueError on malformed input."""
    raw = raw.strip()
    if not raw:
        return {"cmd": "status"}
    if raw.startswith("{"):
        obj = json.loads(raw)
        if "cmd" not in obj:
            raise ValueError("missing 'cmd'")
        return obj
    parts = raw.split()
    cmd = parts[0]
    rest = parts[1:]
    out = {"cmd": "status" if cmd == "st" else cmd}
    if cmd in ("goto", "fly"):
        if rest:
            out["course"] = rest[0]
        if len(rest) > 1:
            out["alt"] = float(rest[1])
        if len(rest) > 2:
            out["timeout"] = float(rest[2])
    elif cmd == "takeoff":
        if rest:
            out["alt"] = float(rest[0])
    elif cmd == "alt":
        out["alt"] = float(rest[0])
    elif cmd == "wait":
        out["secs"] = float(rest[0]) if rest else 1.0
    elif cmd == "rc":
        out["rc"] = [float(rest[i]) if i < len(rest) else 0.0 for i in range(4)]
    return out


def status_dict(lab, pilot):
    hb = lab.telem.get("Heartbeat")
    nav = getattr(hb, "nav_state", -1) if hb else -1
    last = pilot.last or {}
    return {
        "nav": nav, "armed": pilot.armed, "active": pilot.active,
        "pos": [last.get("x", 0.0), last.get("y", 0.0)],
        "alt": -last.get("z", 0.0), "vD": last.get("vD", 0.0),
        "att": [last.get("roll", 0.0), last.get("pitch", 0.0), last.get("yaw", 0.0)],
        "wp": [last.get("wp", 0), len(pilot.wps) - 1], "thr": last.get("thr", 0.0),
        "telem": dict(lab.telem_counts),
    }


def status_text(s):
    tc = ",".join(f"{k}×{v}" for k, v in sorted(s["telem"].items()))
    return (f"nav={s['nav']} armed={s['armed']} active={s['active']} "
            f"pos=({s['pos'][0]:+.2f},{s['pos'][1]:+.2f}) alt={s['alt']:+.2f}m "
            f"vD={s['vD']:+.2f} att=(r{s['att'][0]:+.0f},p{s['att'][1]:+.0f},"
            f"y{s['att'][2]:+.0f}) wp={s['wp'][0]}/{s['wp'][1]} "
            f"thr={s['thr']:.2f} | telem[{tc}]")


class SessionServer:
    """Binds the pure command model to a live SitlSession + Pilot."""

    def __init__(self, lab, pilot):
        self.lab = lab
        self.pilot = pilot
        self._cmd_lock = threading.Lock()

    def dispatch(self, c):
        """Execute one normalised command dict; return (resp_dict, is_bye)."""
        cmd = c.get("cmd", "status")
        lab, pilot = self.lab, self.pilot
        if cmd in ("status", "st"):
            return {"ok": True, "status": status_dict(lab, pilot)}, False
        if cmd in ("quit", "stop", "shutdown"):
            return {"ok": True, "bye": True}, True
        if cmd == "takeoff":
            with self._cmd_lock:
                pilot.arm_takeoff(alt=c.get("alt"))
            return {"ok": True, "status": status_dict(lab, pilot)}, False
        if cmd in ("goto", "fly"):
            course = course_from(c.get("course", [(0.0, 0.0)]))
            alt = c.get("alt")
            with self._cmd_lock:
                if not pilot.armed:
                    pilot.arm_takeoff(alt=alt)
                pilot.goto(course, alt=alt)
            if cmd == "goto":
                return {"ok": True, "status": status_dict(lab, pilot)}, False
            timeout = float(c.get("timeout", 30.0))
            t0 = time.time()
            while time.time() - t0 < timeout:
                if pilot.reached_last():
                    return {"ok": True, "reached": True,
                            "secs": round(time.time() - t0, 1),
                            "status": status_dict(lab, pilot)}, False
                time.sleep(0.1)
            return {"ok": True, "reached": False, "timeout": timeout,
                    "status": status_dict(lab, pilot)}, False
        if cmd == "wait":
            time.sleep(float(c.get("secs", 1.0)))
            return {"ok": True, "status": status_dict(lab, pilot)}, False
        if cmd == "alt":
            with pilot._lock:
                pilot.alt = float(c["alt"])
            return {"ok": True, "status": status_dict(lab, pilot)}, False
        if cmd == "land":
            with self._cmd_lock:
                pilot.land()
            return {"ok": True, "status": status_dict(lab, pilot)}, False
        if cmd == "rc":
            r = c.get("rc", [0, 0, 0, 0])
            with pilot._lock:
                pilot.active = False
            lab.stick(roll=r[0], pitch=r[1], thr=r[2], yaw=r[3])
            lab.set_rc(swa=2000)
            return {"ok": True, "status": status_dict(lab, pilot)}, False
        return {"ok": False, "error": f"unknown command: {cmd}"}, False

    def _handle(self, conn):
        try:
            data = b""
            while b"\n" not in data:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                data += chunk
            raw = data.decode(errors="replace").strip()
            as_json = raw.startswith("{")
            try:
                cmd = parse_command(raw)
                resp, is_bye = self.dispatch(cmd)
            except Exception as e:                       # noqa: BLE001
                resp, is_bye = {"ok": False, "error": str(e)}, False
            if as_json:
                out = json.dumps(resp)
            elif resp.get("bye"):
                out = "BYE"
            elif not resp.get("ok"):
                out = "err " + resp.get("error", "?")
            else:
                pre = ""
                if "reached" in resp:
                    pre = (f"reached in {resp['secs']}s; " if resp["reached"]
                           else f"timeout {resp['timeout']:.0f}s; ")
                out = "ok " + pre + status_text(resp["status"])
            conn.sendall((out + "\n").encode())
            if is_bye:
                self.lab._stop.set()
        except Exception:                                # noqa: BLE001
            pass
        finally:
            conn.close()

    def serve_forever(self, sock_path):
        import os
        if os.path.exists(sock_path):
            os.unlink(sock_path)
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(sock_path)
        srv.listen(8)
        try:
            while not self.lab._stop.is_set():
                srv.settimeout(0.5)
                try:
                    conn, _ = srv.accept()
                except socket.timeout:
                    continue
                threading.Thread(target=self._handle, args=(conn,),
                                 daemon=True).start()
        except KeyboardInterrupt:
            pass
        finally:
            srv.close()
            try:
                os.unlink(sock_path)
            except OSError:
                pass
            self.lab.close()
