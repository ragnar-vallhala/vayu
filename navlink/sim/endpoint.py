#!/usr/bin/env python3
"""One simulator endpoint: role=fc emits telemetry and answers commands; role=gcs
receives, collects link statistics, and probes the link with PINGs and commands.

Run via sim.py, or standalone:
    python3 endpoint.py --role fc  --listen 51001 --peer 51002 --config '{...}'
    python3 endpoint.py --role gcs --listen 51002 --peer 51001 --config '{...}'
"""
import argparse
import json
import math
import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import load_nl          # noqa: E402
import frame                        # noqa: E402
from link import Link, LinkConfig   # noqa: E402

nl = load_nl()

FC_SYS, GCS_SYS, COMP = 1, 255, 1


def udp_socket(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.settimeout(0.2)
    return s


# ── FC telemetry models (realistic-ish values; rates approximate the firmware) ──
def m_heartbeat(t, rng):
    return nl.Heartbeat(type=2, autopilot=1, base_mode=1, system_status=4,
                        nav_state=int(nl.NavState.IN_AIR), capabilities=0x1F,
                        timestamp=int(t * 1000) & 0xFFFFFFFF)


def m_attitude(t, rng):
    return nl.AttitudeEuler(roll=0.3 * math.sin(t), pitch=0.2 * math.sin(0.7 * t),
                            yaw=math.sin(0.1 * t), rollspeed=0.3 * math.cos(t),
                            pitchspeed=0.14 * math.cos(0.7 * t), yawspeed=0.1 * math.cos(0.1 * t))


def m_imu(t, rng):
    return nl.ImuRaw(acc=[rng.gauss(0, 0.2), rng.gauss(0, 0.2), rng.gauss(-9.81, 0.2)],
                     gyr=[rng.gauss(0, 1.0) for _ in range(3)],
                     mag=[rng.gauss(20, 1), rng.gauss(0, 1), rng.gauss(40, 1)],
                     temp=35.0, sample_time_us=int(t * 1e6) & 0xFFFFFFFF)


def m_imu_c(t, rng):
    return nl.ImuCompressed(ref_seq=int(t) & 0xFF, _pad=0,
                            delta=[rng.randrange(0, 0xFFFF) for _ in range(10)])


def m_rc(t, rng):
    return nl.RcChannels(chan=[1500 + int(400 * math.sin(t + i)) for i in range(18)],
                         rssi=200, count=8)


def m_motor(t, rng):
    base = 0.5 + 0.1 * math.sin(t)
    return nl.MotorTelemetry(cmd=[max(0.0, min(1.0, base + 0.05 * i)) for i in range(8)])


def m_control(t, rng):
    ct = nl.ControlTrace()
    ct.roll_angle_sp = 0.3 * math.sin(t)
    ct.roll_angle_curr = ct.roll_angle_sp + rng.gauss(0, 0.01)
    ct.outer_dt, ct.inner_dt = 0.02, 0.005
    return ct


def m_health(t, rng):
    return nl.SystemHealth(tx_overflow=0, imu_drop=0, log_wrap=0,
                           cpu_load=40 + int(10 * math.sin(t)))


def m_est(t, rng):
    return nl.EstPerf(peak_us=120.0, mean_us=80.0, decimation=1.0, rate_hz=500.0)


def m_flightmode(t, rng):
    return nl.FlightMode(mode=int(nl.FlightModeEnum.ANGLE), source=int(nl.ModeSource.RC))


SCHEDULE = [
    (nl.Heartbeat, 1.0, m_heartbeat),
    (nl.AttitudeEuler, 50.0, m_attitude),
    (nl.ImuRaw, 1.0, m_imu),
    (nl.ImuCompressed, 25.0, m_imu_c),
    (nl.RcChannels, 10.0, m_rc),
    (nl.MotorTelemetry, 20.0, m_motor),
    (nl.ControlTrace, 20.0, m_control),
    (nl.SystemHealth, 1.0, m_health),
    (nl.EstPerf, 2.0, m_est),
    (nl.FlightMode, 1.0, m_flightmode),
]


class FC:
    def __init__(self, sock, link, cfg):
        self.sock, self.link, self.cfg = sock, link, cfg
        import random
        self.rng = random.Random(cfg.get("seed", 0) ^ 0xF00D)
        self.seq = 0
        self.lock = threading.Lock()
        self.tx_by = {}
        self.rx_frames = self.cmds_acked = self.pings_echoed = 0
        self.stop = threading.Event()
        # runtime-mutable telemetry streams (enable/disable + rate, live)
        self.streams = [{"name": cls.__name__, "cls": cls, "build": build,
                         "rate": rate, "enabled": True, "count": 0, "due": 0.0}
                        for (cls, rate, build) in SCHEDULE]

    def _send(self, msgid, payload):
        with self.lock:
            seq = self.seq
            self.seq = (self.seq + 1) & 0xFF
        self.link.send(frame.encode(msgid, payload, seq=seq, sysid=FC_SYS, compid=COMP))

    def _rx_loop(self):
        while not self.stop.is_set():
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            d = frame.decode(data)
            if not d.ok:
                continue
            self.rx_frames += 1
            if d.msgid == nl.Ping.MSGID:                      # echo PING back
                self._send(nl.Ping.MSGID, d.payload)
                self.pings_echoed += 1
            elif 8192 <= d.msgid <= 12319:                    # a command → COMMAND_ACK
                req_seq = d.payload[2] if len(d.payload) > 2 else 0
                ack = nl.CommandAck(command=d.msgid, req_seq=req_seq,
                                    result=int(nl.CommandResult.ACCEPTED),
                                    progress=100, result_param2=0)
                self._send(nl.CommandAck.MSGID, ack.pack())
                self.cmds_acked += 1

    def run(self, duration):
        rx = threading.Thread(target=self._rx_loop, daemon=True)
        rx.start()
        start = time.monotonic()
        for s in self.streams:
            s["due"] = start
        while not self.stop.is_set() and time.monotonic() - start < duration:
            now = time.monotonic()
            for s in self.streams:
                if not s["enabled"] or s["rate"] <= 0:
                    continue
                if now >= s["due"]:
                    self._send(s["cls"].MSGID, s["build"](now - start, self.rng).pack())
                    s["count"] += 1
                    self.tx_by[s["name"]] = self.tx_by.get(s["name"], 0) + 1
                    period = 1.0 / s["rate"]
                    s["due"] += period
                    if s["due"] < now:                        # fell behind; resync
                        s["due"] = now + period
            time.sleep(0.001)
        self.stop.set()
        rx.join(timeout=1.0)
        self.link.drain()
        return self.report()

    def set_stream(self, token, enabled=None, rate=None):
        """Apply enable/disable and/or rate to every stream whose name contains
        `token` (case-insensitive), or all streams if token == 'all'."""
        token = token.upper()
        changed = []
        for s in self.streams:
            if token == "ALL" or token in s["name"].upper():
                if rate is not None:
                    s["rate"] = rate
                    if rate > 0 and enabled is None:
                        s["enabled"] = True
                if enabled is not None:
                    s["enabled"] = enabled
                changed.append(s["name"])
        return changed

    def stream_table(self):
        return [(s["name"], s["rate"], s["enabled"], s["count"]) for s in self.streams]

    def report(self):
        return {
            "role": "fc",
            "tx_frames": sum(self.tx_by.values()),
            "by_msgid": self.tx_by,
            "rx_frames": self.rx_frames,
            "cmds_acked": self.cmds_acked,
            "pings_echoed": self.pings_echoed,
            "egress": self.link.stats(),
        }


class GCS:
    def __init__(self, sock, link, cfg):
        self.sock, self.link, self.cfg = sock, link, cfg
        self.seq = 0
        self.lock = threading.Lock()
        self.stop = threading.Event()
        # rx stats
        self.rx_frames = self.rx_bytes = 0
        self.crc_errors = self.undecodable = self.non_frame = 0
        self.by_msgid = {}
        self.first_rx = self.last_rx = None
        # loss/reorder from FC seq stream (reorder-tolerant extended-seq span)
        self.last_raw = None
        self.ext = self.min_ext = self.max_ext = 0
        self.fc_rx = self.reorders = 0
        # probes
        self.ping_sent = {}
        self.ping_rtts = []
        self.ping_count = 0
        self.cmd_sent = {}
        self.cmd_acks = 0
        self.cmd_lat = []
        self.cmd_count = 0
        self.pseq = 0
        self.rseq = 0

    def _send(self, msgid, payload):
        with self.lock:
            seq = self.seq
            self.seq = (self.seq + 1) & 0xFF
        self.link.send(frame.encode(msgid, payload, seq=seq, sysid=GCS_SYS, compid=COMP))

    def send_ping(self):
        with self.lock:
            self.pseq = (self.pseq + 1) & 0xFFFFFFFF
            pseq = self.pseq
        self.ping_sent[pseq] = time.monotonic()
        self.ping_count += 1
        self._send(nl.Ping.MSGID, nl.Ping(seq=pseq, target=FC_SYS).pack())

    def send_command(self, cls, **fields):
        """Send any command message (CMD_*), tracking its ACK latency."""
        with self.lock:
            self.rseq = (self.rseq + 1) & 0xFF
            rseq = self.rseq
        self.cmd_sent[rseq] = time.monotonic()
        self.cmd_count += 1
        msg = cls(target_sys=FC_SYS, target_comp=COMP, req_seq=rseq, **fields)
        self._send(cls.MSGID, msg.pack())

    def _track_seq(self, seq):
        # Unwrap the 8-bit seq into a monotonic extended counter so reordering
        # nets out instead of being mistaken for loss. Loss is then the gap
        # between the span of seqs seen and the number actually received.
        self.fc_rx += 1
        if self.last_raw is None:
            self.last_raw = seq
            return
        diff = (seq - self.last_raw) & 0xFF
        if diff >= 128:
            diff -= 256                  # signed: negative = out-of-order arrival
        self.ext += diff
        self.last_raw = seq
        self.max_ext = max(self.max_ext, self.ext)
        self.min_ext = min(self.min_ext, self.ext)
        if diff <= 0:
            self.reorders += 1

    def _rx_loop(self):
        while not self.stop.is_set():
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            now = time.monotonic()
            d = frame.decode(data)
            self.rx_bytes += d.nbytes
            if self.first_rx is None:
                self.first_rx = now
            self.last_rx = now
            if not d.ok:
                if d.reason == "crc":
                    self.crc_errors += 1
                elif d.reason == "unknown_msgid":
                    self.undecodable += 1
                else:
                    self.non_frame += 1
                continue
            self.rx_frames += 1
            name = nl.MSGID_TO_CLASS[d.msgid].__name__
            self.by_msgid[name] = self.by_msgid.get(name, 0) + 1
            if d.sysid == FC_SYS:
                self._track_seq(d.seq)
            if d.msgid == nl.Ping.MSGID:                       # echo returned
                p = nl.Ping.unpack(d.payload)
                t0 = self.ping_sent.pop(p.seq, None)
                if t0 is not None:
                    self.ping_rtts.append((now - t0) * 1000.0)
            elif d.msgid == nl.CommandAck.MSGID:
                ack = nl.CommandAck.unpack(d.payload)
                t0 = self.cmd_sent.pop(ack.req_seq, None)
                if t0 is not None:
                    self.cmd_acks += 1
                    self.cmd_lat.append((now - t0) * 1000.0)

    def run(self, duration, grace=1.0):
        rx = threading.Thread(target=self._rx_loop, daemon=True)
        rx.start()
        start = time.monotonic()
        next_ping = start + 1.0
        next_cmd = start + 0.5
        while not self.stop.is_set() and time.monotonic() - start < duration:
            now = time.monotonic()
            if now >= next_ping:
                self.send_ping()
                next_ping += 1.0
            if now >= next_cmd:
                self.send_command(nl.CmdSetPid, controller=0, axis=0,
                                  kp=0.1, ki=0.01, kd=0.001, kff=0.0)
                next_cmd += 0.5
            time.sleep(0.005)
        time.sleep(grace)                                      # drain in-flight
        self.stop.set()
        rx.join(timeout=1.0)
        return self.report()

    def report(self):
        span = (self.last_rx - self.first_rx) if (self.first_rx and self.last_rx) else 0.0
        span = span or 1e-9
        rtt = sorted(self.ping_rtts)

        def stat(xs):
            return ({"count": len(xs), "min_ms": round(min(xs), 2),
                     "avg_ms": round(sum(xs) / len(xs), 2), "max_ms": round(max(xs), 2)}
                    if xs else {"count": 0})

        seq_span = (self.max_ext - self.min_ext + 1) if self.fc_rx else 0
        lost = max(0, seq_span - self.fc_rx)
        return {
            "role": "gcs",
            "rx_span_s": round(span, 3),
            "rx_frames": self.rx_frames,
            "rx_bytes": self.rx_bytes,
            "rx_rate_hz": round(self.rx_frames / span, 1),
            "rx_throughput_Bps": round(self.rx_bytes / span, 1),
            "crc_errors": self.crc_errors,
            "undecodable": self.undecodable,
            "non_frame": self.non_frame,
            "by_msgid": self.by_msgid,
            "est_loss_pct": round(100.0 * lost / seq_span, 2) if seq_span else 0.0,
            "lost_est": lost,
            "reorders": self.reorders,
            "ping": {**stat(rtt), "sent": self.ping_count,
                     "lost": self.ping_count - len(rtt)},
            "cmd": {"sent": self.cmd_count, "acked": self.cmd_acks,
                    "ack_pct": round(100.0 * self.cmd_acks / self.cmd_count, 1) if self.cmd_count else 0.0,
                    "ack_lat_ms_avg": round(sum(self.cmd_lat) / len(self.cmd_lat), 2) if self.cmd_lat else None},
            "egress": self.link.stats(),
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--role", choices=["fc", "gcs"], required=True)
    ap.add_argument("--listen", type=int, required=True)
    ap.add_argument("--peer", type=int, required=True)
    ap.add_argument("--config", default="{}")
    args = ap.parse_args()

    cfg = json.loads(args.config)
    duration = float(cfg.get("duration", 10.0))
    sock = udp_socket(args.listen)
    direction = 0 if args.role == "fc" else 1          # 0=downlink, 1=uplink
    link = Link(sock, ("127.0.0.1", args.peer), LinkConfig.from_dict(cfg), direction)

    ep = (FC if args.role == "fc" else GCS)(sock, link, cfg)
    report = ep.run(duration) if args.role == "gcs" else ep.run(duration)
    link.close()
    sock.close()
    print("REPORT " + json.dumps(report))


if __name__ == "__main__":
    main()
