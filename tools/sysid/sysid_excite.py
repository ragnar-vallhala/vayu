#!/usr/bin/env python3
# Copyright (C) 2026 NAVRobotec Pvt Ltd
# Author: Ragnar Vallhala
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""On-hardware system-ID excitation + high-rate capture (Phase 1) over NavLink/UDP.

Sends CMD_SYSID_EXCITE to inject a tapered linear chirp into one rate-loop
setpoint on the real FC, then CMD_SYSID_DUMP to pull the FC's RAM capture buffer
(rate_sp incl. chirp vs measured gyro, ~500 Hz) back as SYSID_SAMPLE chunks and
reassembles them into a CSV for plant fitting.

PHASE 0/1 SAFETY: run PROPS OFF (DISARMED) first. The chirp injects into the
setpoint but motors only move via the controller's ARMED gate, so a disarmed FC
captures the input path with props still. Only arm on the rig after the dump,
waveform, and abort all check out at low amplitude.

    python3 tools/sysid/sysid_excite.py --axis roll --f0 0.5 --f1 12 --amp 30 --dur 6
    # near-open-loop excitation (perturb the PID OUTPUT u directly; cleaner fit):
    python3 tools/sysid/sysid_excite.py --axis roll --inject u --amp 0.1 --dur 6
    python3 tools/sysid/sysid_excite.py --abort                 # stop a run now

Close the Navigator GCS first (UDP 14555 is single-owner).
"""
import argparse
import csv
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "navlink", "sim"))
sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))

import frame                       # noqa: E402
import navlink_msgs as nl          # noqa: E402

AXES = {"roll": 0, "pitch": 1, "yaw": 2}


class Link:
    """Minimal NavLink/UDP endpoint: discovery, send, recv-decode."""

    def __init__(self, port):
        self.port = port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.s.bind(("0.0.0.0", port))
        self.s.settimeout(0.1)
        self.peer = None
        self.seq = 0
        self._last_hello = 0.0

    def pump_hello(self):
        now = time.monotonic()
        if now - self._last_hello >= 0.5:
            self.s.sendto(b"GCS-HELLO", ("255.255.255.255", self.port))
            self._last_hello = now

    def send(self, msgid, payload):
        if not self.peer:
            return
        self.seq = (self.seq + 1) & 0xFF
        self.s.sendto(frame.encode(msgid, payload, seq=self.seq), self.peer)

    def recv(self):
        try:
            data, addr = self.s.recvfrom(2048)
        except socket.timeout:
            return None
        if data == b"GCS-HELLO":
            return None
        self.peer = addr
        d = frame.decode(data)
        return d if d.ok else None

    def wait_peer(self, timeout=5.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.pump_hello()
            self.recv()
            if self.peer:
                return True
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=14555)
    ap.add_argument("--axis", choices=list(AXES), default="roll")
    ap.add_argument("--f0", type=float, default=0.5)
    ap.add_argument("--f1", type=float, default=12.0)
    ap.add_argument("--amp", type=float, default=None,
                    help="chirp amplitude. Units depend on --inject: deg/s for "
                         "'sp' (default 30), control-effort u for 'u' (default "
                         "0.10; FC hard-caps at 0.30). u-injection drives the "
                         "motors directly — start small.")
    ap.add_argument("--inject", choices=("sp", "u"), default="sp",
                    help="excitation point: 'sp' perturbs the rate SETPOINT "
                         "(closed-loop, safe, default); 'u' perturbs the rate-PID "
                         "OUTPUT directly (near-open-loop, cleaner plant fit, more "
                         "aggressive — prefer a stable rig/hover).")
    ap.add_argument("--dur", type=float, default=6.0)
    ap.add_argument("--abort", action="store_true")
    ap.add_argument("--csv", default="/tmp/sysid_dump.csv")
    ap.add_argument("--trigger-throttle", type=float, default=0.0,
                    help="wait until CONTROL_TRACE thro_out >= this (held ~0.3s) "
                         "before firing the chirp; 0 = fire immediately")
    ap.add_argument("--trigger-timeout", type=float, default=30.0)
    args = ap.parse_args()
    axis = AXES[args.axis]
    mode = 1 if args.inject == "u" else 0
    # Mode-dependent amplitude default (deg/s for setpoint, control-effort for u).
    if args.amp is None:
        args.amp = 0.10 if mode == 1 else 30.0

    try:
        link = Link(args.port)
    except OSError as e:
        print(f"[sysid] cannot bind :{args.port} ({e}). Close Navigator GCS first.")
        return 2
    if not link.wait_peer():
        print("[sysid] no FC seen — is it powered and bridged?")
        return 1
    print(f"[sysid] peer {link.peer[0]}:{link.peer[1]}")

    # TIME_SYNC then the command (mirrors the GCS handshake).
    link.send(nl.TimeSync.MSGID, nl.TimeSync(role=0, seq=1,
              t1_gcs_tx=int(time.time() * 1e6)).pack())
    time.sleep(0.05)

    if args.abort:
        link.send(nl.CmdSysidExcite.MSGID,
                  nl.CmdSysidExcite(target_sys=42, target_comp=1, req_seq=link.seq,
                                    axis=0xFF, f0_hz=0, f1_hz=0, amp_dps=0,
                                    duration_s=0, mode=0).pack())
        print("[sysid] ABORT sent")
        time.sleep(0.3)
        return 0

    # Optionally wait until the throttle is up in the full-authority band before
    # firing, so the operator just raises+holds throttle and the chirp self-times.
    if args.trigger_throttle > 0:
        print(f"[sysid] waiting for throttle >= {args.trigger_throttle:.2f} "
              f"(raise + hold; timeout {args.trigger_timeout:.0f}s)...")
        t0 = time.monotonic()
        above_since = None
        last_print = 0.0
        thr = 0.0
        gpk = 0.0
        while time.monotonic() - t0 < args.trigger_timeout:
            link.pump_hello()
            d = link.recv()
            now = time.monotonic()
            if d and d.msgid == nl.ControlTrace.MSGID:
                m = nl.ControlTrace.unpack(d.payload)
                thr = m.thro_out
                gpk = max(gpk, abs([m.roll_rate_curr, m.pitch_rate_curr,
                                    m.yaw_rate_curr][axis]))
                if thr >= args.trigger_throttle:
                    above_since = above_since or now
                    if now - above_since >= 0.3:
                        break
                else:
                    above_since = None
            if now - last_print >= 1.0:
                last_print = now
                print(f"  throttle={thr:.2f}  gyro_pk={gpk:.0f}dps")
        else:
            print("[sysid] throttle never reached threshold — aborting (no chirp "
                  "sent). Disarm.")
            return 1
        print(f"[sysid] throttle {thr:.2f} held -> FIRING chirp")

    link.send(nl.CmdSysidExcite.MSGID,
              nl.CmdSysidExcite(target_sys=42, target_comp=1, req_seq=link.seq,
                                axis=axis, f0_hz=args.f0, f1_hz=args.f1,
                                amp_dps=args.amp, duration_s=args.dur, mode=mode).pack())
    amp_unit = "u" if mode == 1 else "dps"
    print(f"[sysid] EXCITE {args.axis} inject={args.inject} f0={args.f0} f1={args.f1} "
          f"amp={args.amp}{amp_unit} dur={args.dur}s")

    # Wait out the run (the FC captures to RAM at ~500 Hz), keeping the bridge warm.
    t_end = time.monotonic() + args.dur + 0.8
    while time.monotonic() < t_end:
        link.pump_hello()
        link.recv()

    # Pull the capture via CMD_SYSID_DUMP, reassembling chunks by start_index.
    # Re-send a few times to refill any chunks the lossy bridge dropped.
    samples = {}  # index -> (sp_dps, gyro_dps)
    meta = {}
    for attempt in range(8):
        link.send(nl.CmdSysidDump.MSGID,
                  nl.CmdSysidDump(target_sys=42, target_comp=1,
                                  req_seq=link.seq).pack())
        t0 = time.monotonic()
        last_new = t0
        while time.monotonic() - t0 < 15.0:       # hard cap per pass
            link.pump_hello()
            d = link.recv()
            now = time.monotonic()
            if not d or d.msgid != nl.SysidSample.MSGID:
                if now - last_new > 3.0:           # paced stream finished
                    break
                continue
            m = nl.SysidSample.unpack(d.payload)
            meta = {"total": m.total, "hz": m.capture_hz, "axis": m.axis}
            before = len(samples)
            for i in range(m.count):
                idx = m.start_index + i
                # slot scales: u (PID output) x1000, gyro (deg/s) x10.
                samples[idx] = (m.u[i] / 1000.0, m.gyro[i] / 10.0)
            if len(samples) > before:
                last_new = now
        total = meta.get("total", 0)
        got = len(samples)
        print(f"[sysid] dump pass {attempt + 1}: {got}/{total} samples")
        if total and got >= total:
            break
        if not total:
            print("[sysid] no SYSID_SAMPLE received — old firmware? capture empty?")
            return 1

    total = meta.get("total", 0)
    hz = meta.get("hz", 1) or 1
    if not samples:
        print("[sysid] nothing captured.")
        return 1

    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "u", "gyro_dps"])
        for idx in sorted(samples):
            u, gy = samples[idx]
            w.writerow([f"{idx / hz:.4f}", f"{u:.4f}", f"{gy:.2f}"])

    u = [v[0] for v in samples.values()]
    gy = [v[1] for v in samples.values()]
    missing = total - len(samples)
    print(f"[sysid] capture: {len(samples)}/{total} samples @ {hz} Hz "
          f"(axis {meta.get('axis')}), {missing} missing -> {args.csv}")
    print(f"  u (PID out): [{min(u):+.3f} .. {max(u):+.3f}]")
    print(f"  gyro       : [{min(gy):+.1f} .. {max(gy):+.1f}] dps")
    if max(abs(min(gy)), abs(max(gy))) < 2.0:
        print("  (gyro ~flat => disarmed / motors off; fit needs ARMED data with a "
              "real gyro response)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
