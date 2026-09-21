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
"""Standalone NavLink-over-UDP telemetry sniffer for real hardware.

Binds the GCS telemetry port (default 14555), broadcasts the `GCS-HELLO`
discovery datagram so the ESP8266 bridge learns this host and unicasts to us,
then decodes the live NavLink v2 stream with the SAME codec the GCS/sim use
(navlink/sim/frame.py + navlink/generated/python/navlink_msgs.py). No Qt / no
Navigator needed — useful for verifying that a flashed board is actually
streaming, and at what rates, straight from the terminal.

    python3 tools/telemetry/udp_telem_sniff.py                 # port 14555, run until Ctrl-C
    python3 tools/telemetry/udp_telem_sniff.py --port 14550
    python3 tools/telemetry/udp_telem_sniff.py --seconds 10    # auto-stop after 10 s
    python3 tools/telemetry/udp_telem_sniff.py --raw           # also dump each frame's fields

Every second it prints a table: per-message Hz, total count, last seq, plus a
peek at AttitudeEuler / FlightMode / Heartbeat so you can eyeball live attitude.
"""
import argparse
import collections
import csv
import os
import socket
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "navlink", "sim"))
sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))

import frame  # noqa: E402
from common import load_nl  # noqa: E402

nl = load_nl()

# Build MSGID -> message class registry by introspecting the generated codec.
MSG_BY_ID = {}
for _name in dir(nl):
    _obj = getattr(nl, _name)
    if isinstance(_obj, type) and hasattr(_obj, "MSGID") and hasattr(_obj, "unpack"):
        MSG_BY_ID[_obj.MSGID] = _obj


def name_of(msgid):
    cls = MSG_BY_ID.get(msgid)
    return cls.__name__ if cls else f"msgid_{msgid}"


def fmt_peek(msgid, payload):
    """Short human peek for a few high-value messages."""
    cls = MSG_BY_ID.get(msgid)
    if not cls:
        return ""
    try:
        m = cls.unpack(payload)
    except Exception as e:  # never let a bad frame kill the loop
        return f"(unpack err: {e})"
    nm = cls.__name__
    if nm == "AttitudeEuler":
        return (f"roll={getattr(m,'roll',0):+7.2f} pitch={getattr(m,'pitch',0):+7.2f} "
                f"yaw={getattr(m,'yaw',0):+7.2f}")
    if nm == "FlightMode":
        return f"mode={getattr(m,'mode','?')} src={getattr(m,'source','?')}"
    if nm == "Heartbeat":
        return (f"nav_state={getattr(m,'nav_state','?')} "
                f"status={getattr(m,'system_status','?')}")
    if nm == "Statustext":
        txt = getattr(m, "text", b"")
        if isinstance(txt, (bytes, bytearray)):
            txt = txt.split(b"\x00")[0].decode("ascii", "replace")
        return f"sev={getattr(m,'severity','?')} {txt!r}"
    return ""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=14555, help="UDP port to bind (default 14555)")
    ap.add_argument("--seconds", type=float, default=0, help="auto-stop after N s (0 = forever)")
    ap.add_argument("--hello-hz", type=float, default=1.0, help="GCS-HELLO broadcast rate")
    ap.add_argument("--raw", action="store_true", help="print every frame's peek line")
    ap.add_argument("--no-hello", action="store_true", help="don't broadcast GCS-HELLO")
    ap.add_argument("--raw-bin", metavar="PATH", default="",
                    help="record every inbound datagram verbatim to a VREC .bin "
                         "(navigator RecordFormat) so the whole stream replays / "
                         "decodes like a Navigator export.")
    ap.add_argument("--mag-csv", metavar="PATH", default="",
                    help="record IMU (incl. mag) to CSV, reconstructed to the full "
                         "~50 Hz rate from ImuRaw keyframes + ImuCompressed f16 "
                         "deltas. Columns: t,ax,ay,az,gx,gy,gz,mx,my,mz,temp.")
    args = ap.parse_args()

    # Optional IMU/mag recorder. ImuRaw (full snapshot, ~1.7 Hz) re-anchors a
    # running 10-vector [acc xyz, gyr xyz, mag xyz, temp]; each ImuCompressed
    # carries that vector's frame-to-frame f16 deltas (same field order), so
    # `vec += f16(delta)` reconstructs the 50 Hz stream (drop-free here: loss 0%).
    rec = None
    if args.mag_csv:
        import numpy as np  # only needed for the f16 decode

        def _f16(u):
            return float(np.array([u & 0xFFFF], dtype=np.uint16).view(np.float16)[0])

        _f = open(args.mag_csv, "w", newline="")
        _w = csv.writer(_f)
        _w.writerow(["t", "ax", "ay", "az", "gx", "gy", "gz",
                     "mx", "my", "mz", "temp"])
        rec = {"f": _f, "w": _w, "vec": None, "t0": None, "n": 0, "f16": _f16}

    def record(msgid, payload, now):
        if rec is None:
            return
        cls = MSG_BY_ID.get(msgid)
        if cls is None:
            return
        nm = cls.__name__
        if nm == "ImuRaw":
            m = cls.unpack(payload)
            rec["vec"] = [*m.acc, *m.gyr, *m.mag, m.temp]   # re-anchor
        elif nm == "ImuCompressed" and rec["vec"] is not None:
            m = cls.unpack(payload)
            d = [rec["f16"](x) for x in m.delta]            # 10 f16 deltas
            rec["vec"] = [rec["vec"][i] + d[i] for i in range(10)]
        else:
            return
        if rec["t0"] is None:
            rec["t0"] = now
        v = rec["vec"]
        rec["w"].writerow([f"{now - rec['t0']:.4f}"] + [f"{x:.4f}" for x in v])
        rec["n"] += 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(0.2)

    # Optional raw VREC recorder (RecordFormat.h in vayu-navigator):
    # header [magic u32][fmtVer u32][protoVer u32][startWallMs u64], then per
    # datagram [t_us u64][len u32][bytes]. Each datagram is one "raw chunk".
    raw_f = None
    raw_t0 = None
    raw_n = 0
    if args.raw_bin:
        raw_f = open(args.raw_bin, "wb")
        raw_f.write(struct.pack("<IIIQ", 0x56524543, 1, 1, int(time.time() * 1000)))

    print(f"[sniff] bound 0.0.0.0:{args.port}  "
          f"({'broadcasting GCS-HELLO' if not args.no_hello else 'no hello'})")
    print(f"[sniff] known msgids: {sorted(MSG_BY_ID)}")

    counts = collections.Counter()         # msgid -> total
    window = collections.Counter()         # msgid -> count this second
    last_seq = {}
    last_peek = {}
    bad = collections.Counter()            # reason -> count
    peer = None
    total_bytes = 0
    # Knee/throughput accounting (link bandwidth boost — firmware/docs/plans/link-bandwidth-boost.md).
    win_bytes = 0          # payload bytes received this second (effective KB/s)
    win_dgrams = 0         # UDP datagrams this second (ESP coalesces frames into these)
    win_lost = 0           # frames inferred dropped this second (per-msgid seq gaps)
    lost_total = 0
    frames_total = 0

    t0 = time.monotonic()
    last_hello = 0.0
    last_report = t0

    try:
        while True:
            now = time.monotonic()
            if args.seconds and now - t0 >= args.seconds:
                break
            if not args.no_hello and now - last_hello >= 1.0 / max(args.hello_hz, 0.1):
                sock.sendto(b"GCS-HELLO", ("255.255.255.255", args.port))
                last_hello = now

            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                data = None
            else:
                if data == b"GCS-HELLO":
                    pass  # our own loopback broadcast
                else:
                    if peer != addr:
                        peer = addr
                        print(f"[sniff] peer -> {addr[0]}:{addr[1]}")
                    total_bytes += len(data)
                    win_bytes += len(data)
                    win_dgrams += 1
                    if raw_f is not None:
                        if raw_t0 is None:
                            raw_t0 = now
                        raw_f.write(struct.pack("<QI", int((now - raw_t0) * 1e6),
                                                len(data)))
                        raw_f.write(data)
                        raw_n += 1
                    # One UDP datagram carries N coalesced NavLink frames (the ESP bridge
                    # packs whole frames up to the MTU). Walk them ALL — frame.decode()
                    # only parses the first, which silently undercounts under coalescing.
                    # Frame length is HDR_LEN + payload_len(byte[2]) + 2 CRC.
                    off = 0
                    while len(data) - off >= frame.HDR_LEN + 2:
                        if data[off] != 0x56 or data[off + 1] != 0x02:
                            off += 1  # resync past junk / a dropped-fragment boundary
                            continue
                        flen = frame.HDR_LEN + data[off + 2] + 2
                        if len(data) - off < flen:
                            break  # trailing partial frame (bridge is frame-aligned, so rare)
                        d = frame.decode(data[off:off + flen])
                        off += flen
                        if not d.ok:
                            bad[d.reason] += 1
                            continue
                        frames_total += 1
                        counts[d.msgid] += 1
                        window[d.msgid] += 1
                        # Frame-loss via the per-msgid 8-bit header seq: a gap = that many
                        # frames of this stream were dropped (bridge RX-ring overflow or
                        # WiFi loss). Bounded (<64) to ignore 256-wraps/reorders. This is
                        # the knee signal — loss climbs from ~0 as the rate passes the wall.
                        prev = last_seq.get(d.msgid)
                        if prev is not None:
                            gap = (d.seq - prev - 1) & 0xFF
                            if 0 < gap < 64:
                                win_lost += gap
                                lost_total += gap
                        last_seq[d.msgid] = d.seq
                        record(d.msgid, d.payload, now)
                        peek = fmt_peek(d.msgid, d.payload)
                        if peek:
                            last_peek[d.msgid] = peek
                        if args.raw and peek:
                            print(f"  {name_of(d.msgid):16s} {peek}")

            if now - last_report >= 1.0:
                dt = now - last_report
                win_frames = sum(window.values())
                kbps = win_bytes / 1024.0 / dt
                fps = win_frames / dt
                dps = win_dgrams / dt
                loss = 100.0 * win_lost / max(1, win_frames + win_lost)
                fpd = win_frames / win_dgrams if win_dgrams else 0.0
                # The knee line: watch loss% climb from ~0 as you crank TELEM_BASE_MS down.
                print(f"\n=== t+{now - t0:5.1f}s  peer={peer[0] if peer else '—'}  "
                      f"LINK {kbps:6.1f} KiB/s  {fps:6.0f} frame/s  {dps:5.0f} dgram/s "
                      f"({fpd:.1f} f/dg)  loss {loss:5.1f}%  "
                      f"bad={dict(bad) or '{}'} ===")
                win_bytes = win_dgrams = win_lost = 0
                for msgid in sorted(window, key=lambda m: -window[m]):
                    hz = window[msgid] / dt
                    line = (f"  {name_of(msgid):18s} {hz:6.1f} Hz  "
                            f"n={counts[msgid]:<7d} seq={last_seq.get(msgid,0):3d}")
                    if msgid in last_peek:
                        line += f"  {last_peek[msgid]}"
                    print(line)
                if not window:
                    print("  (no frames decoded this second)")
                window.clear()
                last_report = now
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        if raw_f is not None:
            raw_f.close()
            print(f"[sniff] raw VREC: {raw_n} datagrams -> {args.raw_bin}")
        if rec is not None:
            rec["f"].close()
            print(f"[sniff] mag CSV: {rec['n']} IMU rows -> {args.mag_csv}")
        dur = time.monotonic() - t0
        agg_loss = 100.0 * lost_total / max(1, frames_total + lost_total)
        print(f"\n[sniff] stopped after {dur:.1f}s. "
              f"{frames_total} frames, {total_bytes/1024:.1f} KiB "
              f"({total_bytes/1024/max(dur,1e-9):.1f} KiB/s avg), "
              f"~{lost_total} frames lost ({agg_loss:.1f}%).")
        print("[sniff] totals:")
        for msgid in sorted(counts, key=lambda m: -counts[m]):
            print(f"  {name_of(msgid):18s} n={counts[msgid]:<7d} "
                  f"avg {counts[msgid]/dur:6.1f} Hz")
        if bad:
            print(f"  decode failures: {dict(bad)}")
        if not counts and not bad:
            print("  NOTHING RECEIVED — check the board is powered, on the same "
                  "network, and the port matches the bridge (default 14555).")


if __name__ == "__main__":
    main()
