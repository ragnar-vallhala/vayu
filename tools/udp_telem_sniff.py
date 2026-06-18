#!/usr/bin/env python3
"""Standalone NavLink-over-UDP telemetry sniffer for real hardware.

Binds the GCS telemetry port (default 14555), broadcasts the `GCS-HELLO`
discovery datagram so the ESP8266 bridge learns this host and unicasts to us,
then decodes the live NavLink v2 stream with the SAME codec the GCS/sim use
(navlink/sim/frame.py + navlink/generated/python/navlink_msgs.py). No Qt / no
Navigator needed — useful for verifying that a flashed board is actually
streaming, and at what rates, straight from the terminal.

    python3 tools/udp_telem_sniff.py                 # port 14555, run until Ctrl-C
    python3 tools/udp_telem_sniff.py --port 14550
    python3 tools/udp_telem_sniff.py --seconds 10    # auto-stop after 10 s
    python3 tools/udp_telem_sniff.py --raw           # also dump each frame's fields

Every second it prints a table: per-message Hz, total count, last seq, plus a
peek at AttitudeEuler / FlightMode / Heartbeat so you can eyeball live attitude.
"""
import argparse
import collections
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
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
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(0.2)

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
                    d = frame.decode(data)
                    if d.ok:
                        counts[d.msgid] += 1
                        window[d.msgid] += 1
                        last_seq[d.msgid] = d.seq
                        peek = fmt_peek(d.msgid, d.payload)
                        if peek:
                            last_peek[d.msgid] = peek
                        if args.raw and peek:
                            print(f"  {name_of(d.msgid):16s} {peek}")
                    else:
                        bad[d.reason] += 1

            if now - last_report >= 1.0:
                dt = now - last_report
                print(f"\n=== t+{now - t0:5.1f}s  peer={peer[0] if peer else '—'}  "
                      f"{total_bytes/1024:.1f} KiB total  "
                      f"bad={dict(bad) or '{}'} ===")
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
        dur = time.monotonic() - t0
        print(f"\n[sniff] stopped after {dur:.1f}s. totals:")
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
