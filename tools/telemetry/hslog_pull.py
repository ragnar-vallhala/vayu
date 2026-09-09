#!/usr/bin/env python3
"""Pull the live part of the high-speed IMU recording (0:imuhs.bin) off the FC.

The file is preallocated to its full ring size (32 MB on hardware) and is never
truncated, so a plain download would transfer tens of megabytes of clusters the
recorder has not reached yet -- hours over the telemetry link. HSL_STATUS says
how much of the ring actually holds data, so this asks for exactly that and
stops: preamble sector + the live ring slots.

  1. Listen for HSL_STATUS to learn head_slot / wraps / ring_sectors.
  2. XFER_OPEN a DOWNLOAD and ack chunks until the contiguous cursor covers the
     live bytes, then XFER_CLOSE -- the FC's own flow control does the rest.
  3. Write a file that is byte-identical to the head of the SD file, so
     hslog.py reads it exactly as it would read the card.

A wrapped ring is pulled whole (every slot holds data). Usage:

    python3 tools/telemetry/hslog_pull.py -o flight.hsl
"""
import argparse
import collections
import os
import socket
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, "..", "..", "navigator", "headless-sdk"))

import fs_xfer_udp_test as X  # Bridge, discover_and_sync, the xfer constants
from vayu_headless.transport import navlink as _hnl

nl = X.nl
SECTOR = 512


def read_status(port, secs):
    """Listen for one HSL_STATUS. Uses its own socket, so it must NOT overlap
    with the xfer bridge -- only one process may bind :14555."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    s.settimeout(1.0)
    buf, msgs, counts = bytearray(), {}, collections.Counter()
    t0 = time.time()
    try:
        while time.time() - t0 < secs:
            try:
                d, _ = s.recvfrom(4096)
            except socket.timeout:
                continue
            buf += d
            _hnl.parse_frames(buf, msgs, counts)
            if "HslStatus" in msgs:
                return msgs["HslStatus"]
    finally:
        s.close()
    return None


def bounded_download(br, session, path, want, timeout):
    """X.download() but stopping at `want` contiguous bytes instead of EOF."""
    print(f"[dn ] XFER_OPEN download '{path}', want {want} B")
    t0 = time.monotonic()
    total = None
    rx = None
    cursor = 0
    have = []
    last_open = last_ack = 0.0
    t_first = None
    while cursor < want and time.monotonic() - t0 < timeout:
        t = time.monotonic()
        if total is None and t - last_open >= 0.5:
            br.send(nl.XferOpen(target_sys=X.FC_SYS, target_comp=X.FC_COMP,
                                req_seq=2, session=session, dir=X.DIR_DOWNLOAD,
                                mode=X.MODE_FILE, service_id=X.SVC_FILE,
                                offset_start=0, rate_hz=0, arg=path))
            last_open = t
        for d in br.poll():
            if d.msgid == nl.XferInfo.MSGID:
                info = nl.XferInfo.unpack(d.payload)
                if info.session != session:
                    continue
                if info.result != X.R_OK:
                    print(f"[dn ] open REJECTED result={info.result}", file=sys.stderr)
                    return None
                total = info.total_size
                rx = bytearray(total)
                print(f"[dn ] INFO total_size={total} chunk={info.chunk_size} "
                      f"-> pulling {want} ({100.0*want/total:.2f}%)")
            elif d.msgid == nl.XferData.MSGID and rx is not None:
                m = nl.XferData.unpack(d.payload)
                if m.session != session:
                    continue
                if t_first is None:
                    t_first = time.monotonic()
                end = m.offset + m.len
                if end <= len(rx):
                    rx[m.offset:end] = bytes(m.data[:m.len])
                    have.append((m.offset, m.len))
                    have.sort()
                    for off, ln in have:
                        if off <= cursor < off + ln or off == cursor:
                            cursor = off + ln
                if cursor >= want:
                    break
                flags = X.F_NAK if m.offset > cursor else X.F_NONE
                br.send(nl.XferAck(session=session, flags=flags, result=X.R_OK,
                                   next_offset=cursor))
                last_ack = t
        if total is not None and cursor < want and t - last_ack > 0.3:
            br.send(nl.XferAck(session=session, flags=X.F_NAK, result=X.R_OK,
                               next_offset=cursor))
            last_ack = t
        time.sleep(0.002)

    # We are abandoning a transfer the FC still thinks is running, so close it
    # explicitly -- otherwise its read fd stays held and the next open collides.
    br.send(nl.XferClose(target_sys=X.FC_SYS, target_comp=X.FC_COMP, req_seq=9,
                         session=session, result=X.R_OK))
    if cursor < want:
        print(f"[dn ] INCOMPLETE {cursor}/{want} B", file=sys.stderr)
        return None
    dt = time.monotonic() - (t_first or t0)
    print(f"[dn ] got {cursor} B in {dt:.1f}s => {cursor/1024.0/dt:.1f} KiB/s")
    return bytes(rx[:cursor])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--out", default="imuhs.bin")
    ap.add_argument("--path", default="0:imuhs.bin")
    ap.add_argument("--port", type=int, default=14555)
    ap.add_argument("--session", type=int, default=3)
    ap.add_argument("--timeout", type=float, default=600.0)
    ap.add_argument("--sectors", type=int, default=0,
                    help="override: pull this many ring sectors instead of "
                         "asking HSL_STATUS")
    a = ap.parse_args()

    sectors = a.sectors
    if not sectors:
        print("[st ] listening for HSL_STATUS…")
        st = read_status(a.port, 12)
        if st is None:
            print("no HSL_STATUS heard -- is the FC up, and is :14555 free?",
                  file=sys.stderr)
            return 1
        used = st.ring_sectors if st.wraps else st.head_slot
        print(f"[st ] session={st.session} head_slot={st.head_slot} "
              f"wraps={st.wraps} dropped={st.dropped_sectors} -> {used} live sectors")
        if st.dropped_sectors:
            print(f"[st ] WARNING {st.dropped_sectors} sectors were dropped; the "
                  f"card fell behind and those samples do not exist")
        if used == 0:
            print("nothing recorded yet", file=sys.stderr)
            return 1
        sectors = used

    want = (1 + sectors) * SECTOR  # preamble + live ring slots
    br = X.Bridge(a.port)
    X.discover_and_sync(br)
    data = bounded_download(br, a.session, a.path, want, a.timeout)
    if data is None:
        return 1
    with open(a.out, "wb") as fh:
        fh.write(data)
    print(f"wrote {a.out}  {len(data)} B  ({sectors} ring sectors)")
    print(f"  python3 {os.path.relpath(os.path.join(_HERE,'hslog.py'))} {a.out} --fft")
    return 0


if __name__ == "__main__":
    sys.exit(main())
