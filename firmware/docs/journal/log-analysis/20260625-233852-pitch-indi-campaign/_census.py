#!/usr/bin/env python3
"""Message census across every bin in the campaign: which message types are
present, how many, and effective rate. No averaging of signal content — this is
purely an inventory so we know which pipeline stages each capture actually
recorded."""
import os, sys, struct, glob
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "navlink", "sim"))
sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))
import frame
import navlink_msgs as nl

NAME = {getattr(nl, n).MSGID: n for n in dir(nl)
        if hasattr(getattr(nl, n), "MSGID")}

def walk(path):
    with open(path, "rb") as f:
        blob = f.read()
    magic, fmt, proto, startms = struct.unpack_from("<IIIQ", blob, 0)
    assert magic == 0x56524543, f"bad VREC magic {magic:#x}"
    off = 20
    while off + 12 <= len(blob):
        t_us, dlen = struct.unpack_from("<QI", blob, off)
        off += 12
        data = blob[off:off + dlen]; off += dlen
        t = t_us / 1e6
        p = 0
        while len(data) - p >= frame.HDR_LEN + 2:
            if data[p] != 0x56 or data[p + 1] != 0x02:
                p += 1; continue
            flen = frame.HDR_LEN + data[p + 2] + 2
            if len(data) - p < flen:
                break
            d = frame.decode(data[p:p + flen]); p += flen
            if d.ok:
                yield t, d.msgid, d.payload

def census(path):
    counts, times = {}, {}
    t0 = t1 = None
    for t, mid, pl in walk(path):
        if t0 is None: t0 = t
        t1 = t
        counts[mid] = counts.get(mid, 0) + 1
        times.setdefault(mid, []).append(t)
    span = (t1 - t0) if t0 is not None else 0.0
    print(f"\n=== {os.path.relpath(path, HERE)} ===  span {span:.1f}s")
    for mid in sorted(counts, key=lambda m: -counts[m]):
        ts = np.array(times[mid])
        rate = len(ts) / span if span > 0 else 0
        print(f"   {NAME.get(mid,'?'+str(mid)):20s} n={counts[mid]:6d}  ~{rate:6.1f} Hz")

if __name__ == "__main__":
    bins = sys.argv[1:] or sorted(glob.glob(os.path.join(HERE, "*", "*.bin")))
    for b in bins:
        census(b)
