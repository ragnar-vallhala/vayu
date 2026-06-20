#!/usr/bin/env python3
"""Parse a recorded NavLink v2 telemetry session (.bin) into decoded frames.

On-disk container (software/src/replay/RecordFormat.h, little-endian):
    header:  [magic:u32 "VREC"][formatVersion:u32][protocolVersion:u32]
             [startWallClockMs:u64]            -> 20 bytes
    records: N x [t_us:u64][len:u32][bytes:len]

Each record `bytes` is a raw inbound chunk that may contain zero, one, or
several NavLink v2 frames (the recorder stores byte chunks, not packets), so we
feed every chunk through the generated incremental Parser, which resyncs and
CRC-validates each frame. We tag each decoded message with the record's t_us.

Usage:
    parse_log.py LOG.bin              # summary to stdout
    parse_log.py LOG.bin --csv DIR   # also dump per-message-type CSVs to DIR
"""
import argparse
import collections
import csv
import os
import struct
import sys

# Import the generated NavLink v2 Python codec (single source of truth).
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "navlink", "generated", "python"))
import navlink_msgs as nl  # noqa: E402

MAGIC = 0x56524543  # "VREC"
HDR = struct.Struct("<IIIQ")  # magic, formatVersion, protocolVersion, startWallClockMs
REC = struct.Struct("<QI")    # t_us, len


def read_records(path):
    """Yield (t_us, payload_bytes) for every record; also return the header."""
    with open(path, "rb") as f:
        blob = f.read()
    magic, fmt_ver, proto_ver, start_ms = HDR.unpack_from(blob, 0)
    if magic != MAGIC:
        raise SystemExit(f"bad magic 0x{magic:08x} (expected VREC 0x{MAGIC:08x})")
    header = dict(format_version=fmt_ver, protocol_version=proto_ver,
                  start_wall_clock_ms=start_ms, file_size=len(blob))
    off = HDR.size
    recs = []
    while off + REC.size <= len(blob):
        t_us, ln = REC.unpack_from(blob, off)
        off += REC.size
        if off + ln > len(blob):
            header["truncated_at"] = off
            break
        recs.append((t_us, blob[off:off + ln]))
        off += ln
    header["record_count"] = len(recs)
    header["trailing_bytes"] = len(blob) - off
    return header, recs


class Collector:
    """Accumulates every decoded frame, keyed by message class name."""
    def __init__(self):
        self.msgs = collections.defaultdict(list)   # name -> [(t_us, msg, frame)]
        self.crc_errors = 0
        self.unknown = collections.Counter()
        self._t_us = 0

    def _make_handlers(self):
        h = nl.Handlers()
        h.on_default = lambda fr, m: self._record(fr, m)
        # route every specific handler through the same sink
        for mid, hname in nl.MSGID_TO_HANDLER.items():
            setattr(h, hname, lambda fr, m: self._record(fr, m))
        h.on_crc_error = lambda fr: self._crc(fr)
        h.on_unknown = lambda fr: self.unknown.__setitem__(fr.msgid, self.unknown[fr.msgid] + 1)
        return h

    def _record(self, frame, msg):
        self.msgs[type(msg).__name__].append((self._t_us, msg, frame))

    def _crc(self, frame):
        self.crc_errors += 1

    def run(self, records):
        parser = nl.Parser(self._make_handlers())
        for t_us, payload in records:
            self._t_us = t_us
            parser.push(payload)
        return self


def fmt_dur(us):
    s = us / 1e6
    return f"{s:.1f} s ({s/60:.2f} min)" if s else "0 s"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--csv", metavar="DIR", help="dump per-message CSVs here")
    args = ap.parse_args()

    header, recs = read_records(args.log)
    col = Collector().run(recs)

    # time span across all decoded frames
    all_t = [t for lst in col.msgs.values() for (t, _, _) in lst]
    t0, t1 = (min(all_t), max(all_t)) if all_t else (0, 0)
    span = t1 - t0

    print(f"# {os.path.basename(args.log)}")
    print(f"file size        : {header['file_size']:,} bytes")
    print(f"format version   : {header['format_version']}")
    print(f"protocol version : {header['protocol_version']}")
    print(f"start wall clock : {header['start_wall_clock_ms']} ms "
          f"(unix {header['start_wall_clock_ms']/1000:.0f})")
    print(f"records          : {header['record_count']:,}")
    if header.get("trailing_bytes"):
        print(f"trailing bytes   : {header['trailing_bytes']}")
    print(f"time span        : {fmt_dur(span)}  [t_us {t0} .. {t1}]")
    print(f"crc errors       : {col.crc_errors}")
    if col.unknown:
        print(f"unknown msgids   : {dict(col.unknown)}")
    total = sum(len(v) for v in col.msgs.values())
    print(f"decoded frames   : {total:,}\n")

    print(f"{'message':<20}{'count':>8}{'rate Hz':>10}")
    print("-" * 38)
    for name in sorted(col.msgs, key=lambda n: -len(col.msgs[n])):
        n = len(col.msgs[name])
        rate = n / (span / 1e6) if span else 0
        print(f"{name:<20}{n:>8}{rate:>10.1f}")

    if args.csv:
        os.makedirs(args.csv, exist_ok=True)
        for name, lst in col.msgs.items():
            path = os.path.join(args.csv, f"{name}.csv")
            _, sample, _ = lst[0]
            fields = [f[0] for f in sample._FIELDS]
            with open(path, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t_us"] + fields)
                for t, m, _ in lst:
                    w.writerow([t] + [getattr(m, f) for f in fields])
        print(f"\nwrote {len(col.msgs)} CSVs to {args.csv}/")

    return col, header


if __name__ == "__main__":
    main()
