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
"""Fit and write the FC's battery divider calibration (0:batcal.bin).

The scale is not computable. On the first board to carry this the nominal 37k/4k
divider predicts 0.0976 V/count and the bench measures 0.0114 -- 37% out,
because PA0 is also SYS_WKUP1 and something loads the low side. So it is
measured, and because it belongs to the aircraft rather than the binary it lives
on the card, not in a header.

TWO points, not one. A single reference can only fit a slope through the origin,
and the line does not pass through the origin: the converter and the divider
both contribute a zero error. Take both points as far apart as the pack allows
-- a charged pack and a nearly flat one are ideal -- because the offset is the
intercept, and a short baseline puts all the measurement error straight into it.

    # read counts live from the FC while measuring volts with a multimeter
    python3 tools/calib/battery_calib.py fit --point 12.46:1091 --point 7.95:698
    # then put it on the card
    python3 tools/calib/battery_calib.py fit --point ... --point ... --upload

`counts` comes from the BATTERY message (field `counts`), which is on the wire
beside the voltage for exactly this purpose. udp_telem_sniff.py shows it live.
"""
import argparse
import struct
import sys

MAGIC = 0x4C414342  # 'BCAL'
VERSION = 1
PATH = "0:batcal.bin"

# Must match storage/battery_calib.h -- the loader refuses anything outside.
VPC_MIN, VPC_MAX = 0.001, 0.050
OFFSET_ABS_MAX = 200.0


def fit(points):
    """Least-squares line through (counts, volts) -> (volts_per_count, offset_counts).

    Returned in the form the firmware applies, volts = (counts - offset) * vpc,
    so the offset is an x-intercept in counts rather than a voltage.
    """
    if len(points) < 2:
        raise SystemExit("need at least two points: a scale AND an offset")
    xs = [c for c, _ in points]
    ys = [v for _, v in points]
    n = len(points)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        raise SystemExit("all points have the same count -- no baseline to fit")
    vpc = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    if vpc <= 0:
        raise SystemExit(f"fitted scale is not positive ({vpc:.6g}): points inconsistent")
    # y = vpc*(x - offset)  =>  offset = x_mean - y_mean/vpc
    offset = mx - my / vpc
    return vpc, offset


def check(vpc, offset):
    bad = []
    if not VPC_MIN < vpc < VPC_MAX:
        bad.append(f"volts_per_count {vpc:.6f} outside [{VPC_MIN}, {VPC_MAX}]")
    if not -OFFSET_ABS_MAX < offset < OFFSET_ABS_MAX:
        bad.append(f"offset {offset:.1f} counts outside +/-{OFFSET_ABS_MAX}")
    return bad


def pack(vpc, offset):
    """16 bytes, matching battery_calib_store_t exactly (LE, explicit pad)."""
    return struct.pack("<IHHff", MAGIC, VERSION, 0, vpc, offset)


def parse_point(s):
    try:
        v, c = s.split(":")
        return (float(c), float(v))
    except ValueError:
        raise SystemExit(f"--point wants VOLTS:COUNTS, got {s!r}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fit", help="fit two or more points and write the file")
    f.add_argument("--point", action="append", required=True, metavar="VOLTS:COUNTS",
                   help="a multimeter reading and the counts the FC reported")
    f.add_argument("-o", "--out", default="batcal.bin")
    f.add_argument("--upload", action="store_true",
                   help=f"also upload to {PATH} over the telemetry link")
    s = sub.add_parser("show", help="decode an existing file")
    s.add_argument("file")
    a = ap.parse_args()

    if a.cmd == "show":
        blob = open(a.file, "rb").read()
        if len(blob) != 16:
            raise SystemExit(f"{a.file}: expected 16 bytes, got {len(blob)}")
        magic, ver, _pad, vpc, offset = struct.unpack("<IHHff", blob)
        print(f"magic            0x{magic:08X} {'OK' if magic == MAGIC else 'MISMATCH'}")
        print(f"version          {ver} {'OK' if ver == VERSION else 'MISMATCH'}")
        print(f"volts_per_count  {vpc:.6f}")
        print(f"offset_counts    {offset:.2f}")
        for b in check(vpc, offset):
            print(f"  REJECTED: {b}")
        return 0

    points = [parse_point(p) for p in a.point]
    vpc, offset = fit(points)
    print("points (counts -> volts):")
    for c, v in sorted(points):
        print(f"  {c:7.1f} -> {v:7.3f} V     fit {((c - offset) * vpc):7.3f} V"
              f"   residual {((c - offset) * vpc - v) * 1000:+6.1f} mV")
    print(f"\nvolts_per_count  {vpc:.6f}")
    print(f"offset_counts    {offset:+.2f}")
    bad = check(vpc, offset)
    if bad:
        for b in bad:
            print(f"  REJECTED: {b}", file=sys.stderr)
        raise SystemExit("the firmware loader would refuse this -- not written")

    blob = pack(vpc, offset)
    with open(a.out, "wb") as fh:
        fh.write(blob)
    print(f"\nwrote {a.out} ({len(blob)} B)")

    if a.upload:
        sys.path.insert(0, "tools/telemetry")
        import fs_xfer_udp_test as X
        br = X.Bridge(14555)
        if not X.discover_and_sync(br):
            raise SystemExit("no FC on the link")
        X.upload(br, 1, PATH, blob, rate_bps=20000, window=4)
        print(f"uploaded to {PATH} -- power-cycle or reboot for it to take effect")
    else:
        print(f"upload with:  --upload   (or copy to {PATH} on the card)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
