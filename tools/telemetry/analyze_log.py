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
"""Summarise a VREC telemetry recording (Navigator export or handlift_record.py).

Same on-disk format either way (RecordFormat.h in vayu-navigator): a
[magic][fmtVer][protoVer][startWallMs] header then [t_us][len][bytes] records
holding RAW inbound datagrams — so a file records every field the FC sent, even
if the app that wrote it predates them.

    python3 tools/telemetry/analyze_log.py LOG.bin [LOG2.bin ...]
    python3 tools/telemetry/analyze_log.py --vertical LOG.bin   # per-sample table
"""
import collections
import os
import re
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "navlink", "sim"))
sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))
import frame  # noqa: E402
import navlink_msgs as N  # noqa: E402

NAME = {getattr(N, a).MSGID: a for a in dir(N) if hasattr(getattr(N, a), "MSGID")}
NAV = {0: "UNINIT", 1: "INIT", 2: "STANDBY", 3: "PREARM", 4: "ARMED",
       5: "IN_AIR", 6: "FAILSAFE", 7: "TERM", 8: "CALIB"}
HM = {0: "OFF", 1: "HOLD", 2: "LAND"}
HDR = struct.calcsize("<IIIQ")


def parse(path):
    """Yield (t_s, msgid, decoded_frame) for every frame in the file."""
    raw = open(path, "rb").read()
    if len(raw) < HDR:
        return
    off = HDR
    while off + 12 <= len(raw):
        t_us, ln = struct.unpack_from("<QI", raw, off)
        off += 12
        if off + ln > len(raw):
            break
        dg, off = raw[off:off + ln], off + ln
        o = 0
        # One datagram carries N coalesced frames (the ESP bridge packs them).
        while len(dg) - o >= frame.HDR_LEN + 2:
            if dg[o] != 0x56 or dg[o + 1] != 0x02:
                o += 1
                continue
            fl = frame.HDR_LEN + dg[o + 2] + 2
            if len(dg) - o < fl:
                break
            d = frame.decode(dg[o:o + fl])
            o += fl
            if d.ok:
                yield t_us / 1e6, d.msgid, d


def collect(path):
    got, lost, by, last = (collections.Counter(), collections.Counter(),
                           collections.Counter(), {})
    rows, tof, states = [], [], collections.Counter()
    motors, nav, t0, t1 = [0.0] * 4, None, None, None
    for t, mid, d in parse(path):
        if t0 is None:
            t0 = t
        t1 = t
        got[mid] += 1
        by[mid] += frame.HDR_LEN + len(d.payload) + 2
        p = last.get(mid)
        if p is not None:
            g = (d.seq - p - 1) & 0xFF
            if 0 < g < 64:      # bounded: ignore 256-wraps and reorders
                lost[mid] += g
        last[mid] = d.seq
        if mid == N.MotorTelemetry.MSGID:
            motors = list(N.MotorTelemetry.unpack(d.payload).cmd)[:4]
        elif mid == N.Heartbeat.MSGID:
            nav = N.Heartbeat.unpack(d.payload).nav_state
            states[nav] += 1
        elif mid == N.Statustext.MSGID:
            m = N.Statustext.unpack(d.payload)
            txt = (m.text if isinstance(m.text, str)
                   else bytes(m.text).decode("ascii", "replace")).split("\x00")[0]
            mm = re.search(r"VL53L0X:\s*(-?\d+) mm \(status (\d+)\)", txt)
            if mm:
                tof.append((int(mm.group(1)), int(mm.group(2))))
        elif mid == N.VerticalState.MSGID:
            m = N.VerticalState.unpack(d.payload)
            rows.append(dict(t=t, nav=nav, hs=m.height_state, agl=m.agl_tof,
                             tv=m.tof_valid, alt=m.altitude, cr=m.climb_rate,
                             va=m.vertical_accel, b=m.accel_bias,
                             unh=m.accel_unhealthy, thr=sum(motors) / 4.0))
    return dict(got=got, lost=lost, by=by, rows=rows, tof=tof, states=states,
                dur=(t1 - t0) if t0 is not None else 0.0)


def report(path, c):
    dur = c["dur"]
    tg, tl = sum(c["got"].values()), sum(c["lost"].values())
    size = os.path.getsize(path)
    print(f"=== {os.path.basename(path)} — {size/1024:.0f} KiB ===")
    if not tg:
        print("  NO DECODABLE FRAMES\n")
        return
    print(f"  {dur:.1f} s, {tg} frames ({tg/dur:.0f}/s), "
          f"{sum(c['by'].values())/dur/1024:.2f} KiB/s, loss {100*tl/(tg+tl):.1f}%")
    print(f"  states: {', '.join(f'{NAV.get(k,k)}x{v}' for k, v in c['states'].most_common())}")
    print(f"  {'stream':<18} {'n':>6} {'Hz':>6} {'loss%':>7}")
    for mid, n in c["got"].most_common(8):
        l = c["lost"][mid]
        print(f"  {NAME.get(mid,'?'):<18} {n:>6} {n/dur:>6.1f} {100*l/(n+l):>6.1f}%")
    r = c["rows"]
    if r:
        air = [x for x in r if x["nav"] == 5]
        print(f"  VerticalState: {len(r)} samples; IN_AIR {len(air)}; "
              f"max collective {max(x['thr'] for x in r):.2f}; "
              f"agl_tof max {max(x['agl'] for x in r):.2f} m; "
              f"accel_bias {min(x['b'] for x in r):+.2f}..{max(x['b'] for x in r):+.2f}; "
              f"unhealthy {'YES' if any(x['unh'] for x in r) else 'no'}")
        print(f"  height modes seen: {sorted(set(HM.get(x['hs']&3,'?') for x in r))}")
    if c["tof"]:
        s = collections.Counter(st for _, st in c["tof"])
        for st, n in s.most_common():
            v = [mm for mm, k in c["tof"] if k == st]
            print(f"  VL53L0X status {st:>2}: {n:>3} lines, {min(v)}..{max(v)} mm")
    print()


def vertical_table(c):
    prev = None
    print(f"{'t':>7} {'state':>8} {'mode':>5} {'thr':>5} {'agl':>6} {'tof':>3} "
          f"{'alt':>8} {'climb':>7} {'vacc':>6} {'bias':>7} {'unh':>3}")
    for r in c["rows"]:
        if prev and r["t"] - prev < 0.25:
            continue
        prev = r["t"]
        print(f"{r['t']:7.1f} {NAV.get(r['nav'],'?'):>8} {HM.get(r['hs']&3,'?'):>5} "
              f"{r['thr']:5.2f} {r['agl']:6.3f} {r['tv']:>3} {r['alt']:8.2f} "
              f"{r['cr']:+7.3f} {r['va']:+6.2f} {r['b']:+7.3f} {r['unh']:>3}")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    for p in args:
        c = collect(p)
        report(p, c)
        if "--vertical" in sys.argv:
            vertical_table(c)
