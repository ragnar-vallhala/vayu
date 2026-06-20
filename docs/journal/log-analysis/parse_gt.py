#!/usr/bin/env python3
"""Parse a Vayu ground-truth log (gt-*.bin) written by the SITL sim.

This is the PHYSICS truth (vsim SimSnapshot), the counterpart to the FC's own
telemetry (export-*.bin / sim-*.bin, decoded by parse_log.py). One run yields
both, so attitude/position estimate-vs-truth is an offline overlay.

On-disk container (little-endian), written by SimulatorWidget::logGroundTruth:
    header (20 bytes): [magic "VGT1"][version:u32][record_bytes:u32][start_wall_ms:u64]
    records: record_bytes each, packed:
        tick:u64  t_us:u64                          # physics tick, us since log open
        pos[3]:f32   quat_wxyz[4]:f32               # NED position [m], body->world quat
        vel[3]:f32   omega_b[3]:f32                 # NED vel [m/s], body rates [rad/s]
        roll,pitch,yaw:f32                          # NED Tait-Bryan [deg]
        motor_omega[4]:f32  motor_duty[4]:f32       # per-rotor [rad/s], [0,1]
        airspeed:f32  batt_voltage:f32  batt_soc:f32

Usage:
    parse_gt.py LOG.bin              # summary to stdout
    parse_gt.py LOG.bin --csv OUT.csv
"""
import argparse
import struct
import sys

HDR = struct.Struct("<4sIIQ")          # magic, version, record_bytes, start_wall_ms
REC = struct.Struct("<2Q27f")          # tick,t_us + 27 floats  (124 bytes)
FIELDS = ["tick", "t_us", "pos_n", "pos_e", "pos_d", "qw", "qx", "qy", "qz",
          "vel_n", "vel_e", "vel_d", "wx", "wy", "wz", "roll", "pitch", "yaw",
          "m_omega0", "m_omega1", "m_omega2", "m_omega3",
          "m_duty0", "m_duty1", "m_duty2", "m_duty3",
          "airspeed", "batt_v", "batt_soc"]


def read(path):
    with open(path, "rb") as f:
        blob = f.read()
    magic, ver, rb, start_ms = HDR.unpack_from(blob, 0)
    if magic != b"VGT1":
        raise SystemExit(f"bad magic {magic!r} (expected b'VGT1')")
    if rb != REC.size:
        raise SystemExit(f"record_bytes {rb} != parser {REC.size} — version skew")
    rows = []
    off = HDR.size
    while off + REC.size <= len(blob):
        rows.append(REC.unpack_from(blob, off))
        off += REC.size
    return dict(version=ver, start_wall_ms=start_ms, record_bytes=rb,
                trailing=len(blob) - off), rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--csv", metavar="OUT")
    args = ap.parse_args()
    hdr, rows = read(args.log)
    n = len(rows)
    print(f"# {args.log}")
    print(f"version          : {hdr['version']}")
    print(f"start wall clock : {hdr['start_wall_ms']} ms (unix {hdr['start_wall_ms']/1000:.0f})")
    print(f"records          : {n:,}")
    if not n:
        return
    t0, t1 = rows[0][1], rows[-1][1]
    span = (t1 - t0) / 1e6
    print(f"time span        : {span:.1f} s   rate {n/span:.1f} Hz" if span else "time span: 0")
    col = {f: [r[i] for r in rows] for i, f in enumerate(FIELDS)}

    def rng(f):
        xs = col[f]
        return f"{min(xs):+.2f} .. {max(xs):+.2f}"
    print(f"\nground-truth attitude (deg):")
    print(f"  roll  {rng('roll')}")
    print(f"  pitch {rng('pitch')}")
    print(f"  yaw   {rng('yaw')}")
    alt = [-d for d in col["pos_d"]]   # NED down -> altitude
    print(f"altitude (m)     : {min(alt):.1f} .. {max(alt):.1f}")
    print(f"vel_d (m/s)      : {rng('vel_d')}  (climb = -vel_d)")
    print(f"airspeed (m/s)   : {rng('airspeed')}")
    if hdr["trailing"]:
        print(f"trailing bytes   : {hdr['trailing']}")

    if args.csv:
        import csv
        with open(args.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(FIELDS)
            w.writerows(rows)
        print(f"\nwrote {n} rows to {args.csv}")


if __name__ == "__main__":
    main()
