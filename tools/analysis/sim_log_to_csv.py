#!/usr/bin/env python3
"""
sim_log_to_csv.py - decode a vayu sim log (.bin from logs/sim-*.bin) into
per-packet-type CSVs.

The .bin file is the raw UART2 byte stream written by SimulatorWidget while
the in-app SITL runs. Each frame is a DroneProtocol packet:
   0x56  type|version  len  device_id  timestamp(4 LE)  payload(len)  crc32(4 LE)
We parse every well-formed frame (STM32 CRC32 variant -- poly 0x04C11DB7,
init 0xFFFFFFFF, no input/output reflection, no final XOR) and split by
packet type into one CSV per stream:

  heartbeat.csv          - just timestamps
  imu.csv                - acc xyz, gyr xyz (dps), mag xyz, temp; reconstructed
                           from IMU_FULL keyframes + IMU_COMPRESSED fp16 deltas
  attitude.csv           - roll pitch yaw (deg)
  rc.csv                 - up to 14 channel us values
  motor.csv              - 4 motor duty floats (0..1)
  state.csv              - state machine transitions
  pid.csv                - 18-float PID telemetry (origin 0x05 of SYS_STATUS)
  log.csv                - vayu_log() text lines
  unknown.csv            - any SYS_STATUS we don't know how to decode

Usage:
  ./sim_log_to_csv.py <log.bin> [--out-dir DIR]

If --out-dir is omitted, CSVs land next to the .bin (with the same stem).
"""

from __future__ import annotations

import argparse
import csv
import os
import struct
import sys
from collections import defaultdict


def stm32_crc32(data: bytes) -> int:
    """STM32 hardware CRC32 - same variant as the firmware + GCS uses."""
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b << 24
        for _ in range(8):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc


def fp16_to_f32(u: int) -> float:
    """IEEE 754 binary16 -> binary32 (no NumPy dep)."""
    s = (u >> 15) & 1
    e = (u >> 10) & 0x1F
    m = u & 0x3FF
    if e == 0:
        if m == 0:
            return -0.0 if s else 0.0
        v = (m / 1024.0) * (2 ** -14)
    elif e == 31:
        if m:
            return float("nan")
        return float("-inf") if s else float("inf")
    else:
        v = (1.0 + m / 1024.0) * (2 ** (e - 15))
    return -v if s else v


# Packet type IDs from include/comm/comm_types.h.
PACKET_HEARTBEAT       = 0x0
PACKET_IMU_DATA_FULL   = 0x1
PACKET_IMU_COMPRESSED  = 0x2
PACKET_COMMAND         = 0x3
PACKET_ATTITUDE        = 0x4
PACKET_RC_CHANNELS     = 0x5
PACKET_SYSTEM_STATUS   = 0x6
PACKET_LOG             = 0x7
PACKET_MOTOR_TELEMETRY = 0x8

# SYSTEM_STATUS origin bytes (payload[0]).
ORIGIN_SYS_STATE  = 0x04
ORIGIN_PID_ERROR  = 0x05

# Human-readable state names from include/sys/state.h.
STATE_NAMES = {
    0x001: "UNINITIALIZED",
    0x002: "INIT",
    0x004: "STANDBY",
    0x008: "PREARM",
    0x010: "ARMED",
    0x020: "IN_AIR",
    0x040: "FAILSAFE",
    0x080: "TERMINATED",
    0x100: "CALIBRATING",
}


def parse_frames(raw: bytes):
    """Generator yielding (idx, type, timestamp_ms, device_id, payload) per
    valid frame."""
    i = 0
    count = 0
    bad_crc = 0
    while i < len(raw) - 12:
        # Header byte 0x56 + version nibble 0x01
        if raw[i] == 0x56 and (raw[i + 1] & 0x0F) == 0x01:
            length = raw[i + 2]
            total = 8 + length + 4
            if i + total > len(raw):
                break
            pkt = raw[i:i + total]
            expected = int.from_bytes(pkt[-4:], "little")
            if stm32_crc32(pkt[:-4]) == expected:
                ptype = (pkt[1] >> 4) & 0x0F
                device_id = pkt[3]
                ts = int.from_bytes(pkt[4:8], "little")
                payload = pkt[8:8 + length]
                yield count, ptype, ts, device_id, payload
                count += 1
                i += total
                continue
            bad_crc += 1
        i += 1
    if bad_crc:
        print(f"  warning: {bad_crc} CRC failures", file=sys.stderr)


# --- per-type decoders -----------------------------------------------------

def write_heartbeat(rows, out):
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "t_s"])
        for _, ts, _ in rows:
            w.writerow([ts, f"{ts / 1000.0:.4f}"])


def write_attitude(rows, out):
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "t_s", "roll_deg", "pitch_deg", "yaw_deg"])
        for _, ts, payload in rows:
            if len(payload) != 12:
                continue
            r, p, y = struct.unpack("<3f", payload)
            w.writerow([ts, f"{ts / 1000.0:.4f}",
                        f"{r:.4f}", f"{p:.4f}", f"{y:.4f}"])


def write_motor(rows, out):
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "t_s", "m1", "m2", "m3", "m4"])
        for _, ts, payload in rows:
            if len(payload) != 16:
                continue
            m = struct.unpack("<4f", payload)
            w.writerow([ts, f"{ts / 1000.0:.4f}",
                        *[f"{x:.6f}" for x in m]])


def write_rc(rows, out):
    # RC payload is N uint16 (N = len/2). Firmware sends up to 14 channels;
    # we just emit whatever's there.
    if not rows:
        return
    max_ch = max((len(p) // 2) for _, _, p in rows)
    headers = ["t_ms", "t_s"] + [f"ch{i}_us" for i in range(max_ch)]
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(headers)
        for _, ts, payload in rows:
            n = len(payload) // 2
            chs = struct.unpack(f"<{n}H", payload[:n * 2])
            row = [ts, f"{ts / 1000.0:.4f}"] + list(chs) + [""] * (max_ch - n)
            w.writerow(row)


def write_imu(full_rows, comp_rows, out):
    """Reconstruct the absolute IMU stream from FULL keyframes +
    COMPRESSED fp16 deltas. The compressed packet carries 10 fp16 deltas
    of [acc xyz, gyr xyz, mag xyz, temp]; we accumulate them on top of
    the most recent FULL keyframe."""
    # Merge by parse order so deltas always come after their keyframe.
    merged = []
    for idx, ts, payload in full_rows:
        if len(payload) == 40:
            vals = list(struct.unpack("<10f", payload))
            merged.append((idx, ts, "F", vals))
    for idx, ts, payload in comp_rows:
        if len(payload) == 20:
            merged.append((idx, ts, "C", payload))
    merged.sort(key=lambda r: r[0])

    state = None
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "t_s", "kind",
                    "ax", "ay", "az",
                    "gx_dps", "gy_dps", "gz_dps",
                    "mx", "my", "mz", "temp"])
        for _idx, ts, kind, payload in merged:
            if kind == "F":
                state = list(payload)
            else:
                if state is None:
                    continue  # no keyframe yet
                deltas = struct.unpack("<10H", payload)
                for i, d in enumerate(deltas):
                    state[i] += fp16_to_f32(d)
            w.writerow([ts, f"{ts / 1000.0:.4f}", kind] +
                       [f"{v:.6f}" for v in state])


def write_state(rows, out):
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "t_s", "state_value", "state_name"])
        for _, ts, payload in rows:
            if len(payload) != 6 or payload[0] != ORIGIN_SYS_STATE:
                continue
            (val,) = struct.unpack("<f", payload[2:])
            s = int(val)
            w.writerow([ts, f"{ts / 1000.0:.4f}",
                        s, STATE_NAMES.get(s, f"0x{s:X}")])


def write_pid(rows, out):
    # 74-byte SYS_STATUS payload: 0x05, count_u8, 18 floats.
    # The 18 floats are the control_telemetry_t struct from variables.h.
    field_names = [
        "roll_angle_sp", "pitch_angle_sp", "yaw_angle_sp",
        "roll_angle_curr", "pitch_angle_curr", "yaw_angle_curr",
        "roll_rate_sp", "pitch_rate_sp", "yaw_rate_sp",
        "roll_rate_curr", "pitch_rate_curr", "yaw_rate_curr",
        "roll_out", "pitch_out", "yaw_out",
        "thro_out", "outer_dt", "inner_dt",
    ]
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "t_s"] + field_names)
        for _, ts, payload in rows:
            if len(payload) != 74 or payload[0] != ORIGIN_PID_ERROR:
                continue
            vals = struct.unpack("<18f", payload[2:])
            w.writerow([ts, f"{ts / 1000.0:.4f}"] +
                       [f"{v:.6f}" for v in vals])


def write_log(rows, out):
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "t_s", "text"])
        for _, ts, payload in rows:
            try:
                txt = payload.decode("utf-8", errors="replace")
            except Exception:
                txt = repr(payload)
            txt = txt.rstrip("\x00\r\n")
            w.writerow([ts, f"{ts / 1000.0:.4f}", txt])


def write_unknown_status(rows, out):
    if not rows:
        return
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "t_s", "origin", "payload_hex"])
        for _, ts, payload in rows:
            origin = payload[0] if payload else 0
            w.writerow([ts, f"{ts / 1000.0:.4f}",
                        f"0x{origin:02X}", payload.hex()])


# --- main ------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="Path to a sim-*.bin log file")
    ap.add_argument("--out-dir",
                    help="Output directory (default: next to log)")
    args = ap.parse_args(argv)

    raw = open(args.log, "rb").read()
    print(f"input: {args.log} ({len(raw)} bytes)")

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.log))
    stem = os.path.splitext(os.path.basename(args.log))[0]
    target = os.path.join(out_dir, stem + "_csv")
    os.makedirs(target, exist_ok=True)
    print(f"output: {target}/")

    # Bucket frames by type.
    by_type = defaultdict(list)
    status_rows_state = []
    status_rows_pid   = []
    status_rows_other = []
    for idx, ptype, ts, _dev, payload in parse_frames(raw):
        if ptype == PACKET_SYSTEM_STATUS:
            if payload and payload[0] == ORIGIN_SYS_STATE:
                status_rows_state.append((idx, ts, payload))
            elif payload and payload[0] == ORIGIN_PID_ERROR:
                status_rows_pid.append((idx, ts, payload))
            else:
                status_rows_other.append((idx, ts, payload))
        else:
            by_type[ptype].append((idx, ts, payload))

    total = sum(len(v) for v in by_type.values()) + len(status_rows_state) \
            + len(status_rows_pid) + len(status_rows_other)
    print(f"parsed: {total} frames")

    # Write per-stream CSVs.
    write_heartbeat(by_type[PACKET_HEARTBEAT],
                    os.path.join(target, "heartbeat.csv"))
    write_imu(by_type[PACKET_IMU_DATA_FULL],
              by_type[PACKET_IMU_COMPRESSED],
              os.path.join(target, "imu.csv"))
    write_attitude(by_type[PACKET_ATTITUDE],
                   os.path.join(target, "attitude.csv"))
    write_rc(by_type[PACKET_RC_CHANNELS],
             os.path.join(target, "rc.csv"))
    write_motor(by_type[PACKET_MOTOR_TELEMETRY],
                os.path.join(target, "motor.csv"))
    write_log(by_type[PACKET_LOG],
              os.path.join(target, "log.csv"))
    write_state(status_rows_state,
                os.path.join(target, "state.csv"))
    write_pid(status_rows_pid,
              os.path.join(target, "pid.csv"))
    write_unknown_status(status_rows_other,
                         os.path.join(target, "unknown_status.csv"))

    # One-line summary per stream.
    summary = [
        ("heartbeat",    by_type[PACKET_HEARTBEAT]),
        ("imu_full",     by_type[PACKET_IMU_DATA_FULL]),
        ("imu_comp",     by_type[PACKET_IMU_COMPRESSED]),
        ("attitude",     by_type[PACKET_ATTITUDE]),
        ("rc",           by_type[PACKET_RC_CHANNELS]),
        ("motor",        by_type[PACKET_MOTOR_TELEMETRY]),
        ("log",          by_type[PACKET_LOG]),
        ("sys_state",    status_rows_state),
        ("sys_pid",      status_rows_pid),
        ("sys_other",    status_rows_other),
    ]
    print()
    for name, rows in summary:
        print(f"  {name:12s}  {len(rows):6d}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
