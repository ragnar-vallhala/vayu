#!/usr/bin/env python3
"""Headless gyro (and any IMU) calibration harness for the REAL FC over UDP.

Speaks the same NavLink v2 wire as the GCS: binds the GCS telemetry port,
broadcasts GCS-HELLO so the ESP8266 bridge unicasts to us, sends a TIME_SYNC
(mirrors the GCS handshake) then CMD_CALIBRATE_IMU, and live-prints everything
that matters to debug a stuck calibration:

  * Statustext  -> the firmware's [CALIB] vayu_log lines
  * CalibrationStatus -> step / progress% / coverage, incl. COMPLETE/FAILED
  * ImuRaw      -> live gyro (deg/s) and accel, so we can SEE what the pose gate
                   is being fed while it decides whether to advance

Requires the Navigator GCS to be CLOSED first (only one process can own the
UDP port / the bridge's unicast target).

    python3 tools/calib/gyro_calib_harness.py                 # gyro bias (which=0x20)
    python3 tools/calib/gyro_calib_harness.py --which 0x21    # gyro full
    python3 tools/calib/gyro_calib_harness.py --which 0x10    # accel bias
    python3 tools/calib/gyro_calib_harness.py --monitor-only  # no command, just watch
"""
import argparse
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "navlink", "sim"))
sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))

import frame                       # noqa: E402  navlink/sim/frame.py
import navlink_msgs as nl          # noqa: E402  generated codec

MSG_BY_ID = {c.MSGID: c for c in vars(nl).values()
             if isinstance(c, type) and hasattr(c, "MSGID") and hasattr(c, "unpack")}

STEP_NAME = {0: "PROGRESS", 1: "NOSE_UP", 2: "NOSE_DOWN", 3: "RIGHT_DOWN",
             4: "LEFT_DOWN", 5: "UPRIGHT", 6: "UPSIDE_DOWN", 7: "FREE_ROT",
             8: "MAG_AXIS_COVERAGE", 9: "COMPLETE", 10: "FAILED"}


def decode_text(raw):
    if isinstance(raw, (bytes, bytearray)):
        return raw.split(b"\x00")[0].decode("ascii", "replace")
    return str(raw)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=14555)
    ap.add_argument("--which", default="0x20",
                    help="calibration selector byte: hi nibble sensor (1=accel,"
                         " 2=gyro, 3=mag), lo nibble mode (0=bias, 1=full). "
                         "default 0x20 = gyro bias")
    ap.add_argument("--seconds", type=float, default=60.0,
                    help="give up after N s with no COMPLETE/FAILED (default 60)")
    ap.add_argument("--monitor-only", action="store_true",
                    help="don't send a calibrate command, just stream telemetry")
    args = ap.parse_args()
    which = int(args.which, 0)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        sock.bind(("0.0.0.0", args.port))
    except OSError as e:
        print(f"[harness] cannot bind :{args.port} ({e}). "
              f"Close the Navigator GCS first.")
        return 2
    sock.settimeout(0.1)
    print(f"[harness] bound :{args.port}  which=0x{which:02x} "
          f"({'monitor-only' if args.monitor_only else 'will start calibration'})")

    peer = None
    seq = 0
    sent_cmd = False
    last_hello = 0.0
    last_gyro_print = 0.0
    last_gyr = last_acc = None
    last_nav = None
    t0 = time.monotonic()
    result = None

    def send(msgid, payload):
        nonlocal seq
        seq = (seq + 1) & 0xFF
        sock.sendto(frame.encode(msgid, payload, seq=seq), peer)

    while True:
        now = time.monotonic()
        if now - t0 > args.seconds:
            print(f"[harness] TIMEOUT after {args.seconds:.0f}s, no terminal status.")
            break

        # Broadcast discovery so the bridge keeps unicasting to us.
        if now - last_hello >= 0.5:
            sock.sendto(b"GCS-HELLO", ("255.255.255.255", args.port))
            last_hello = now

        # Once we know the FC's address: TIME_SYNC then the calibrate command.
        if peer and not sent_cmd and not args.monitor_only:
            ts = nl.TimeSync(role=0, seq=1, t1_gcs_tx=int(time.time() * 1e6))
            send(ts.MSGID, ts.pack())
            time.sleep(0.05)
            cmd = nl.CmdCalibrateImu(target_sys=42, target_comp=1,
                                     req_seq=seq, which=which)
            send(cmd.MSGID, cmd.pack())
            print(f"[harness] -> CMD_CALIBRATE_IMU which=0x{which:02x} "
                  f"to {peer[0]}:{peer[1]}")
            sent_cmd = True

        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        if data == b"GCS-HELLO":
            continue
        if peer != addr:
            peer = addr
            print(f"[harness] peer -> {addr[0]}:{addr[1]}")
        d = frame.decode(data)
        if not d.ok:
            continue
        cls = MSG_BY_ID.get(d.msgid)
        if not cls:
            continue
        try:
            m = cls.unpack(d.payload)
        except Exception:
            continue

        nm = cls.__name__
        if nm == "Heartbeat":
            ns = getattr(m, "nav_state", None)
            if ns != last_nav:
                NAV = {0: "UNINIT", 1: "INIT", 2: "STANDBY", 3: "PREARM",
                       4: "ARMED", 5: "IN_AIR", 6: "FAILSAFE", 7: "TERMINATED",
                       8: "CALIBRATING"}
                print(f"[{now - t0:6.2f}s] NAV_STATE -> {NAV.get(ns, ns)}")
                last_nav = ns
        elif nm == "Statustext":
            txt = decode_text(getattr(m, "text", b""))
            print(f"[{now - t0:6.2f}s] LOG: {txt}")
            if "Gyro Bias" in txt:
                result = ("GYRO BIAS COMPUTED", txt)
        elif nm == "CommandAck":
            if getattr(m, "command", 0) == nl.CmdCalibrateImu.MSGID:
                res = getattr(m, "result", "?")
                print(f"[{now - t0:6.2f}s] ACK calibrate: result={res} "
                      f"(0=ACCEPTED)")
        elif nm == "CalibrationStatus":
            step = getattr(m, "step", 0)
            prog = getattr(m, "progress", 0)
            cov = getattr(m, "coverage", [0, 0, 0])
            sname = STEP_NAME.get(step, f"step{step}")
            extra = ""
            if step == 8:  # MAG_AXIS_COVERAGE: bar = mean of the three
                extra = f"  mean={sum(cov) / 3.0:5.1f}%"
            print(f"[{now - t0:6.2f}s] CALIB step={sname:11s} progress={prog:3d}% "
                  f"cov={['%.1f' % c for c in cov]}{extra}")
            if step == 9:
                result = ("COMPLETE", None)
            elif step == 10:
                result = ("FAILED", None)
        elif nm == "ImuRaw":
            last_gyr = getattr(m, "gyr", None)
            last_acc = getattr(m, "acc", None)
            if now - last_gyro_print >= 1.0:
                last_gyro_print = now
                if last_gyr and last_acc:
                    print(f"[{now - t0:6.2f}s] IMU gyr(dps)="
                          f"[{last_gyr[0]:+7.2f} {last_gyr[1]:+7.2f} {last_gyr[2]:+7.2f}]"
                          f"  acc(m/s2)=[{last_acc[0]:+6.2f} {last_acc[1]:+6.2f} "
                          f"{last_acc[2]:+6.2f}]")

        if result:
            tag, extra = result
            print(f"\n[harness] === {tag} === "
                  + (extra if extra else ""))
            break

    sock.close()
    return 0 if (result and result[0] in ("COMPLETE", "GYRO BIAS COMPUTED")) else 1


if __name__ == "__main__":
    sys.exit(main())
