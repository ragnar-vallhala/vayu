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
"""Re-apply a saved sysid tune (gains + motor geometry) to the FC over NavLink/UDP.

The FC does not persist gains/geometry across battery cycles, so after each boot
run this to push the working tune from tools/sysid/sysid_tune.json (or another file).
It sends CMD_SET_MOTOR_GEOMETRY first (fixes the yaw sign), then CMD_SET_PID for
every rate/angle axis present in the file.

    python3 tools/sysid/apply_tune.py                       # uses tools/sysid/sysid_tune.json
    python3 tools/sysid/apply_tune.py --file my_tune.json
    python3 tools/sysid/apply_tune.py --dry-run             # print, don't send

DISARM first. Close the Navigator GCS (UDP 14555 is single-owner).
"""
import argparse
import json
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "navlink", "sim"))
sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))
import frame                       # noqa: E402
import navlink_msgs as nl          # noqa: E402

AXES = {"roll": 0, "pitch": 1, "yaw": 2}

# The FC's §10.5 gate rejects every command until its clock is disciplined, and
# ONLY role=2 does that (it forces _clock_synced via set_offset64(0)); role=0 is
# an ordinary sample and leaves the gate shut. Sending commands into a shut gate
# is silent — they are dropped with no ack and no log, which is indistinguishable
# from a lossy link. Verified 2026-09-05: a run that reported "sent 5 commands"
# left every gain at its compiled default (read back over SWD).
TIME_SYNC_REQUEST_WIDE = 2


def _frames(dgram):
    """Yield every NavLink frame in a datagram.

    The ESP bridge packs several frames per UDP datagram, so decoding only the
    first silently loses the rest — which is why acks appeared to be missing."""
    off = 0
    while len(dgram) - off >= frame.HDR_LEN + 2:
        if dgram[off] != 0x56 or dgram[off + 1] != 0x02:
            off += 1
            continue
        flen = frame.HDR_LEN + dgram[off + 2] + 2
        if len(dgram) - off < flen:
            break
        d = frame.decode(dgram[off:off + flen])
        off += flen
        if d.ok:
            yield d


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", default=os.path.join(HERE, "sysid_tune.json"))
    ap.add_argument("--port", type=int, default=14555)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    with open(args.file) as f:
        tune = json.load(f)

    # Build the command list (geometry first, then rate, then angle).
    cmds = []
    g = tune.get("motor_geometry")
    if g:
        cmds.append(("geometry", g))
    for ctrl_name, ctrl_id in (("rate_pid", 1), ("angle_pid", 0)):
        block = tune.get(ctrl_name, {})
        for ax_name, gains in block.items():
            if ax_name in AXES and isinstance(gains, dict):
                cmds.append((f"{ctrl_name}.{ax_name}", (ctrl_id, AXES[ax_name], gains)))
    # Optional D-term LPF block: { "roll": 0.004, "pitch": 0.004 } (rc seconds).
    # The compiled DEAFULT_*_RATE_D_LPF_RC kills the derivative path unless set;
    # CMD_SET_D_LPF makes it tunable live like the gains.
    for ax_name, rc in tune.get("d_lpf", {}).items():
        if ax_name in AXES:
            cmds.append((f"d_lpf.{ax_name}", ("dlpf", AXES[ax_name], float(rc))))

    print(f"[tune] {args.file}  ({tune.get('captured','?')}, {tune.get('airframe','')[:40]})")
    for name, payload in cmds:
        if name == "geometry":
            print(f"  GEOMETRY  spin={payload['spin']}")
        elif name.startswith("d_lpf."):
            _, ax, rc = payload
            print(f"  D-LPF {list(AXES)[ax]:5s}  rc={rc} s")
        else:
            ctrl, ax, gg = payload
            print(f"  {'RATE ' if ctrl==1 else 'ANGLE'} {list(AXES)[ax]:5s}  "
                  f"kp={gg.get('kp',0)} ki={gg.get('ki',0)} kd={gg.get('kd',0)}  "
                  f"[{gg.get('status','')}]")
    if args.dry_run:
        return 0

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        s.bind(("0.0.0.0", args.port))
    except OSError as e:
        print(f"[tune] cannot bind :{args.port} ({e}). Close Navigator GCS first.")
        return 2
    s.settimeout(0.1)
    peer = None
    seq = 0
    lh = 0.0
    t0 = time.time()
    while not peer and time.time() - t0 < 5:
        if time.time() - lh > 0.4:
            s.sendto(b"GCS-HELLO", ("255.255.255.255", args.port)); lh = time.time()
        try:
            d, a = s.recvfrom(2048)
            if d != b"GCS-HELLO":
                peer = a
        except socket.timeout:
            pass
    if not peer:
        print("[tune] no FC seen"); return 1

    def send(mid, pl):
        nonlocal seq
        seq = (seq + 1) & 0xFF
        s.sendto(frame.encode(mid, pl, seq=seq), peer)

    # Clear the §10.5 gate and CONFIRM it: retransmit until the FC answers with
    # role=1. Without the confirmation a single dropped request (this link runs
    # 10-15% loss) leaves the gate shut and every command below is discarded.
    synced = False
    t0 = time.time()
    last_req = 0.0
    while not synced and time.time() - t0 < 10.0:
        # Keep announcing ourselves: the ESP bridge unicasts to the last peer it
        # heard a GCS-HELLO from, and goes quiet if we stop.
        if time.time() - lh > 0.4:
            s.sendto(b"GCS-HELLO", ("255.255.255.255", args.port))
            lh = time.time()
        if time.time() - last_req > 0.3:
            send(nl.TimeSync.MSGID,
                 nl.TimeSync(role=TIME_SYNC_REQUEST_WIDE, seq=seq & 0xFF,
                             t1_gcs_tx=int(time.time() * 1e6)).pack())
            last_req = time.time()
        try:
            d, _ = s.recvfrom(2048)
        except socket.timeout:
            continue
        if d == b"GCS-HELLO":
            continue
        for fr in _frames(d):
            if fr.msgid == nl.TimeSync.MSGID:
                m = nl.TimeSync.unpack(fr.payload)
                if int(getattr(m, "role", 0)) == 1:
                    synced = True
    if not synced:
        print("[tune] FAILED: the FC never confirmed TIME_SYNC, so the command "
              "gate is shut and NOTHING would be applied. Not sending.",
              file=sys.stderr)
        s.close()
        return 2
    print("[tune] §10.5 gate cleared (FC clock disciplined).")

    sent = {}  # req_seq -> label
    for name, payload in cmds:
        # send() increments seq before it transmits, so the value placed in the
        # payload's req_seq (echoed back in CommandAck) is seq+1. Key sent[] by
        # that same value, not the pre-increment seq, or the labels shift by one
        # slot and the last command can never be matched to its ack.
        req = (seq + 1) & 0xFF
        if name == "geometry":
            m = nl.CmdSetMotorGeometry(target_sys=42, target_comp=1, req_seq=req,
                                       layout=0, pos_x=payload["pos_x"],
                                       pos_y=payload["pos_y"], spin=payload["spin"])
        elif name.startswith("d_lpf."):
            _, ax, rc = payload
            m = nl.CmdSetDLpf(target_sys=42, target_comp=1, req_seq=req,
                              axis=ax, rc=rc)
        else:
            ctrl, ax, gg = payload
            m = nl.CmdSetPid(target_sys=42, target_comp=1, req_seq=req, controller=ctrl,
                             axis=ax, kp=gg.get("kp", 0.0), ki=gg.get("ki", 0.0),
                             kd=gg.get("kd", 0.0), kff=gg.get("kff", 0.0))
        send(m.MSGID, m.pack())
        sent[req] = name
        time.sleep(0.12)

    # Collect acks.
    acked = {}
    t0 = time.time()
    while time.time() - t0 < 3:
        if time.time() - lh > 0.4:
            s.sendto(b"GCS-HELLO", ("255.255.255.255", args.port)); lh = time.time()
        try:
            d, a = s.recvfrom(2048)
        except socket.timeout:
            continue
        if d == b"GCS-HELLO":
            continue
        for fr in _frames(d):
            if fr.msgid == 5:
                mm = nl.CommandAck.unpack(fr.payload)
                if mm.req_seq in sent:
                    acked[sent[mm.req_seq]] = mm.result
    s.close()

    print(f"[tune] sent {len(cmds)} commands; acks: "
          f"{ {k: ('OK' if v == 0 else v) for k, v in acked.items()} }")
    missing = [n for _, (n) in [(k, sent[k]) for k in sent] if n not in acked]
    if missing:
        print(f"[tune] no ack seen for {missing} (lossy link; likely applied — re-run "
              f"to confirm).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
