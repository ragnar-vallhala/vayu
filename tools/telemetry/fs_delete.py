#!/usr/bin/env python3
"""Delete one file from the FC's SD card over NavLink (FS_DELETE, msgid 8207).

The FC answers with COMMAND_ACK alone -- there is nothing to report but the
result -- and the result code is the whole story:

    ACCEPTED              the file is gone
    DENIED                absent, a directory, or protected (cal.bin/pid.bin)
    TEMPORARILY_REJECTED  in use right now (imuhs.bin while recording); retry
    FAILED                allowed, but the unlink errored

Reuses the transfer tool's bridge discovery and TIME_SYNC, because FS_DELETE is
in the command range and the FC's spec-10.5 gate rejects commands until the
clock is disciplined.

    python3 tools/telemetry/fs_delete.py 0:xtest.bin
    python3 tools/telemetry/fs_delete.py --list            # just show the card
"""
import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import fs_xfer_udp_test as X  # Bridge, discover_and_sync, FC_SYS/FC_COMP, nl

nl = X.nl

RESULT = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
          3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}

# Why a refusal happened, so the operator is not left guessing at a bare code.
WHY = {
    1: "the file is in use -- imuhs.bin is refused while a session is "
       "recording; disarm and retry",
    2: "absent, a directory, or protected -- cal.bin and pid.bin are refused "
       "permanently, losing either costs a recalibration or a retune",
    4: "the unlink itself failed (card error?)",
}


def delete(br, path, timeout=6.0):
    print(f"[del] FS_DELETE '{path}'")
    t0 = time.monotonic()
    sent = 0.0
    while time.monotonic() - t0 < timeout:
        t = time.monotonic()
        if t - sent >= 0.5:
            br.send(nl.FsDelete(target_sys=X.FC_SYS, target_comp=X.FC_COMP,
                                req_seq=7, path=path))
            sent = t
        for d in br.poll():
            if d.msgid != nl.CommandAck.MSGID:
                continue
            a = nl.CommandAck.unpack(d.payload)
            if getattr(a, "acked_msgid", None) not in (None, nl.FsDelete.MSGID):
                continue  # an ack for some other command
            res = a.result
            print(f"[del] -> {RESULT.get(res, res)}")
            if res in WHY:
                print(f"       {WHY[res]}")
            return res
        time.sleep(0.005)
    print("[del] -> (no COMMAND_ACK within timeout)")
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?", help="SD path to delete, e.g. 0:xtest.bin")
    ap.add_argument("--port", type=int, default=14555)
    ap.add_argument("--list", action="store_true",
                    help="list the card instead of deleting")
    a = ap.parse_args()
    if not a.path and not a.list:
        ap.error("give a path to delete, or --list")

    br = X.Bridge(a.port)
    X.discover_and_sync(br)
    if a.list:
        X.fs_probe(br, a.path or "0:")
        return 0
    res = delete(br, a.path)
    X.fs_probe(br, a.path)
    return 0 if res == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
