#!/usr/bin/env python3
"""End-to-end loopback for the xfer (FTP) substrate over REAL frames.

Wires the Python XferClient (GCS) against the Python XferServer (FC) through the
actual frame codec — two independent implementations of the same wire contract —
and checks a file round-trips in both directions, including a lossy case that
exercises resume-by-rewind. Complements the C SM/provider tests and the C<->Py
codec parity: this is the protocol-level interop check.

Standalone (prints PASS/FAIL, exits non-zero on failure) so run_tests.py can call
it.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "sim"))

import frame                       # noqa: E402
from xfer_client import XferClient  # noqa: E402
from xfer_server import XferServer  # noqa: E402

SVC_FILE = 0
checks = 0
fails = 0


def check(cond, msg):
    global checks, fails
    checks += 1
    if cond:
        print(f"  ok   {msg}")
    else:
        fails += 1
        print(f"  FAIL {msg}")


def make_pair(files, drop_nth_server_data=None):
    """Return (client, server, pump). pump() shuttles frames once each way; an
    optional drop_nth_server_data skips that XFER_DATA frame (1-based) once."""
    c2s, s2c = [], []
    state = {"data_seen": 0, "dropped": False}

    def to_server(fr):
        c2s.append(fr)

    def to_client(fr):
        d = frame.decode(fr)
        if (drop_nth_server_data is not None and d.ok
                and d.msgid == __import__("xfer_client").nl.XferData.MSGID):
            state["data_seen"] += 1
            if state["data_seen"] == drop_nth_server_data and not state["dropped"]:
                state["dropped"] = True
                return  # simulate a lost downlink frame
        s2c.append(fr)

    client = XferClient(send=to_server)
    server = XferServer(send=to_client, files=files)

    def pump():
        for fr in list(c2s):
            server.feed(frame.decode(fr))
        c2s.clear()
        server.tick()
        for fr in list(s2c):
            client.feed(frame.decode(fr))
        s2c.clear()

    return client, server, pump


def test_download(lossy):
    label = "download (lossy resume)" if lossy else "download"
    payload = bytes((i * 37 + 9) & 0xFF for i in range(1500))
    client, server, pump = make_pair(
        {"f.bin": payload}, drop_nth_server_data=2 if lossy else None)
    s = client.open_download(0, SVC_FILE, "f.bin")
    for _ in range(200):
        pump()
        if s.done:
            break
    check(s.done, f"{label}: session completed")
    check(client.result_bytes(0) == payload, f"{label}: bytes match the source")


def test_upload():
    payload = bytes((i * 11 + 3) & 0xFF for i in range(1200))
    client, server, pump = make_pair({})
    s = client.open_upload(1, SVC_FILE, "up.bin", payload)
    for _ in range(200):
        pump()
        if s.done:
            break
    check(s.done, "upload: session completed")
    check(server.files.get("up.bin") == payload, "upload: server received the bytes")


def main():
    print("== xfer loopback (Python client <-> Python server, real frames) ==")
    test_download(lossy=False)
    test_download(lossy=True)
    test_upload()
    print(f"\n{checks} checks, {fails} failures")
    sys.exit(0 if fails == 0 else 1)


if __name__ == "__main__":
    main()
