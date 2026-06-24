#!/usr/bin/env python3
"""Loopback for filesystem navigation (FS_LIST / FS_INFO) over real frames.

An independent Python FC responder vs a Python client, exercising the wire two
impls must agree on: a directory listing (entries + terminal count), a stat, and
the "path does not exist" (DENIED) case in both modes. Standalone; run_tests.py
calls it as a step.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "generated", "python"))

import frame                       # noqa: E402
import navlink_msgs as nl          # noqa: E402

DEV = 1
_R = nl.CommandResult
checks = 0
fails = 0


def check(cond, msg):
    global checks, fails
    checks += 1
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails += 1


class FsServer:
    """Models a flat root '0:' with a few files."""
    def __init__(self, send, files):
        self._send = send
        self._seq = 0
        self.files = dict(files)  # name -> size

    def _emit(self, msg):
        self._send(frame.encode(msg.MSGID, msg.pack(), seq=self._seq,
                                sysid=DEV, compid=1))
        self._seq = (self._seq + 1) & 0xFF

    def feed(self, d):
        if not d.ok:
            return
        if d.msgid == nl.FsList.MSGID:
            self._on_list(nl.FsList.unpack(d.payload))
        elif d.msgid == nl.FsInfo.MSGID:
            self._on_info(nl.FsInfo.unpack(d.payload))

    def _ack(self, msgid, req):
        self._emit(nl.CommandAck(command=msgid, req_seq=req,
                                 result=int(_R.ACCEPTED), progress=100,
                                 result_param2=0))

    def _on_list(self, m):
        self._ack(nl.FsList.MSGID, m.req_seq)
        path = m.path if isinstance(m.path, str) else \
            bytes(m.path).split(b"\0")[0].decode()
        if path not in ("0:", "0:/", ""):
            self._emit(nl.FsEntry(req_seq=m.req_seq, result=int(_R.DENIED),
                                  index=0, count=0, type=0, size=0, name=""))
            return
        names = sorted(self.files)
        for i, name in enumerate(names):
            self._emit(nl.FsEntry(req_seq=m.req_seq, result=int(_R.ACCEPTED),
                                  index=i, count=0, type=0,
                                  size=self.files[name], name=name))
        # terminal entry: empty name, total count
        self._emit(nl.FsEntry(req_seq=m.req_seq, result=int(_R.ACCEPTED),
                              index=len(names), count=len(names), type=0,
                              size=0, name=""))

    def _on_info(self, m):
        self._ack(nl.FsInfo.MSGID, m.req_seq)
        path = m.path if isinstance(m.path, str) else \
            bytes(m.path).split(b"\0")[0].decode()
        name = path[2:] if path.startswith("0:") else path
        if name in self.files:
            self._emit(nl.FsInfoReply(req_seq=m.req_seq, result=int(_R.ACCEPTED),
                                      type=0, size=self.files[name], mtime=0))
        else:
            self._emit(nl.FsInfoReply(req_seq=m.req_seq, result=int(_R.DENIED),
                                      type=0, size=0, mtime=0))


class FsClient:
    def __init__(self, send):
        self._send = send
        self._seq = 0
        self.entries = []
        self.list_done = False
        self.list_total = None
        self.info_reply = None

    def _emit(self, msgid, payload):
        self._send(frame.encode(msgid, payload, seq=self._seq, sysid=DEV,
                                compid=1))
        self._seq = (self._seq + 1) & 0xFF

    def list(self, path):
        self.entries = []
        self.list_done = False
        self._emit(nl.FsList.MSGID,
                   nl.FsList(target_sys=DEV, target_comp=1, req_seq=1,
                             start_index=0, path=path).pack())

    def info(self, path):
        self.info_reply = None
        self._emit(nl.FsInfo.MSGID,
                   nl.FsInfo(target_sys=DEV, target_comp=1, req_seq=2,
                             path=path).pack())

    def feed(self, d):
        if not d.ok:
            return
        if d.msgid == nl.FsEntry.MSGID:
            e = nl.FsEntry.unpack(d.payload)
            nm = e.name if isinstance(e.name, str) else \
                bytes(e.name).split(b"\0")[0].decode()
            if e.result == int(_R.DENIED):
                self.list_done = True
                self.list_total = -1
            elif nm == "":
                self.list_done = True
                self.list_total = e.count
            else:
                self.entries.append((nm, e.size))
        elif d.msgid == nl.FsInfoReply.MSGID:
            self.info_reply = nl.FsInfoReply.unpack(d.payload)


def make_pair(files):
    c2s, s2c = [], []
    cli = FsClient(send=c2s.append)
    srv = FsServer(send=s2c.append, files=files)

    def pump():
        for fr in list(c2s):
            srv.feed(frame.decode(fr))
        c2s.clear()
        for fr in list(s2c):
            cli.feed(frame.decode(fr))
        s2c.clear()
    return cli, srv, pump


def main():
    print("== fs-nav loopback (Python client <-> server, real frames) ==")
    files = {"PID.BIN": 140, "CAL.BIN": 92, "LOG.BIN": 4096}
    cli, srv, pump = make_pair(files)

    cli.list("0:")
    for _ in range(10):
        pump()
        if cli.list_done:
            break
    got = dict(cli.entries)
    check(cli.list_done and got == files, "list: entries match the directory")
    check(cli.list_total == len(files), "list: terminal count is the total")

    cli.list("0:bogus")
    for _ in range(10):
        pump()
        if cli.list_done:
            break
    check(cli.list_total == -1, "list of a missing path -> DENIED")

    cli.info("0:PID.BIN")
    pump()
    pump()
    check(cli.info_reply is not None and cli.info_reply.result == int(_R.ACCEPTED)
          and cli.info_reply.size == 140, "info: existing file size + ACCEPTED")

    cli.info("0:nope.bin")
    pump()
    pump()
    check(cli.info_reply is not None and cli.info_reply.result == int(_R.DENIED),
          "info of a missing path -> DENIED")

    print(f"\n{checks} checks, {fails} failures")
    sys.exit(0 if fails == 0 else 1)


if __name__ == "__main__":
    main()
