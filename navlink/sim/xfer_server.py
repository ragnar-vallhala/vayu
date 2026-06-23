"""FC-side responder for the NavLink bulk-transfer (FTP) substrate.

The Python mirror of the firmware xfer state machine (src/comm/xfer/navlink_xfer.c)
— enough of it to answer a real XferClient end-to-end over real frames. Used by
the loopback conformance test (independent C-driven client semantics vs an
independent Python server) and as the reroute target for endpoint.py's FC role.

Pure protocol logic, transport-agnostic: feed it decoded inbound frames via
feed(decoded), call tick() to emit, and supply a `send(frame_bytes)` callable.
Reliability mirrors the firmware: two-phase open (COMMAND_ACK + XFER_INFO),
cumulative-ack + resume-by-rewind, contiguous-only upload ingest.

`files` maps an arg name -> bytes: a download source, or an upload sink (created
/ overwritten on upload-open; readable afterwards for verification).
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field

sys.path.insert(0, os.path.dirname(__file__))  # flat sim layout (no __init__.py)
import frame                       # noqa: E402
from common import load_nl         # noqa: E402

nl = load_nl()

DIR_DOWNLOAD = 0
DIR_UPLOAD = 1
FC_SYS = 1
FC_COMP = 1
CHUNK = 247

_F = nl.XferFlags
_R = nl.CommandResult


@dataclass
class _Session:
    session: int
    direction: int
    arg: str
    req_seq: int
    total: int = 0
    cursor: int = 0          # download: next offset to emit; upload: next expected
    src: bytes = b""         # download payload
    sink: bytearray = field(default_factory=bytearray)  # upload accumulator
    opened: bool = False
    done: bool = False
    eof_seen: bool = False


class XferServer:
    def __init__(self, send, files=None):
        self._send = send
        self._seq = 0
        self.files = dict(files or {})
        self.sessions: dict[int, _Session] = {}
        self._pending_open: list[_Session] = []

    # -- plumbing ----------------------------------------------------------
    def _emit(self, msg):
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        self._send(frame.encode(msg.MSGID, msg.pack(), seq=seq,
                                sysid=FC_SYS, compid=FC_COMP))

    def _command_ack(self, msgid, req_seq, result):
        self._emit(nl.CommandAck(command=msgid, req_seq=req_seq, result=int(result),
                                 progress=100, result_param2=0))

    # -- inbound -----------------------------------------------------------
    def feed(self, decoded):
        if not decoded.ok:
            return
        mid = decoded.msgid
        if mid == nl.XferOpen.MSGID:
            self._on_open(nl.XferOpen.unpack(decoded.payload))
        elif mid == nl.XferClose.MSGID:
            self._on_close(nl.XferClose.unpack(decoded.payload))
        elif mid == nl.XferData.MSGID:
            self._on_data(nl.XferData.unpack(decoded.payload))
        elif mid == nl.XferAck.MSGID:
            self._on_ack(nl.XferAck.unpack(decoded.payload))

    def _on_open(self, m):
        arg = m.arg if isinstance(m.arg, str) else bytes(m.arg).split(b"\0")[0].decode()
        s = _Session(session=m.session, direction=m.dir, arg=arg, req_seq=m.req_seq,
                     cursor=m.offset_start)
        if m.dir == DIR_DOWNLOAD:
            if arg not in self.files:
                self.sessions[m.session] = s
                self._command_ack(nl.XferOpen.MSGID, m.req_seq, _R.FAILED)
                return
            s.src = self.files[arg]
            s.total = len(s.src)
        else:  # upload: truncate/create the sink
            self.files[arg] = b""
            s.sink = bytearray()
            s.total = 0xFFFFFFFF
        self.sessions[m.session] = s
        self._pending_open.append(s)  # deferred ACK+INFO on next tick (like the FC)

    def _on_close(self, m):
        self.sessions.pop(m.session, None)
        self._command_ack(nl.XferClose.MSGID, m.req_seq, _R.ACCEPTED)

    def _on_data(self, m):
        s = self.sessions.get(m.session)
        if not s or s.direction != DIR_UPLOAD:
            return
        chunk = bytes(m.data[:m.len])
        if m.offset == s.cursor:                       # contiguous-only
            s.sink.extend(chunk)
            s.cursor += m.len
            self.files[s.arg] = bytes(s.sink)
        if m.flags & int(_F.EOF):
            s.eof_seen = True
            if m.offset + m.len <= s.cursor:
                self.files[s.arg] = bytes(s.sink)
                s.done = True
                self._emit(nl.XferAck(session=s.session, flags=int(_F.DONE),
                                      result=int(_R.ACCEPTED), next_offset=s.cursor))

    def _on_ack(self, m):
        s = self.sessions.get(m.session)
        if not s or s.direction != DIR_DOWNLOAD:
            return
        s.opened = True
        if m.flags & int(_F.DONE):
            self.sessions.pop(m.session, None)
            return
        if m.next_offset < s.cursor or (m.flags & int(_F.NAK)):
            s.cursor = m.next_offset                   # resume-by-rewind
            s.done = False                             # reactivate to refill the gap

    # -- tick (emit) -------------------------------------------------------
    def tick(self, budget=8):
        for s in list(self._pending_open):
            s.opened = True
            self._command_ack(nl.XferOpen.MSGID, s.req_seq, _R.ACCEPTED)
            self._emit(nl.XferInfo(session=s.session, result=int(_R.ACCEPTED),
                                   chunk_size=CHUNK, total_size=s.total, mtime=0))
        self._pending_open.clear()

        emitted = 0
        for s in list(self.sessions.values()):
            if s.direction == DIR_UPLOAD:
                self._emit(nl.XferAck(session=s.session, flags=int(_F.NONE),
                                      result=int(_R.ACCEPTED), next_offset=s.cursor))
                continue
            while budget > 0 and not s.done and s.cursor < s.total:
                end = min(s.cursor + CHUNK, s.total)
                chunk = s.src[s.cursor:end]
                last = end >= s.total
                flags = int(_F.EOF) if last else int(_F.NONE)
                payload = list(chunk) + [0] * (CHUNK - len(chunk))
                self._emit(nl.XferData(session=s.session, flags=flags,
                                       len=len(chunk), offset=s.cursor, data=payload))
                s.cursor = end
                budget -= 1
                emitted += 1
                if last:
                    s.done = True
        return emitted

    def busy(self):
        return bool(self._pending_open) or any(
            not s.done for s in self.sessions.values())
