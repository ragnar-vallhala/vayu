"""GCS-side driver for the NavLink bulk-transfer (FTP) substrate.

Pure protocol logic — no sockets or threads. The caller wires it to a transport
by supplying a `send` callable (frame bytes -> wire) and feeding every decoded
inbound frame to `feed(decoded)`. That keeps the client deterministically
drivable from host tests (against a Python `XferServer` or the C state machine)
*and* reusable over the real UDP/UART link.

Wire/semantics: docs/plans/navlink-xfer-substrate.md. The reliability model is
cumulative-ack + resume-by-rewind (no sliding window):
  - download: the FC streams XFER_DATA by offset; we reassemble sparse-by-offset
    and periodically send XFER_ACK{next_offset = lowest missing byte}. A gap sets
    the NAK flag so the FC rewinds its cursor and refills.
  - upload: we read chunk_size from XFER_INFO, send XFER_DATA from our cursor, and
    advance/retransmit based on the FC's XFER_ACK{next_offset}. The final chunk
    carries EOF.

Direction encoding (matches dialect.json XFER_OPEN.dir): 0 = download (FC->GCS),
1 = upload (GCS->FC).
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
MODE_FILE = 0
MODE_STREAM = 1

GCS_SYS = 1
GCS_COMP = 1
FC_SYS = 1
FC_COMP = 1

_F = nl.XferFlags
_R = nl.CommandResult


@dataclass
class XferSession:
    session: int
    direction: int
    mode: int
    service_id: int
    arg: str
    req_seq: int
    # negotiated in XFER_INFO
    chunk_size: int = 247
    total_size: int = 0xFFFFFFFF
    opened: bool = False
    done: bool = False
    failed: bool = False
    result: int = 0
    # download reassembly
    rx = None                       # bytearray, sized once total_size known
    have: set = field(default_factory=set)   # set of received (offset,len) runs, merged
    cursor: int = 0                 # next contiguous byte (lowest missing)
    # upload progress
    src: bytes = b""
    tx_cursor: int = 0              # next offset to (re)send
    acked: int = 0                  # FC's last next_offset

    def is_stream(self) -> bool:
        return self.total_size == 0xFFFFFFFF


class XferClient:
    """One client can drive several concurrent sessions keyed by session id."""

    def __init__(self, send, *, seq0=0):
        """send: callable(frame_bytes) -> None."""
        self._send = send
        self._seq = seq0 & 0xFF
        self.sessions: dict[int, XferSession] = {}
        # collected stream samples, per session, for stream-mode consumers/tests
        self.stream_rx: dict[int, list[tuple[int, bytes]]] = {}

    # -- frame plumbing ----------------------------------------------------
    def _emit(self, msg):
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        self._send(frame.encode(msg.MSGID, msg.pack(), seq=seq,
                                sysid=GCS_SYS, compid=GCS_COMP))

    def _next_req_seq(self) -> int:
        r = self._seq
        return r & 0xFF

    # -- opening sessions --------------------------------------------------
    def open_download(self, session, service_id, arg, *, offset_start=0,
                      mode=MODE_FILE, rate_hz=0):
        s = XferSession(session=session, direction=DIR_DOWNLOAD, mode=mode,
                        service_id=service_id, arg=arg, req_seq=self._next_req_seq(),
                        cursor=offset_start)
        self.sessions[session] = s
        if mode == MODE_STREAM:
            self.stream_rx.setdefault(session, [])
        self._emit(nl.XferOpen(target_sys=FC_SYS, target_comp=FC_COMP,
                               req_seq=s.req_seq, session=session,
                               dir=DIR_DOWNLOAD, mode=mode, service_id=service_id,
                               offset_start=offset_start, rate_hz=rate_hz, arg=arg))
        return s

    def open_upload(self, session, service_id, arg, data, *, offset_start=0):
        s = XferSession(session=session, direction=DIR_UPLOAD, mode=MODE_FILE,
                        service_id=service_id, arg=arg, req_seq=self._next_req_seq(),
                        src=bytes(data), tx_cursor=offset_start, cursor=offset_start)
        self.sessions[session] = s
        self._emit(nl.XferOpen(target_sys=FC_SYS, target_comp=FC_COMP,
                               req_seq=s.req_seq, session=session,
                               dir=DIR_UPLOAD, mode=MODE_FILE, service_id=service_id,
                               offset_start=offset_start, rate_hz=0, arg=arg))
        return s

    def close(self, session, result=None):
        s = self.sessions.get(session)
        req = self._next_req_seq()
        res = int(_R.ACCEPTED if result is None else result)
        self._emit(nl.XferClose(target_sys=FC_SYS, target_comp=FC_COMP,
                                req_seq=req, session=session, result=res))
        if s:
            s.done = True

    # -- inbound dispatch --------------------------------------------------
    def feed(self, decoded):
        """Feed one decoded inbound frame (sim.frame.Decoded). Ignores anything
        not addressed to a known xfer session."""
        if not decoded.ok:
            return
        mid = decoded.msgid
        if mid == nl.XferInfo.MSGID:
            self._on_info(nl.XferInfo.unpack(decoded.payload))
        elif mid == nl.XferData.MSGID:
            self._on_data(nl.XferData.unpack(decoded.payload))
        elif mid == nl.XferAck.MSGID:
            self._on_ack(nl.XferAck.unpack(decoded.payload))
        elif mid == nl.CommandAck.MSGID:
            pass  # open/close acks are advisory; XFER_INFO carries the real state

    def _on_info(self, info):
        s = self.sessions.get(info.session)
        if not s:
            return
        s.result = info.result
        if info.result != int(_R.ACCEPTED):
            s.failed = True
            s.done = True
            return
        s.opened = True
        s.chunk_size = info.chunk_size or s.chunk_size
        s.total_size = info.total_size
        if s.direction == DIR_DOWNLOAD and not s.is_stream():
            s.rx = bytearray(info.total_size)
        if s.direction == DIR_UPLOAD:
            self._pump_upload(s)

    def _on_data(self, data):
        s = self.sessions.get(data.session)
        if not s or s.direction != DIR_DOWNLOAD:
            return
        chunk = bytes(data.data[:data.len])
        if s.is_stream():
            self.stream_rx[s.session].append((data.offset, chunk))
            s.cursor = data.offset + data.len
        else:
            if s.rx is None:
                s.rx = bytearray(s.total_size)
            end = data.offset + data.len
            if end <= len(s.rx):
                s.rx[data.offset:end] = chunk
                s.have.add((data.offset, data.len))
                self._advance_cursor(s)
        # flow-control / refill ack
        flags = int(_F.NONE)
        nak = (not s.is_stream()) and (data.offset > s.cursor)
        if nak:
            flags |= int(_F.NAK)
        if (data.flags & int(_F.EOF)) and self._download_complete(s):
            self._emit(nl.XferAck(session=s.session, flags=int(_F.DONE),
                                  result=int(_R.ACCEPTED), next_offset=s.cursor))
            s.done = True
            return
        self._emit(nl.XferAck(session=s.session, flags=flags,
                              result=int(_R.ACCEPTED), next_offset=s.cursor))

    def _on_ack(self, ack):
        s = self.sessions.get(ack.session)
        if not s or s.direction != DIR_UPLOAD:
            return
        s.acked = ack.next_offset
        if ack.flags & int(_F.DONE):
            s.done = True
            return
        if ack.flags & int(_F.NAK) or ack.next_offset < s.tx_cursor:
            s.tx_cursor = ack.next_offset   # rewind/resume
        self._pump_upload(s)

    # -- helpers -----------------------------------------------------------
    def _advance_cursor(self, s):
        # cursor = first byte not yet contiguously received
        runs = sorted(s.have)
        for off, ln in runs:
            if off <= s.cursor < off + ln:
                s.cursor = off + ln
            elif off == s.cursor:
                s.cursor = off + ln

    def _download_complete(self, s):
        return (not s.is_stream()) and s.cursor >= s.total_size

    def _pump_upload(self, s, max_chunks=8):
        """Send up to max_chunks XFER_DATA from tx_cursor (caller paces by repeat
        calls / on ack)."""
        sent = 0
        n = len(s.src)
        while s.tx_cursor < n and sent < max_chunks:
            off = s.tx_cursor
            chunk = s.src[off:off + s.chunk_size]
            last = (off + len(chunk)) >= n
            flags = int(_F.EOF) if last else int(_F.NONE)
            payload = list(chunk) + [0] * (247 - len(chunk))
            self._emit(nl.XferData(session=s.session, flags=flags,
                                   len=len(chunk), offset=off, data=payload))
            s.tx_cursor += len(chunk)
            sent += 1

    # -- convenience -------------------------------------------------------
    def result_bytes(self, session) -> bytes:
        s = self.sessions[session]
        return bytes(s.rx) if s.rx is not None else b""
