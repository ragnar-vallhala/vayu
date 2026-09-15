#!/usr/bin/env python3
"""End-to-end FILE upload -> download round-trip against the REAL flight
controller over the ESP8266 WiFi/UDP telemetry bridge.

Path: this host  ──UDP :14555──▶  ESP8266  ──UART USART6──▶  FC comm_processor
                                                              └ xfer_service_task
                                                                └ FILE provider ─ SD

What it does:
  1. Discover the bridge (broadcast GCS-HELLO, learn its address from the first
     telemetry datagram) and clear the FC's §10.5 command gate with a TIME_SYNC.
  2. UPLOAD a pseudo-random blob to an SD path via the xfer FILE provider
     (XFER_OPEN dir=UPLOAD -> stream XFER_DATA -> XFER_ACK flow control -> EOF/DONE).
  3. DOWNLOAD the same path back (XFER_OPEN dir=DOWNLOAD -> reassemble XFER_DATA).
  4. Byte-compare + SHA-256 the round-trip, and report achieved throughput each way.
  5. Bonus: FS_INFO + FS_LIST of the file/dir to exercise the new fs_query path.

Reliability model mirrors the firmware: offset-addressed cumulative-ack with
resume-by-rewind. We pace uploads with a sliding byte-window + token bucket
(the FC RX is a 460800-baud UART, ~44 KB/s payload ceiling) and a stall-retransmit
that rewinds to the FC's acked cursor so a lost EOF chunk still finalises.

Usage:
    python3 tools/telemetry/fs_xfer_udp_test.py                     # 16 KiB, 0:xtest.bin
    python3 tools/telemetry/fs_xfer_udp_test.py --size 65536 --rate 40000
    python3 tools/telemetry/fs_xfer_udp_test.py --port 14555 --path 0:xtest.bin --seed 7
"""
import argparse
import hashlib
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "navlink", "sim"))
sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))

import frame                       # noqa: E402
from common import load_nl         # noqa: E402

nl = load_nl()

GCS_SYS, GCS_COMP = 1, 1
FC_SYS, FC_COMP = 1, 1
DIR_DOWNLOAD, DIR_UPLOAD = 0, 1
MODE_FILE = 0
SVC_FILE = 0
CHUNK = 247                        # XFER_DATA payload (XFER_CHUNK_MAX)
# Cap the *upload* chunk so a full XFER_DATA frame fits the ESP bridge's inbound
# udp.read() buffer. Frame = 19 + data (10 hdr + 2 CRC + 7 xfer fields). The
# stock bridge uses cmd[256] and truncates larger frames; data<=237 keeps the
# frame <=256. (Bridges flashed with the cmd[512] fix can pass the full 247.)
ESP_UPLINK_FRAME_MAX = 256
UPLOAD_CHUNK_MAX = ESP_UPLINK_FRAME_MAX - 19   # = 237
TIME_SYNC_REQUEST_WIDE = 2         # forces _clock_synced=1 via set_offset64(0)

_F = nl.XferFlags
_R = nl.CommandResult
F_NONE = int(_F.NONE)
F_DONE = int(_F.DONE)
F_NAK = int(_F.NAK)
F_EOF = int(_F.EOF)
R_OK = int(_R.ACCEPTED)


# ── transport: one NavLink v2 frame per datagram out; coalesced frames in ──────
class Bridge:
    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.0)
        self.port = port
        self.peer = None
        self._seq = 0
        self.tx_frames = 0
        self.rx_frames = 0
        self.rx_bad = 0

    def hello(self):
        self.sock.sendto(b"GCS-HELLO", ("255.255.255.255", self.port))

    def send(self, msg):
        if self.peer is None:
            return
        f = frame.encode(msg.MSGID, msg.pack(), seq=self._seq,
                         sysid=GCS_SYS, compid=GCS_COMP)
        self._seq = (self._seq + 1) & 0xFF
        self.sock.sendto(f, self.peer)
        self.tx_frames += 1

    def poll(self):
        """Drain all pending datagrams; return list of Decoded frames."""
        out = []
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except (BlockingIOError, socket.timeout):
                break
            except OSError:
                break
            if data == b"GCS-HELLO":
                continue
            if self.peer != addr:
                self.peer = addr
                print(f"[link] bridge peer -> {addr[0]}:{addr[1]}")
            # One datagram = N coalesced whole frames (ESP packs to the MTU).
            off = 0
            while len(data) - off >= frame.HDR_LEN + 2:
                if data[off] != 0x56 or data[off + 1] != 0x02:
                    off += 1
                    continue
                flen = frame.HDR_LEN + data[off + 2] + 2
                if len(data) - off < flen:
                    break
                d = frame.decode(data[off:off + flen])
                off += flen
                if d.ok:
                    self.rx_frames += 1
                    out.append(d)
                else:
                    self.rx_bad += 1
        return out


def now_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFFFFFFFFFF


def pad247(chunk):
    return list(chunk) + [0] * (CHUNK - len(chunk))


# ── phase 0: discover + clear the command gate ────────────────────────────────
def discover_and_sync(br, timeout=20.0):
    print("[sync] discovering bridge + clearing §10.5 time-sync gate…")
    t0 = time.monotonic()
    last_hello = 0.0
    last_sync = 0.0
    synced = False
    seq = 0
    while time.monotonic() - t0 < timeout:
        t = time.monotonic()
        if t - last_hello >= 1.0:
            br.hello()
            last_hello = t
        for d in br.poll():
            if d.msgid == nl.TimeSync.MSGID:
                m = nl.TimeSync.unpack(d.payload)
                if int(getattr(m, "role", 0)) == 1:   # FC RESPONSE -> gate cleared
                    synced = True
        if br.peer is not None and t - last_sync >= 0.3:
            br.send(nl.TimeSync(role=TIME_SYNC_REQUEST_WIDE, seq=seq & 0xFF,
                                t1_gcs_tx=now_ms(), commanded_offset_ms=0,
                                commanded_offset_hi_ms=0))
            seq += 1
            last_sync = t
        if synced:
            print("[sync] FC clock disciplined — commands accepted.")
            return True
        time.sleep(0.01)
    print("[sync] FAILED: no TIME_SYNC response (board powered? on this network? "
          f"bridge on :{br.port}?)", file=sys.stderr)
    return False


# ── phase 1: upload (GCS -> FC), windowed cumulative-ack + stall-retransmit ────
def upload(br, session, path, payload, rate_bps, window, timeout=60.0):
    n = len(payload)
    print(f"[up ] XFER_OPEN upload '{path}'  {n} B "
          f"(rate≈{rate_bps/1000:.0f} KB/s, window {window} B)")
    chunk_size = CHUNK
    opened = False
    t0 = time.monotonic()
    # open (retry until INFO)
    last_open = 0.0
    while not opened and time.monotonic() - t0 < timeout:
        t = time.monotonic()
        if t - last_open >= 0.5:
            br.send(nl.XferOpen(target_sys=FC_SYS, target_comp=FC_COMP, req_seq=1,
                                session=session, dir=DIR_UPLOAD, mode=MODE_FILE,
                                service_id=SVC_FILE, offset_start=0, rate_hz=0,
                                arg=path))
            last_open = t
        for d in br.poll():
            if d.msgid == nl.XferInfo.MSGID:
                info = nl.XferInfo.unpack(d.payload)
                if info.session != session:
                    continue
                if info.result != R_OK:
                    print(f"[up ] open REJECTED result={info.result}", file=sys.stderr)
                    return None
                chunk_size = min(info.chunk_size or CHUNK, UPLOAD_CHUNK_MAX)
                opened = True
        time.sleep(0.005)
    if not opened:
        print("[up ] open timed out", file=sys.stderr)
        return None

    acked = 0
    tx_cursor = 0
    tokens = 0.0      # start empty: pace from the first chunk, no initial blast
    last_refill = time.monotonic()
    last_progress = time.monotonic()
    t_send0 = None
    done = False
    while not done and time.monotonic() - t0 < timeout:
        # inbound acks
        for d in br.poll():
            if d.msgid != nl.XferAck.MSGID:
                continue
            a = nl.XferAck.unpack(d.payload)
            if a.session != session:
                continue
            if a.next_offset > acked:
                acked = a.next_offset
                last_progress = time.monotonic()
            if a.flags & F_DONE:
                done = True
                break
            if (a.flags & F_NAK) or a.next_offset < tx_cursor:
                tx_cursor = a.next_offset       # rewind/resume
        if done:
            break
        # token-bucket refill
        t = time.monotonic()
        tokens = min(float(window), tokens + (t - last_refill) * rate_bps)
        last_refill = t
        # send forward within window + rate budget
        while (tx_cursor < n and (tx_cursor - acked) < window
               and tokens >= 1):
            off = tx_cursor
            chunk = payload[off:off + chunk_size]
            last = (off + len(chunk)) >= n
            br.send(nl.XferData(session=session, flags=(F_EOF if last else F_NONE),
                                len=len(chunk), offset=off, data=pad247(chunk)))
            if t_send0 is None:
                t_send0 = time.monotonic()
            tx_cursor += len(chunk)
            tokens -= len(chunk)
        # stall -> rewind to FC's acked cursor and resend (covers a lost EOF)
        if time.monotonic() - last_progress > 0.4 and acked < n:
            tx_cursor = acked
            last_progress = time.monotonic()
        time.sleep(0.002)
    if not done:
        print(f"[up ] timed out (acked {acked}/{n})", file=sys.stderr)
        return None
    dt = time.monotonic() - (t_send0 or t0)
    kbps = n / 1024.0 / dt if dt > 0 else 0.0
    print(f"[up ] DONE {n} B in {dt:.2f}s  =>  {kbps:.1f} KiB/s")
    return kbps


# ── phase 2: download (FC -> GCS), reassemble + cumulative-ack ─────────────────
def download(br, session, path, expect_n, timeout=60.0, sack=True):
    print(f"[dn ] XFER_OPEN download '{path}'")
    t0 = time.monotonic()
    total = None
    rx = None
    cursor = 0
    have = []                       # list of (off, len) runs
    last_open = 0.0
    last_ack = 0.0
    t_first = None
    done = False
    gaps = []
    while not done and time.monotonic() - t0 < timeout:
        t = time.monotonic()
        if total is None and t - last_open >= 0.5:
            br.send(nl.XferOpen(target_sys=FC_SYS, target_comp=FC_COMP, req_seq=2,
                                session=session, dir=DIR_DOWNLOAD, mode=MODE_FILE,
                                service_id=SVC_FILE, offset_start=0, rate_hz=0,
                                arg=path))
            last_open = t
        for d in br.poll():
            if d.msgid == nl.XferInfo.MSGID:
                info = nl.XferInfo.unpack(d.payload)
                if info.session != session:
                    continue
                if info.result != R_OK:
                    print(f"[dn ] open REJECTED result={info.result}", file=sys.stderr)
                    return None, None
                total = info.total_size
                rx = bytearray(total)
                print(f"[dn ] INFO total_size={total} chunk={info.chunk_size}")
            elif d.msgid == nl.XferData.MSGID and rx is not None:
                m = nl.XferData.unpack(d.payload)
                if m.session != session:
                    continue
                if t_first is None:
                    t_first = time.monotonic()
                end = m.offset + m.len
                if end <= len(rx):
                    rx[m.offset:end] = bytes(m.data[:m.len])
                    have.append((m.offset, m.len))
                    # advance contiguous cursor
                    have.sort()
                    for off, ln in have:
                        if off <= cursor < off + ln:
                            cursor = off + ln
                        elif off == cursor:
                            cursor = off + ln
                # Holes above the cumulative cursor, as (offset, length) pairs:
                # the gaps between the runs we do have. Capped at the 8 the
                # message carries; the rest are re-reported next time.
                gaps = []
                if sack:
                    probe = cursor
                    for off, ln in have:
                        if off > probe:
                            gaps.append((probe, min(off - probe, CHUNK)))
                            if len(gaps) >= 8:
                                break
                        probe = max(probe, off + ln)
                eof = bool(m.flags & F_EOF)
                if eof and cursor >= total:
                    br.send(nl.XferAck(session=session, flags=F_DONE,
                                       result=R_OK, next_offset=cursor))
                    done = True
                    break
                if sack:
                    # Selective repeat: name the holes above the cumulative
                    # cursor and let the FC refill only those. It does not
                    # rewind, so nothing already in flight comes back.
                    br.send(nl.XferSack(session=session, n_ranges=len(gaps),
                                        next_offset=cursor,
                                        miss_off=[g[0] for g in gaps] + [0] * (8 - len(gaps)),
                                        miss_len=[g[1] for g in gaps] + [0] * (8 - len(gaps))))
                else:
                    # Go-back-N: a NAK rewinds the sender to the cursor, so one
                    # loss re-sends everything since.
                    flags = F_NAK if m.offset > cursor else F_NONE
                    br.send(nl.XferAck(session=session, flags=flags, result=R_OK,
                                       next_offset=cursor))
                last_ack = t
        # Nudge a refill if the stream stalls below total. This MUST speak the
        # same protocol the transfer is using: a receiver that only reports on
        # arriving data cannot recover when the sender has gone quiet holding a
        # repair it believes is in flight -- no data, no report, no repair.
        if total is not None and not done and t - last_ack > 0.3 and cursor < total:
            if sack:
                br.send(nl.XferSack(session=session, n_ranges=len(gaps),
                                    next_offset=cursor,
                                    miss_off=[g[0] for g in gaps] + [0] * (8 - len(gaps)),
                                    miss_len=[g[1] for g in gaps] + [0] * (8 - len(gaps))))
            else:
                br.send(nl.XferAck(session=session, flags=F_NAK, result=R_OK,
                                   next_offset=cursor))
            last_ack = t
        time.sleep(0.002)
    if not done:
        print(f"[dn ] timed out (got {cursor}/{total})", file=sys.stderr)
        return None, None
    dt = time.monotonic() - (t_first or t0)
    kbps = expect_n / 1024.0 / dt if dt > 0 else 0.0
    print(f"[dn ] DONE {total} B in {dt:.2f}s  =>  {kbps:.1f} KiB/s")
    return bytes(rx), kbps


# ── bonus: exercise fs_query (FS_INFO + FS_LIST) ──────────────────────────────
def fs_probe(br, path, timeout=4.0):
    print(f"[fs ] FS_INFO '{path}' + FS_LIST '0:'")
    t0 = time.monotonic()
    info = None
    entries = {}
    list_total = None
    sent_info = sent_list = 0.0
    while time.monotonic() - t0 < timeout and (info is None or list_total is None):
        t = time.monotonic()
        if t - sent_info >= 0.5 and info is None:
            br.send(nl.FsInfo(target_sys=FC_SYS, target_comp=FC_COMP, req_seq=3,
                              path=path))
            sent_info = t
        if t - sent_list >= 0.5 and list_total is None:
            br.send(nl.FsList(target_sys=FC_SYS, target_comp=FC_COMP, req_seq=4,
                              start_index=0, path="0:"))
            sent_list = t
        for d in br.poll():
            if d.msgid == nl.FsInfoReply.MSGID:
                info = nl.FsInfoReply.unpack(d.payload)
            elif d.msgid == nl.FsEntry.MSGID:
                e = nl.FsEntry.unpack(d.payload)
                nm = e.name if isinstance(e.name, str) else \
                    bytes(e.name).split(b"\0")[0].decode("ascii", "replace")
                if e.result != R_OK:
                    list_total = -1
                elif nm == "":
                    list_total = e.count
                else:
                    entries[nm] = e.size
        time.sleep(0.005)
    if info is not None:
        ok = info.result == R_OK
        print(f"[fs ] FS_INFO -> {'OK' if ok else 'DENIED'} "
              f"size={info.size} type={info.type}")
    else:
        print("[fs ] FS_INFO -> (no reply)")
    if list_total == -1:
        print("[fs ] FS_LIST -> DENIED")
    elif list_total is not None:
        print(f"[fs ] FS_LIST -> {list_total} entries: " +
              ", ".join(f"{k}({v})" for k, v in sorted(entries.items())))
    else:
        print("[fs ] FS_LIST -> (no reply)")
    return info, entries


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=14555, help="UDP bridge port")
    ap.add_argument("--path", default="0:xtest.bin", help="SD path (8.3 name, drive 0)")
    ap.add_argument("--goback", action="store_true",
                    help="use go-back-N (XFER_ACK/NAK) instead of selective repeat")
    ap.add_argument("--size", type=int, default=16384, help="payload bytes")
    ap.add_argument("--rate", type=float, default=38000, help="upload byte-rate cap (B/s)")
    ap.add_argument("--window", type=int, default=16384, help="upload in-flight window (B)")
    ap.add_argument("--seed", type=int, default=1, help="PRNG seed for the blob")
    ap.add_argument("--settle", type=float, default=0.6, help="post-upload SD-flush wait (s)")
    args = ap.parse_args()

    # Deterministic, incompressible-ish payload (avoids trailing-zero truncation
    # quirks compressing every chunk; seed makes failures reproducible).
    import random
    rng = random.Random(args.seed)
    payload = bytes(rng.getrandbits(8) for _ in range(args.size))
    up_sha = hashlib.sha256(payload).hexdigest()
    print(f"[gen] payload {args.size} B  sha256={up_sha[:16]}…")

    br = Bridge(args.port)
    if not discover_and_sync(br):
        return 2

    up_kbps = upload(br, 0, args.path, payload, args.rate, args.window)
    if up_kbps is None:
        return 1
    br.send(nl.XferClose(target_sys=FC_SYS, target_comp=FC_COMP, req_seq=9,
                         session=0, result=R_OK))

    print(f"[..] settling {args.settle}s for the write-at lane to flush to SD…")
    time.sleep(args.settle)
    for _ in range(3):
        br.poll()
        br.hello()
        time.sleep(0.05)

    fs_probe(br, args.path)

    rx, dn_kbps = download(br, 1, args.path, args.size, sack=not args.goback)
    if rx is None:
        return 1
    br.send(nl.XferClose(target_sys=FC_SYS, target_comp=FC_COMP, req_seq=10,
                         session=1, result=R_OK))

    dn_sha = hashlib.sha256(rx).hexdigest()
    print("\n" + "═" * 60)
    match = (rx == payload)
    print(f"  round-trip   : {'✅ BYTE-EXACT MATCH' if match else '❌ MISMATCH'}")
    print(f"  size         : up {len(payload)} B  /  down {len(rx)} B")
    print(f"  sha256 up    : {up_sha}")
    print(f"  sha256 down  : {dn_sha}")
    if not match:
        # first differing offset, for debugging
        for i in range(min(len(rx), len(payload))):
            if rx[i] != payload[i]:
                print(f"  first diff   : offset {i}  up={payload[i]:#04x} down={rx[i]:#04x}")
                break
    if up_kbps and dn_kbps:
        print(f"  throughput   : up {up_kbps:.1f} KiB/s   down {dn_kbps:.1f} KiB/s")
    print(f"  link frames  : tx {br.tx_frames}  rx {br.rx_frames}  bad {br.rx_bad}")
    print("═" * 60)
    return 0 if match else 1


if __name__ == "__main__":
    sys.exit(main())
