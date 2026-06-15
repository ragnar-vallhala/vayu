"""NavLink v2 frame codec (spec §3/§4) for the simulator.

Encodes/decodes the on-wire frame: sync + 10-byte header + payload + CRC-16
seeded with CRC_EXTRA, with trailing-zero truncation (§5.6). One frame per UDP
datagram. Reuses the generated crc16/CRC_EXTRA so it stays in lock-step with the
dialect."""
import struct
from dataclasses import dataclass

from common import load_nl

nl = load_nl()

SYNC = 0x56
VERSION = 0x02
HDR_LEN = 10


def encode(msgid, payload, *, seq=0, sysid=1, compid=1, incompat=0, truncate=True):
    if truncate:
        payload = payload.rstrip(b"\x00")
    plen = len(payload) & 0xFF
    hdr = bytes([SYNC, VERSION, plen, incompat & 0xFF, seq & 0xFF,
                 sysid & 0xFF, compid & 0xFF,
                 msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    crc = nl.crc16(hdr[1:] + payload)               # exclude sync (§4.2)
    crc = nl.crc_accumulate(nl.CRC_EXTRA.get(msgid, 0), crc)
    return hdr + payload + struct.pack("<H", crc)


@dataclass
class Decoded:
    ok: bool
    reason: str = ""          # "", "crc", "unknown_msgid", "short", "non_frame"
    msgid: int = -1
    seq: int = 0
    sysid: int = 0
    compid: int = 0
    payload: bytes = b""
    nbytes: int = 0


def decode(data):
    """Parse + verify one datagram. Always returns a Decoded (never raises)."""
    n = len(data)
    if n < HDR_LEN + 2 or data[0] != SYNC or data[1] != VERSION:
        return Decoded(False, "non_frame", nbytes=n)
    plen = data[2]
    if n < HDR_LEN + plen + 2:
        return Decoded(False, "short", nbytes=n)
    payload = data[HDR_LEN:HDR_LEN + plen]
    msgid = data[7] | (data[8] << 8) | (data[9] << 16)
    rx_crc = struct.unpack_from("<H", data, HDR_LEN + plen)[0]
    ce = nl.CRC_EXTRA.get(msgid)
    if ce is None:
        return Decoded(False, "unknown_msgid", msgid=msgid, seq=data[4],
                       sysid=data[5], compid=data[6], nbytes=n)
    crc = nl.crc16(data[1:HDR_LEN] + payload)
    crc = nl.crc_accumulate(ce, crc)
    ok = (crc == rx_crc)
    return Decoded(ok, "" if ok else "crc", msgid=msgid, seq=data[4],
                   sysid=data[5], compid=data[6], payload=payload, nbytes=n)
