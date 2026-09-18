#!/usr/bin/env python3
"""Decoder + spectrum tool for the high-speed IMU stream ("HSL1").

The on-disk format is specified in firmware/include/storage/imu_hs_log.h. This
file is the reference reader for it, and `--selftest` is the check that the two
agree: it builds a file containing a stale block and an unknown frame type, and
asserts both are handled by the rules the spec promises.

Usage:
  hslog.py FILE                 summarise the recording
  hslog.py FILE --fft           dominant vibration peaks per axis
  hslog.py FILE --csv OUT.csv   dump samples
  hslog.py --selftest           verify decoder vs. format contract
"""
import argparse
import struct
import sys

MAGIC = 0x314C5348  # "HSL1"
SECTOR = 512        # the preamble, and every ring slot

T_PAD, T_FMT, T_BLOCK, T_EVENT, T_SESSION = 0x00, 0x01, 0x02, 0x03, 0x04
EV_NAME = {1: "state", 2: "flight_mode", 3: "accel_health", 4: "notch"}
STATE_NAME = {0x01: "UNINIT", 0x02: "INIT", 0x04: "STANDBY", 0x08: "PREARM",
              0x10: "ARMED", 0x20: "IN_AIR", 0x40: "FAILSAFE",
              0x80: "TERMINATED", 0x100: "CALIBRATING"}
SENTINEL = 0xA5     # frame `flags` in a ring slot; card garbage rarely has it
FTYPE = {1: ("h", 2), 2: ("H", 2), 3: ("i", 4), 4: ("f", 4)}


STREAM_NAME = {1: "imu", 2: "act", 3: "vrt", 4: "ctl"}


class Stream:
    def __init__(self, sid, rec_bytes, rate_hz, fields):
        self.sid, self.rec_bytes, self.rate_hz = sid, rec_bytes, rate_hz
        self.fields = fields  # [(name, ftype, scale)]

    @property
    def name(self):
        return STREAM_NAME.get(self.sid, "s%d" % self.sid)

    @property
    def names(self):
        return [f[0] for f in self.fields]


def _u32d(a, b):
    """Wrap-safe 32-bit delta. Blocks are ~20 ms apart and the DWT counter
    wraps every ~51 s, so accumulating these is exact."""
    return (a - b) & 0xFFFFFFFF


def decode(data):
    """-> (header, {sid: Stream}, [frame dicts in chronological order]).

    Two regimes, because the file has two.

    The PREAMBLE (sector 0) is generic framing -- an unknown frame type is
    skipped by its `len`, which is the format's extension contract.

    Everything after it is a CIRCULAR ring of one-sector frames, so slot order
    is not time order and there is no "end" to stop at: after the first wrap
    the oldest live sector sits immediately after the newest. Ordering comes
    from `seq`, which increases for the life of the file. Sort by it and the
    rotation, the crash gaps and the partially-filled first lap all come out
    right without special cases.
    """
    if len(data) < 32:
        raise ValueError("file shorter than the 32-byte header")
    magic, version, hdr_len, clock_hz = struct.unpack_from("<IHHI", data, 0)
    if magic != MAGIC:
        raise ValueError("bad magic %#010x (not an HSL file, or byte-swapped)" % magic)
    ring_start, ring_sectors, head_slot, next_seq, wraps = \
        struct.unpack_from("<IIIII", data, 12)
    hdr = dict(version=version, clock_hz=clock_hz, ring_start=ring_start,
               ring_sectors=ring_sectors, head_slot=head_slot,
               next_seq=next_seq, wraps=wraps)

    # --- preamble: generic frames up to the first sector boundary -----------
    streams, skipped = {}, {}
    off = hdr_len
    while off + 4 <= ring_start:
        ftype, _flags, plen = struct.unpack_from("<BBH", data, off)
        p = data[off + 4:off + 4 + plen]
        if ftype == T_FMT:
            sid, rec_bytes, rate_hz, nf = struct.unpack_from("<BBHB", p, 0)
            fields = []
            for i in range(nf):
                fo = 8 + i * 16
                name = p[fo:fo + 8].split(b"\0")[0].decode("ascii", "replace")
                (scale,) = struct.unpack_from("<f", p, fo + 12)
                fields.append((name, p[fo + 8], scale))
            streams[sid] = Stream(sid, rec_bytes, rate_hz, fields)
        elif ftype != T_PAD:
            skipped[ftype] = skipped.get(ftype, 0) + 1
        off += 4 + plen

    # --- ring: read every slot, keep what this firmware wrote ---------------
    frames = []
    for slot in range(ring_sectors):
        off = ring_start + slot * SECTOR
        if off + SECTOR > len(data):
            break
        ftype, flags, plen = struct.unpack_from("<BBH", data, off)
        # A slot the firmware never reached still holds whatever the card had
        # before these clusters were allocated. The sentinel plus the exact
        # length is what separates the two.
        if flags != SENTINEL or plen != SECTOR - 4:
            continue
        p = data[off + 4:off + SECTOR]
        seq = struct.unpack_from("<I", p, 4)[0]
        stag = p[1]  # session membership, present on every ring frame
        if ftype == T_BLOCK:
            sid, _rsv, n = struct.unpack_from("<BBH", p, 0)
            st = streams.get(sid)
            if st is None or n * st.rec_bytes > len(p) - 16:
                continue
            t_first, t_last = struct.unpack_from("<II", p, 8)
            frames.append(dict(kind="block", slot=slot, seq=seq, stag=stag,
                               sid=sid, n=n, t_first=t_first, t_last=t_last,
                               raw=p[16:]))
        elif ftype == T_SESSION:
            session, unix_lo, unix_hi, cyc0 = struct.unpack_from("<IIII", p, 8)
            frames.append(dict(kind="session", slot=slot, seq=seq, stag=stag,
                               session=session,
                               unix_ms=unix_lo | (unix_hi << 32),
                               cyc0=cyc0, synced=bool(p[24])))
        elif ftype == T_EVENT:
            t_cyc, ev = struct.unpack_from("<IB", p, 8)
            a, b = struct.unpack_from("<II", p, 16)
            frames.append(dict(kind="event", slot=slot, seq=seq, stag=stag,
                               t_cyc=t_cyc, ev=ev, a=a, b=b))
        else:
            skipped[ftype] = skipped.get(ftype, 0) + 1

    frames.sort(key=lambda f: f["seq"])

    # A seq gap means sectors were lost -- overwritten after a power cut, see
    # the resume margin in the firmware. It is not the end of the data.
    gaps = [(a["seq"], b["seq"]) for a, b in zip(frames, frames[1:])
            if b["seq"] != a["seq"] + 1]
    hdr["skipped_frames"] = skipped
    hdr["gaps"] = gaps
    hdr["slots_used"] = len(frames)
    return hdr, streams, frames


def sessions(frames):
    """Split the ring into armed periods.

    A new period starts at any of three things, and all three are needed:

      * a SESSION frame -- the authoritative marker, one per arm;
      * the per-frame session tag changing -- catches an arm whose own SESSION
        frame has already been overwritten by the ring;
      * a gap in seq -- a reboot or a power cut, which always ends an arm.

    The tag alone is not enough: the firmware's session counter restarts at 1
    on every boot (it is not carried in the file header the way head_slot and
    next_seq are), so two arms either side of a reset share tag 1 and would
    merge -- and a merged group accumulates the disarmed gap between them into
    its timebase, which then reads as one long session at a fraction of the
    real sample rate. Splitting on the SESSION frame fixes both.

    The one case still ambiguous: an arm whose SESSION frame wrapped away AND
    that shares a tag with the arm before it. Nothing in the file separates
    those; it needs the firmware to persist `session` across boots."""
    out, cur, tag, prev_seq = [], None, None, None

    def start(f):
        g = dict(session=None, unix_ms=None, cyc0=None, synced=False,
                 tag=f["stag"], blocks=[], events=[], truncated=False)
        out.append(g)
        return g

    for f in frames:
        gap = prev_seq is not None and f["seq"] != prev_seq + 1
        if (cur is None or f["kind"] == "session" or f["stag"] != tag or gap):
            if cur is not None and gap:
                cur["truncated"] = True  # ended by a reboot/power cut
            cur = start(f)
            tag = f["stag"]
        prev_seq = f["seq"]
        if f["kind"] == "session":
            cur.update(session=f["session"], unix_ms=f["unix_ms"],
                       cyc0=f["cyc0"], synced=f["synced"])
        elif f["kind"] == "block":
            cur["blocks"].append(f)
        elif f["kind"] == "event":
            cur["events"].append(f)
    return [g for g in out if g["blocks"] or g["events"]]


def session_end(sess):
    """How the arm ended, from its events: 'STANDBY', 'FAILSAFE', or None when
    no end event survives (power cut while armed)."""
    for e in reversed(sess["events"]):
        if e["ev"] == 1 and e["a"] in (0x04, 0x40):
            return STATE_NAME.get(e["a"])
    return None


def session_accel_unhealthy(sess):
    """True if the vertical estimator's accel-health veto latched during it."""
    return any(e["ev"] == 3 and e["a"] == 1 for e in sess["events"])


def describe_event(e):
    """Human text for one EVENT. `a` is the new value, `b` the previous."""
    name = EV_NAME.get(e["ev"], "kind%d" % e["ev"])
    if e["ev"] == 1:  # state
        f = lambda v: STATE_NAME.get(v, "0x%X" % v)
        return "state %s -> %s" % (f(e["b"]), f(e["a"]))
    if e["ev"] == 3:
        return "accel_health %s" % ("UNHEALTHY" if e["a"] else "recovered")
    if e["ev"] == 4:
        return "gyro notch %s" % ("on" if e["a"] else "off")
    return "%s %d -> %d" % (name, e["b"], e["a"])


def samples(hdr, streams, blocks, sid=1):
    """-> (t seconds, {name: numpy array in SI}) for one session's blocks.
    Per-sample time is interpolated inside a block and accumulated wrap-safely
    across blocks, so the ~51 s DWT rollover never shows."""
    import numpy as np

    st = streams.get(sid)
    if st is None:
        raise ValueError("no FMT for stream %d" % sid)
    fmt = "<" + "".join(FTYPE[f[1]][0] for f in st.fields)
    if struct.calcsize(fmt) != st.rec_bytes:
        raise ValueError("FMT fields (%dB) disagree with rec_bytes (%d)"
                         % (struct.calcsize(fmt), st.rec_bytes))

    cols = {n: [] for n in st.names}
    times, elapsed, prev_last, prev_step = [], 0, None, None
    gaps = 0
    for b in blocks:
        if b.get("kind", "block") != "block" or b["sid"] != sid:
            continue
        if prev_last is not None:
            step = _u32d(b["t_first"], prev_last)
            # Consecutive blocks of a stream are one sample apart. Much more
            # than that means sectors are missing (a dropped sector, or a
            # reboot between them) -- and since the DWT stamp wraps every ~51 s
            # the true gap is not recoverable anyway. Step one sample instead
            # of inventing a duration, and count the break.
            #
            # The threshold calibrates itself off the previous block's own
            # measured sample interval, so it holds for all three streams
            # without a per-stream constant (their block periods differ by 40x).
            if prev_step and step > 4 * prev_step:
                gaps += 1
                step = prev_step
            elapsed += step
        t0 = elapsed
        span = _u32d(b["t_last"], b["t_first"])
        n = b["n"]
        for i in range(n):
            rec = struct.unpack_from(fmt, b["raw"], i * st.rec_bytes)
            for name, v in zip(st.names, rec):
                cols[name].append(v)
            times.append(t0 + (span * i // max(n - 1, 1)))
        elapsed = t0 + span
        prev_last = b["t_last"]
        if n > 1:
            prev_step = span // (n - 1)

    t = np.asarray(times, dtype=np.float64) / hdr["clock_hz"]
    out = {k: np.asarray(cols[k], dtype=np.float64) * sc
           for (_n, _ft, sc), k in zip(st.fields, st.names)}
    out["_gaps"] = gaps
    return t, out


def spectrum(t, series, top=5):
    """-> {name: [(hz, amplitude)]}, strongest first. Uses the measured mean
    sample rate, not the nominal one, so a clock that ran off spec still gives
    a correctly-scaled frequency axis."""
    import numpy as np

    if len(t) < 64:
        return {}
    fs = (len(t) - 1) / (t[-1] - t[0])
    out = {}
    for name, y in series.items():
        if name.startswith("_"):   # bookkeeping, not a signal
            continue
        y = y - y.mean()
        w = np.hanning(len(y))
        mag = np.abs(np.fft.rfft(y * w)) * (2.0 / w.sum())
        freq = np.fft.rfftfreq(len(y), 1.0 / fs)
        lo = freq > 5.0  # ignore DC and airframe rigid-body motion
        idx = np.argsort(mag[lo])[::-1][:top]
        out[name] = [(float(freq[lo][i]), float(mag[lo][i])) for i in idx]
    return out, fs


# ---------------------------------------------------------------------------
RING_SECTORS = 8  # tiny ring, so the selftest can actually wrap it


def _ring_frame(ftype, seq, body, stag=0):
    p = bytearray(SECTOR - 4)
    p[1] = stag
    struct.pack_into("<I", p, 4, seq)
    p[8:8 + len(body)] = body
    return struct.pack("<BBH", ftype, SENTINEL, SECTOR - 4) + bytes(p)


def _session_frame(seq, session, unix_ms, cyc0, synced=1):
    return _ring_frame(T_SESSION, seq,
                       struct.pack("<IIIIB", session, unix_ms & 0xFFFFFFFF,
                                   unix_ms >> 32, cyc0, synced),
                       stag=session & 0xFF)


def _block_frame(seq, n, t0, t1, gen, stag=1):
    p = bytearray(SECTOR - 4)
    struct.pack_into("<BBH", p, 0, 1, stag, n)
    struct.pack_into("<I", p, 4, seq)
    struct.pack_into("<II", p, 8, t0, t1)
    for i in range(n):
        struct.pack_into("<hhhhhh", p, 16 + i * 12, *gen(i))
    return struct.pack("<BBH", T_BLOCK, SENTINEL, SECTOR - 4) + bytes(p)


def _preamble(head_slot, next_seq, wraps):
    sec = bytearray(SECTOR)
    struct.pack_into("<IHHI", sec, 0, MAGIC, 1, 32, 84_000_000)
    struct.pack_into("<IIIII", sec, 12, SECTOR, RING_SECTORS, head_slot,
                     next_seq, wraps)
    fields = [("gx", 1, -0.0610), ("gy", 1, 0.0610), ("gz", 1, -0.0610),
              ("ax", 1, -0.0048), ("ay", 1, 0.0048), ("az", 1, -0.0048)]
    pay = struct.pack("<BBHBBBB", 1, 12, 2000, len(fields), 0, 0, 0)
    for name, ft, scale in fields:
        pay += name.encode().ljust(8, b"\0") + bytes([ft, 0, 0, 0]) + struct.pack("<f", scale)
    struct.pack_into("<BBH", sec, 32, T_FMT, 0, len(pay))
    sec[36:36 + len(pay)] = pay
    off = 36 + len(pay)
    # an unknown frame type in the preamble: must be skipped by len
    struct.pack_into("<BBH", sec, off, 0x7E, 0, 6)
    sec[off + 4:off + 10] = b"\xde\xad\xbe\xef\x00\x01"
    off += 10
    struct.pack_into("<BBH", sec, off, T_PAD, 0, SECTOR - off - 4)
    return bytes(sec)


def _build_test_file():
    """A file shaped like a real one: a preamble carrying an unknown frame type
    that must be skipped, two armed periods separated by a REBOOT (a seq gap --
    the only way one occurs), the newer of which has outlived its own SESSION
    frame, all in a ring that has WRAPPED so slot order is not time order, plus
    one slot of raw card content that was never written.

    slot:  0     1     2     3        4       5    6    7
    seq:   22    23    24    garbage  s:12    13   14   15
           `--- arm 2 (tag 2) ---'    `--- arm 1 (tag 1) ---'
                                       (older lap)
    """
    import math

    session = 0x107  # low byte 0x07 -- arm 1's tag
    sec = bytearray(SECTOR)
    struct.pack_into("<IHHI", sec, 0, MAGIC, 1, 32, 84_000_000)
    struct.pack_into("<IIIII", sec, 12, SECTOR, RING_SECTORS, 3, 25, 1)

    fields = [("gx", 1, -0.0610), ("gy", 1, 0.0610), ("gz", 1, -0.0610),
              ("ax", 1, -0.0048), ("ay", 1, 0.0048), ("az", 1, -0.0048)]
    pay = struct.pack("<BBHBBBB", 1, 12, 2000, len(fields), 0, 0, 0)
    for name, ft, scale in fields:
        pay += name.encode().ljust(8, b"\0") + bytes([ft, 0, 0, 0]) + struct.pack("<f", scale)
    struct.pack_into("<BBH", sec, 32, T_FMT, 0, len(pay))
    sec[36:36 + len(pay)] = pay
    off = 36 + len(pay)
    # an unknown frame type inside the preamble: must be skipped by len
    struct.pack_into("<BBH", sec, off, 0x7E, 0, 6)
    sec[off + 4:off + 10] = b"\xde\xad\xbe\xef\x00\x01"
    off += 10
    struct.pack_into("<BBH", sec, off, T_PAD, 0, SECTOR - off - 4)

    n, cyc = 41, 84_000_000 // 2000
    flat = lambda i: (7, 0, 0, 0, 0, 0)

    def tone(base):
        # continuous phase across blocks, so the FFT sees one clean 200 Hz tone
        return lambda i: (int(1000 * math.sin(2 * math.pi * 200 * (base + i) / 2000)),
                          0, 0, 0, 0, 0)

    slots = [None] * RING_SECTORS
    # arm 1 -- older lap, keeps its SESSION frame
    slots[4] = _session_frame(12, 1, 1_757_000_000_000, 0)
    for k in range(3):
        slots[5 + k] = _block_frame(13 + k, n, cyc * n * k, cyc * (n * (k + 1) - 1),
                                    flat, stag=1)
    # REBOOT: seq jumps the resume margin. arm 2 wrapped round and overwrote
    # slots 0..2; its own SESSION frame landed on the previous lap and is gone.
    for k in range(3):
        slots[k] = _block_frame(22 + k, n, cyc * n * k, cyc * (n * (k + 1) - 1),
                                tone(n * k), stag=2)
    # slot 3 was never rewritten: raw card content, no sentinel
    slots[3] = bytes((i * 37 + 11) & 0xFF for i in range(SECTOR))

    out = bytes(sec)
    for sl in slots:
        out += sl
    return out


def selftest():
    import numpy as np

    hdr, streams, frames = decode(_build_test_file())

    assert hdr["version"] == 1, hdr
    assert hdr["clock_hz"] == 84_000_000, hdr
    assert hdr["ring_sectors"] == RING_SECTORS, hdr
    assert hdr["wraps"] == 1, hdr
    assert hdr["skipped_frames"] == {0x7E: 1}, \
        "unknown frame type must be skipped by len and counted: %r" % (hdr["skipped_frames"],)

    st = streams[1]
    assert st.rec_bytes == 12 and st.rate_hz == 2000, vars(st)
    assert st.names == ["gx", "gy", "gz", "ax", "ay", "az"], st.names

    # the garbage slot must not be mistaken for a frame
    assert hdr["slots_used"] == 7, hdr["slots_used"]
    assert all(f["slot"] != 3 for f in frames), "card garbage decoded as a frame"

    # seq ordering must beat slot ordering: slot 4 is OLDER than slot 0
    assert [f["seq"] for f in frames] == [12, 13, 14, 15, 22, 23, 24], \
        [f["seq"] for f in frames]
    assert [f["slot"] for f in frames] == [4, 5, 6, 7, 0, 1, 2], \
        "ring rotation not undone: %r" % [f["slot"] for f in frames]

    # the reboot gap is reported, not treated as the end of the file
    assert hdr["gaps"] == [(15, 22)], hdr["gaps"]

    ss = sessions(frames)
    assert len(ss) == 2, "expected two armed periods, got %d" % len(ss)
    assert ss[0]["session"] == 1 and ss[0]["synced"], ss[0]
    assert ss[0]["truncated"], "arm ended by a reboot must be marked truncated"
    assert len(ss[0]["blocks"]) == 3, len(ss[0]["blocks"])
    assert ss[1]["session"] is None and ss[1]["tag"] == 2, \
        "an arm whose SESSION frame wrapped away must still split off: %r" % ss[1]
    assert len(ss[1]["blocks"]) == 3, len(ss[1]["blocks"])

    # the 200 Hz tone in the newest arm must come back at 200 Hz, and its
    # timebase must be continuous (no gap inside an arm)
    t, sig = samples(hdr, streams, ss[1]["blocks"])
    assert len(t) == 123, len(t)
    assert sig["_gaps"] == 0, "no gap should be seen inside one arm"
    assert np.all(np.diff(t) >= 0), "time base must be monotonic"
    peaks, fs = spectrum(t, {"gx": sig["gx"]})
    assert abs(fs - 2000) < 60, "recovered sample rate %.1f" % fs
    f0 = peaks["gx"][0][0]
    assert abs(f0 - 200) < 20, "recovered tone at %.1f Hz, expected 200" % f0

    assert _u32d(5, 0xFFFFFFF0) == 21, _u32d(5, 0xFFFFFFF0)
    print("selftest OK: wrapped ring reordered, reboot gap %r split 2 arms, "
          "tone at %.1f Hz, fs %.1f Hz" % (hdr["gaps"], f0, fs))


SEQ_RESUME_MARGIN = 32  # HSL_SEQ_RESUME_MARGIN in imu_hs_log.h
ENC_MOTORS = [0.1, 0.2, 0.3, 0.4]
ENC_THROTTLE = 0.5
ENC_VRT = dict(baro=1.5, agl=2.5, agltof=3.5, alt=4.5, climb=5.5, abias=6.5)


def verify_encoder_file(path):
    """Decode a file the C encoder wrote. This is the only check that pins the
    firmware writer and this reader to the same format -- everything else
    verifies each half against its own idea of the spec.

    sim/host/tests/test_hslog.c drives three armed sessions of four streams
    through a 63-slot ring with a simulated reboot, so the file exercises
    rotation, an overwritten SESSION frame, per-stream decimation, EVENT frames
    and a reboot seq jump at once."""
    import numpy as np

    with open(path, "rb") as fh:
        data = fh.read()
    hdr, streams, frames = decode(data)

    assert hdr["version"] == 1, hdr
    assert hdr["clock_hz"] == 84_000_000, hdr
    assert not hdr["skipped_frames"], hdr["skipped_frames"]
    assert hdr["wraps"] >= 1, "test should have wrapped the ring: %r" % hdr

    # --- all four streams declared, with their rates and layouts -----------
    assert sorted(streams) == [1, 2, 3, 4], sorted(streams)
    imu, act, vrt, ctl = streams[1], streams[2], streams[3], streams[4]
    assert (imu.rec_bytes, imu.rate_hz) == (12, 2000), vars(imu)
    assert (act.rec_bytes, act.rate_hz) == (12, 400), vars(act)
    assert (vrt.rec_bytes, vrt.rate_hz) == (28, 20), vars(vrt)
    assert (ctl.rec_bytes, ctl.rate_hz) == (12, 1000), vars(ctl)
    assert imu.names == ["gx", "gy", "gz", "ax", "ay", "az"], imu.names
    assert act.names == ["m1", "m2", "m3", "m4", "thr", "flags"], act.names
    assert vrt.names[:6] == ["baro", "agl", "agltof", "alt", "climb", "abias"], vrt.names
    assert ctl.names == ["rfx", "rfy", "rfz", "ux", "uy", "uz"], ctl.names
    # "ctl" rates share the gyro's count scale so the two streams can be
    # differenced; "imu" carries the sensor->body sign map, "ctl" does not.
    assert ctl.fields[0][2] == abs(imu.fields[0][2]), (ctl.fields[0], imu.fields[0])
    gscale = imu.fields[0][2]
    assert abs(abs(gscale) - 0.0610351562) < 1e-6, gscale

    # --- one seq gap, and it is the deliberate reboot jump ------------------
    assert len(hdr["gaps"]) == 1, "expected only the reboot gap: %r" % (hdr["gaps"],)
    a, b = hdr["gaps"][0]
    assert b - a >= SEQ_RESUME_MARGIN, \
        "reboot must clear the resume margin, jumped %d" % (b - a)

    ss = sessions(frames)
    assert len(ss) >= 2, "expected several armed periods, got %d" % len(ss)
    assert len({x["tag"] for x in ss}) == len(ss), "sessions share a tag"

    # At least one session ran off the end of the ring and round. Only seq
    # ordering recovers that; slot order alone would tear it in half.
    assert any(x["blocks"][-1]["slot"] < x["blocks"][0]["slot"] for x in ss), \
        "ring did not actually wrap; the test is not exercising rotation"

    # --- "ctl" round-trips what the encoder was handed ----------------------
    # test_hslog.c pushes constant rates and outputs, so anything but those
    # values back means the scale, the rounding or the packing has moved.
    for x in ss:
        try:
            t, sig = samples(hdr, streams, x["blocks"], sid=4)
        except Exception:
            continue
        if not len(t):
            continue
        want = {"rfx": 10.0, "rfy": -20.0, "rfz": 30.0,
                "ux": 0.25, "uy": -0.5, "uz": 0.125}
        for k, v in want.items():
            got = sig[k]
            lsb = abs(dict((f[0], f[2]) for f in ctl.fields)[k])
            assert abs(got[0] - v) <= lsb, "ctl %s: %r, wanted %r" % (k, got[0], v)
            assert (got == got[0]).all(), "ctl %s is not constant: %r" % (k, got[:4])
        fs = (len(t) - 1) / (t[-1] - t[0])
        assert abs(fs - 1000) < 1.0, "ctl rate %.2f Hz, expected 1000" % fs
        break

    # --- EVENT frames: arm and disarm must both be visible ------------------
    evs = [e for x in ss for e in x["events"]]
    assert evs, "no EVENT frames survived"
    states = [(e["b"], e["a"]) for e in evs if e["ev"] == 1]
    assert any(new == 0x10 for _old, new in states), \
        "no arm event: %r" % states
    assert any(old == 0x10 and new == 0x04 for old, new in states), \
        "no disarm event -- the end of a flight is unmarked: %r" % states

    # --- per-stream content -------------------------------------------------
    checked = set()
    for sess in ss:
        for sid in (1, 2, 3):
            blks = [x for x in sess["blocks"] if x["sid"] == sid]
            if not blks:
                continue
            t, sig = samples(hdr, streams, blks, sid)
            if len(t) < 3:
                continue
            checked.add(sid)
            fs = (len(t) - 1) / (t[-1] - t[0])
            assert np.all(np.diff(t) > 0), \
                "stream %d time base must be strictly increasing" % sid

            if sid == 1:
                # gx is a ramp in counts; a session whose head was overwritten
                # starts partway up it, so require contiguity, not a zero start.
                idx = np.rint(sig["gx"] / gscale)
                assert np.all(np.diff(idx) == 1), \
                    "imu ramp not contiguous: %r" % idx[:8]
                assert abs(fs - 2000) < 1.0, "imu rate %.2f Hz" % fs
            elif sid == 2:
                # Decimation is the module's, not the caller's: the test offers
                # act samples at 2 kHz and they must come back at 400.
                assert abs(fs - 400) < 2.0, "act rate %.2f Hz, expected 400" % fs
                for k, want in zip(["m1", "m2", "m3", "m4"], ENC_MOTORS):
                    assert np.allclose(sig[k], want, atol=2e-5), \
                        "%s round-tripped to %r, expected %g" % (k, sig[k][:3], want)
                assert np.allclose(sig["thr"], ENC_THROTTLE, atol=2e-5), sig["thr"][:3]
            else:
                assert abs(fs - 20) < 0.5, "vrt rate %.2f Hz, expected 20" % fs
                for k, want in ENC_VRT.items():
                    assert np.allclose(sig[k], want), \
                        "%s round-tripped to %r, expected %g" % (k, sig[k][:3], want)

    assert checked == {1, 2, 3}, "did not see all three streams: %r" % checked

    print("encoder round-trip OK: %s, %d frames, %d wraps, %d sessions, "
          "%d events, 4 streams at 2000/1000/400/20 Hz, reboot gap %d"
          % (path, len(frames), hdr["wraps"], len(ss), len(evs), b - a))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?")
    ap.add_argument("--fft", action="store_true", help="report dominant peaks")
    ap.add_argument("--csv", help="write samples to this path")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--selftest-file", metavar="FILE",
                    help="decode a file the firmware encoder wrote and verify "
                         "it against the format contract (the ramp on gx that "
                         "sim/host/tests/test_hslog.c writes)")
    a = ap.parse_args()

    if a.selftest:
        selftest()
        return 0
    if a.selftest_file:
        verify_encoder_file(a.selftest_file)
        return 0
    if not a.file:
        ap.error("need a file (or --selftest)")

    with open(a.file, "rb") as fh:
        hdr, streams, frames = decode(fh.read())
    ss = sessions(frames)
    print("%d ring slots used, %d wraps, %d armed sessions"
          % (hdr["slots_used"], hdr["wraps"], len(ss)))
    if hdr["gaps"]:
        print("seq gaps %r -- a reboot, or sectors lost to a power cut" % (hdr["gaps"],))
    if hdr["skipped_frames"]:
        print("skipped unknown frame types: %r" % hdr["skipped_frames"])

    all_series = {}
    for i, sess in enumerate(ss):
        when = "no SESSION frame (header wrapped away)"
        if sess["unix_ms"] is not None:
            import datetime
            when = datetime.datetime.fromtimestamp(
                sess["unix_ms"] / 1000.0).strftime("%Y-%m-%d %H:%M:%S")
            if not sess["synced"]:
                when += "  (clock never synced -- uptime, not epoch)"
        print("\n[%d] session %s  started %s"
              % (i, sess["session"] if sess["session"] is not None else "?", when))

        for sid in sorted(streams):
            blks = [b for b in sess["blocks"] if b["sid"] == sid]
            if not blks:
                continue
            t, sig = samples(hdr, streams, blks, sid)
            if len(t) < 2:
                continue
            dur = t[-1] - t[0]
            note = ("  %d GAP%s" % (sig["_gaps"], "s" if sig["_gaps"] > 1 else "")
                    if sig.get("_gaps") else "")
            print("    %-5s %5d samples  %6.2f s  %7.1f Hz  [%s]%s"
                  % (streams[sid].name, len(t), dur, (len(t) - 1) / dur,
                     ",".join(streams[sid].names), note))
            all_series[(i, sid)] = (t, sig)

            if a.fft and sid == 1:
                peaks, fs = spectrum(t, sig)
                print("      dominant peaks (>5 Hz), fs %.1f Hz" % fs)
                for name in sig:
                    row = "  ".join("%7.1f Hz %8.3f" % pk for pk in peaks[name][:4])
                    print("        %-3s %s" % (name, row))

        for e in sess["events"]:
            print("    event: %s" % describe_event(e))

    if a.csv:
        keys = [k for k in all_series if k[1] == 1]
        if not keys:
            print("nothing to write")
            return 0
        key = max(keys)
        t, sig = all_series[key]
        with open(a.csv, "w") as fh:
            cols = [c for c in sig if not c.startswith("_")]
            fh.write("t," + ",".join(cols) + "\n")
            for k in range(len(t)):
                fh.write("%.6f," % t[k] + ",".join("%.4f" % sig[c][k] for c in cols) + "\n")
        print("wrote %s (session index %d, stream %d)" % (a.csv, key[0], key[1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
