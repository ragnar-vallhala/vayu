#!/usr/bin/env python3
"""NavLink v2 Python codec tests + spec §16 conformance vectors.

Run directly (`python3 tests/test_codec.py`) or via `tests/run_tests.py`, which
also compiles the C codec and checks cross-language parity.
"""
import os
import struct
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)                                  # generate.py
sys.path.insert(0, os.path.join(ROOT, "generated", "python"))  # navlink_msgs.py

import generate  # noqa: E402

# Ensure the generated module exists before importing it.
if not os.path.exists(os.path.join(ROOT, "generated", "python", "navlink_msgs.py")):
    os.system(f"{sys.executable} {os.path.join(ROOT, 'generate.py')} >/dev/null")

import navlink_msgs as nl  # noqa: E402

with open(os.path.join(ROOT, "dialect.json")) as fh:
    import json
    DIALECT = json.load(fh)


# ── test-side frame layer (spec §3/§4) — NOT generated; exercises the generated
#    crc16 + per-message CRC_EXTRA in real framing. ─────────────────────────────
SYNC = 0x56
VERSION_V2 = 0x02


def build_frame(msgid, payload, crc_extra, *, seq=0, sysid=1, compid=1, incompat=0,
                truncate=True):
    """Assemble an unsecured v2 frame (spec §3.2, §4.2, §5.6)."""
    if truncate:
        payload = payload.rstrip(b"\x00")           # trailing-zero truncation (§5.6)
    hdr = bytes([SYNC, VERSION_V2, len(payload), incompat, seq, sysid, compid,
                 msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    crc = nl.crc16(hdr[1:] + payload)               # exclude sync (§4.2)
    crc = nl.crc_accumulate(crc_extra, crc)         # CRC_EXTRA accumulated last
    return hdr + payload + struct.pack("<H", crc)


def parse_frame(frame):
    """Validate + split a v2 frame. Returns (msgid, payload_padded_len_hint, payload)."""
    assert frame[0] == SYNC and frame[1] == VERSION_V2
    plen = frame[2]
    payload = frame[10:10 + plen]
    msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
    rx_crc = struct.unpack_from("<H", frame, 10 + plen)[0]
    return msgid, payload, rx_crc


def verify_frame(frame, crc_extra):
    msgid, payload, rx_crc = parse_frame(frame)
    crc = nl.crc16(frame[1:10] + payload)
    crc = nl.crc_accumulate(crc_extra, crc)
    return crc == rx_crc, msgid, payload


def demux_version(byte0, byte1):
    """Spec §3.4 / §16.5."""
    if byte0 != SYNC:
        return "resync"
    if (byte1 & 0x0F) == 0x01:
        return "v1"
    if byte1 == 0x02:
        return "v2"
    return "resync"


# ── tests ─────────────────────────────────────────────────────────────────────
class TestCRC(unittest.TestCase):
    def test_crc16_self_check(self):                     # §16.1
        self.assertEqual(nl.crc16(b"123456789"), 0x6F91)

    def test_crc_extra_is_excluded_zero_is_valid(self):
        # a checksum of 0x0000 must not be special-cased (§4.2); just sanity that
        # crc16 of empty is the init value.
        self.assertEqual(nl.crc16(b""), 0xFFFF)


class TestCrcExtraContract(unittest.TestCase):
    def test_attitude_euler_golden(self):                # §16.2 worked example
        # Pinned golden value; both C and Python must agree (cross-checked in
        # run_tests.py). If this changes, the wire layout changed.
        self.assertEqual(nl.AttitudeEuler.CRC_EXTRA, 213)

    def test_crc_extra_matches_manual_string(self):
        # Recompute the §4.3 input string for ATTITUDE_EULER independently.
        s = b"ATTITUDE_EULER "
        for name in ("roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"):
            s += b"f32 " + name.encode() + b" "
        crc = 0xFFFF
        for b in s:
            crc = nl.crc_accumulate(b, crc)
        expected = ((crc & 0xFF) ^ (crc >> 8)) & 0xFF
        self.assertEqual(expected, nl.AttitudeEuler.CRC_EXTRA)

    def test_extension_excluded_from_crc_extra(self):
        # IMU_RAW has an extension field (sample_time_us). Removing it from the
        # dialect must NOT change CRC_EXTRA (spec §4.3 / §5.5).
        msg = next(m for m in DIALECT["messages"] if m["name"] == "IMU_RAW")
        self.assertTrue(any(f.get("extension") for f in msg["fields"]))
        no_ext = dict(msg, fields=[f for f in msg["fields"] if not f.get("extension")])
        self.assertEqual(generate.crc_extra(msg), generate.crc_extra(no_ext))
        self.assertEqual(generate.crc_extra(msg), nl.ImuRaw.CRC_EXTRA)


class TestRoundTrip(unittest.TestCase):
    def test_all_messages_round_trip(self):
        for cls in nl.MSGID_TO_CLASS.values():
            with self.subTest(msg=cls.__name__):
                obj = cls()
                buf = obj.pack()
                self.assertEqual(len(buf), cls.WIRE_SIZE,
                                 f"{cls.__name__} packed {len(buf)} != WIRE_SIZE {cls.WIRE_SIZE}")
                back = cls.unpack(buf)
                self.assertEqual(back.pack(), buf)

    def test_attitude_euler_values(self):
        a = nl.AttitudeEuler(roll=0.5, pitch=-0.25, yaw=1.0,
                             rollspeed=0.1, pitchspeed=0.2, yawspeed=0.3)
        b = nl.AttitudeEuler.unpack(a.pack())
        self.assertAlmostEqual(b.roll, 0.5, places=6)
        self.assertAlmostEqual(b.yawspeed, 0.3, places=6)

    def test_u24_command_ack(self):
        k = nl.CommandAck(command=0x123456, req_seq=7,
                          result=nl.CommandResult.ACCEPTED, progress=100, result_param2=-3)
        k2 = nl.CommandAck.unpack(k.pack())
        self.assertEqual(k2.command, 0x123456)
        self.assertEqual(k2.result_param2, -3)

    def test_char_array(self):
        pv = nl.ParamValue(param_id="ROLL_KP", index=2, count=40, generation=5,
                           type=nl.ParamType.F32, value=list(range(8)))
        pv2 = nl.ParamValue.unpack(pv.pack())
        self.assertEqual(pv2.param_id, "ROLL_KP")
        self.assertEqual(pv2.value, list(range(8)))


def build_canonical(cls, msg):
    """Instance filled with the shared deterministic per-field values."""
    obj = cls()
    for (name, _, _), v in zip(cls._FIELDS, generate.canonical_values(msg)):
        setattr(obj, name, v)
    return obj


BY_ID = {m["msgid"]: m for m in DIALECT["messages"]}


class TestValueRoundTrip(unittest.TestCase):
    """Round-trip every message with non-trivial values across every field type."""

    def _assert_eq(self, cls, a, b):
        for name, ftype, ln in cls._FIELDS:
            va, vb = getattr(a, name), getattr(b, name)
            if ftype in ("f32", "f64"):
                if ln is None:
                    self.assertAlmostEqual(va, vb, places=4, msg=f"{cls.__name__}.{name}")
                else:
                    for x, y in zip(va, vb):
                        self.assertAlmostEqual(x, y, places=4, msg=f"{cls.__name__}.{name}")
            else:
                self.assertEqual(va, vb, f"{cls.__name__}.{name}")

    def test_all_messages_value_round_trip(self):
        for msgid, cls in nl.MSGID_TO_CLASS.items():
            with self.subTest(msg=cls.__name__):
                obj = build_canonical(cls, BY_ID[msgid])
                self._assert_eq(cls, obj, cls.unpack(obj.pack()))


class TestTypeExtremes(unittest.TestCase):
    def test_u64_extremes(self):
        t = nl.TimeReference(epoch_unix_s=0xDEADBEEF, gcs_send_us=0xFFFFFFFFFFFFFFFF)
        self.assertEqual(nl.TimeReference.unpack(t.pack()).gcs_send_us, 0xFFFFFFFFFFFFFFFF)

    def test_signed_negative_array(self):
        g = nl.CmdSetMotorGeometry(spin=[1, -1, 1, -1])
        self.assertEqual(nl.CmdSetMotorGeometry.unpack(g.pack()).spin, [1, -1, 1, -1])

    def test_i32_negative(self):
        k = nl.CommandAck(result_param2=-2000000000)
        self.assertEqual(nl.CommandAck.unpack(k.pack()).result_param2, -2000000000)

    def test_u24_max(self):
        k = nl.CommandAck(command=0xFFFFFF)
        self.assertEqual(nl.CommandAck.unpack(k.pack()).command, 0xFFFFFF)

    def test_float_array(self):
        c = nl.CalibrationStatus(step=8, progress=50, coverage=[1.25, -2.5, 3.75])
        c2 = nl.CalibrationStatus.unpack(c.pack())
        self.assertEqual(c2.coverage, [1.25, -2.5, 3.75])


class TestEdgeUnpack(unittest.TestCase):
    def test_oversize_buffer_clamped(self):
        extra = nl.AttitudeEuler(roll=1.0).pack() + b"\xAA" * 8
        self.assertAlmostEqual(nl.AttitudeEuler.unpack(extra).roll, 1.0, places=6)

    def test_short_buffer_zero_filled(self):
        pv = nl.ParamValue.unpack(b"\x01\x02")
        self.assertEqual(pv.count, 0)
        self.assertEqual(pv.value, [0] * 8)

    def test_empty_buffer(self):
        self.assertEqual(nl.PerfFifo.unpack(b"").drops, 0)


class TestRegistry(unittest.TestCase):
    def test_crc_extra_dict_matches_classes(self):
        for msgid, cls in nl.MSGID_TO_CLASS.items():
            self.assertEqual(nl.CRC_EXTRA[msgid], cls.CRC_EXTRA)

    def test_registry_covers_dialect(self):
        self.assertEqual({m["msgid"] for m in DIALECT["messages"]}, set(nl.MSGID_TO_CLASS))

    def test_msgids_in_core_half(self):
        for msgid in nl.MSGID_TO_CLASS:
            self.assertLessEqual(msgid, 0x7FFFFF)


class TestFraming(unittest.TestCase):
    def test_heartbeat_frame_layout(self):               # §16.3 step 1
        hb = nl.Heartbeat(type=2, autopilot=1, base_mode=0, system_status=4,
                          nav_state=int(nl.NavState.STANDBY), capabilities=0xABCD,
                          timestamp=0x00010203)
        frame = build_frame(nl.Heartbeat.MSGID, hb.pack(), nl.Heartbeat.CRC_EXTRA,
                            seq=0, sysid=1, compid=1, truncate=False)
        self.assertEqual(frame[0], 0x56)
        self.assertEqual(frame[1], 0x02)
        self.assertEqual(frame[3], 0x00)                 # incompat
        self.assertEqual(frame[4], 0x00)                 # seq
        self.assertEqual(frame[5], 0x01)                 # sysid
        self.assertEqual(frame[6], 0x01)                 # compid
        self.assertEqual(frame[7:10], b"\x00\x00\x00")   # msgid 0
        ok, msgid, payload = verify_frame(frame, nl.Heartbeat.CRC_EXTRA)
        self.assertTrue(ok)
        self.assertEqual(msgid, 0)
        self.assertEqual(nl.Heartbeat.unpack(payload).capabilities, 0xABCD)

    def test_truncation_round_trip(self):                # §16.3 step 3 / §5.6
        hb = nl.Heartbeat(type=1)                        # everything else 0 → trailing zeros
        full = hb.pack()
        frame = build_frame(nl.Heartbeat.MSGID, full, nl.Heartbeat.CRC_EXTRA)
        self.assertLess(frame[2], len(full))             # payload was shortened
        ok, msgid, payload = verify_frame(frame, nl.Heartbeat.CRC_EXTRA)
        self.assertTrue(ok)
        hb2 = nl.Heartbeat.unpack(payload)               # unpack zero-fills (§5.6)
        self.assertEqual(hb2.type, 1)
        self.assertEqual(hb2.timestamp, 0)

    def test_crc_detects_corruption(self):
        frame = bytearray(build_frame(nl.AttitudeEuler.MSGID,
                                      nl.AttitudeEuler(roll=1.0).pack(),
                                      nl.AttitudeEuler.CRC_EXTRA))
        frame[11] ^= 0xFF                                # flip a payload byte
        ok, _, _ = verify_frame(bytes(frame), nl.AttitudeEuler.CRC_EXTRA)
        self.assertFalse(ok)

    def test_version_demux(self):                        # §16.5
        self.assertEqual(demux_version(0x56, 0x31), "v1")
        self.assertEqual(demux_version(0x56, 0x02), "v2")
        self.assertEqual(demux_version(0x56, 0x07), "resync")
        self.assertEqual(demux_version(0x55, 0x02), "resync")

    def test_all_messages_frame_round_trip(self):
        """Every message survives frame assembly → CRC verify → unpack (with §5.6
        truncation), across every field type."""
        for msgid, cls in nl.MSGID_TO_CLASS.items():
            with self.subTest(msg=cls.__name__):
                payload = build_canonical(cls, BY_ID[msgid]).pack()
                frame = build_frame(msgid, payload, cls.CRC_EXTRA, seq=msgid & 0xFF)
                ok, mid, rx = verify_frame(frame, cls.CRC_EXTRA)
                self.assertTrue(ok)
                self.assertEqual(mid, msgid)
                # unpack zero-fills the truncated tail back to the full payload
                self.assertEqual(cls.unpack(rx).pack(), cls.unpack(payload).pack())

    def test_unknown_msgid_crc_mismatch(self):
        # A frame whose CRC_EXTRA the receiver doesn't know fails the check.
        frame = build_frame(nl.Heartbeat.MSGID, nl.Heartbeat(type=3).pack(),
                            nl.Heartbeat.CRC_EXTRA)
        ok, _, _ = verify_frame(frame, crc_extra=(nl.Heartbeat.CRC_EXTRA ^ 0x01))
        self.assertFalse(ok)


class TestDialectValidation(unittest.TestCase):
    def test_clean_dialect_passes(self):
        self.assertEqual(generate.validate(DIALECT), [])

    def test_duplicate_msgid_rejected(self):
        bad = {"messages": [
            {"msgid": 1, "name": "A", "fields": [{"index": 0, "name": "x", "type": "u8"}]},
            {"msgid": 1, "name": "B", "fields": [{"index": 0, "name": "x", "type": "u8"}]},
        ]}
        self.assertTrue(any("duplicate msgid" in e for e in generate.validate(bad)))

    def test_noncontiguous_indices_rejected(self):
        bad = {"messages": [
            {"msgid": 1, "name": "A",
             "fields": [{"index": 0, "name": "x", "type": "u8"},
                        {"index": 2, "name": "y", "type": "u8"}]},
        ]}
        self.assertTrue(any("contiguous" in e for e in generate.validate(bad)))

    def test_vendor_msgid_rejected(self):
        bad = {"messages": [
            {"msgid": 0x800000, "name": "V",
             "fields": [{"index": 0, "name": "x", "type": "u8"}]},
        ]}
        self.assertTrue(any("core half" in e for e in generate.validate(bad)))

    def test_oversize_payload_rejected(self):
        bad = {"messages": [
            {"msgid": 1, "name": "BIG",
             "fields": [{"index": 0, "name": "x", "type": "u8", "len": 256}]},
        ]}
        self.assertTrue(any("> 255" in e for e in generate.validate(bad)))

    def test_unknown_type_rejected(self):
        bad = {"messages": [
            {"msgid": 1, "name": "A", "fields": [{"index": 0, "name": "x", "type": "f128"}]},
        ]}
        self.assertTrue(any("unknown type" in e for e in generate.validate(bad)))

    def test_unknown_enum_rejected(self):
        bad = {"messages": [
            {"msgid": 1, "name": "A",
             "fields": [{"index": 0, "name": "x", "type": "u8", "enum": "nope"}]},
        ]}
        self.assertTrue(any("unknown enum" in e for e in generate.validate(bad)))

    def test_duplicate_field_name_rejected(self):
        bad = {"messages": [
            {"msgid": 1, "name": "A",
             "fields": [{"index": 0, "name": "x", "type": "u8"},
                        {"index": 1, "name": "x", "type": "u8"}]},
        ]}
        self.assertTrue(any("duplicate field name" in e for e in generate.validate(bad)))

    def test_duplicate_message_name_rejected(self):
        bad = {"messages": [
            {"msgid": 1, "name": "A", "fields": [{"index": 0, "name": "x", "type": "u8"}]},
            {"msgid": 2, "name": "A", "fields": [{"index": 0, "name": "x", "type": "u8"}]},
        ]}
        self.assertTrue(any("duplicate message name" in e for e in generate.validate(bad)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
