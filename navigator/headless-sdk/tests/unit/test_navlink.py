"""Unit tests for telemetry decode (uses the generated codec, no daemon)."""
from vayu_headless.transport import navlink


def test_codec_loaded():
    assert navlink.MSG_BY_ID, "no NavLink message classes discovered"
    # Heartbeat (msgid 0) must be present — the harness relies on nav_state.
    assert 0 in navlink.MSG_BY_ID


def _heartbeat_frame():
    """A full on-wire Heartbeat frame via the generated codec (class.pack() +
    frame.encode), matching exactly what the FC emits."""
    hb_cls = navlink.MSG_BY_ID[0]
    msg = hb_cls()
    if hasattr(msg, "nav_state"):
        msg.nav_state = 4
    return navlink.nlframe.encode(hb_cls.MSGID, msg.pack())


def test_parse_frames_decodes_encoded_message():
    telem, counts = {}, {}
    buf = bytearray(_heartbeat_frame())
    navlink.parse_frames(buf, telem, counts)
    assert counts.get("Heartbeat", 0) == 1
    assert "Heartbeat" in telem
    assert len(buf) == 0                      # frame fully consumed


def test_parse_frames_resyncs_on_garbage():
    telem, counts = {}, {}
    buf = bytearray(b"\x00\xff\x12" + bytes(_heartbeat_frame()))   # leading junk
    navlink.parse_frames(buf, telem, counts)
    assert counts.get("Heartbeat", 0) == 1               # found after resync
