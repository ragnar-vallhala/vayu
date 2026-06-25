"""FC telemetry decode — thin wrapper over the generated NavLink codec.

The codec is the single source of truth (navlink/generated/python +
navlink/sim). We locate the repo root by walking up from this file and add
those dirs to sys.path, then expose:
  - nlframe        : the navlink/sim/frame.py module (SYNC, HDR_LEN, decode)
  - MSG_BY_ID      : msgid -> message class
  - parse_frames() : consume a byte buffer, update {name: msg} + counts
"""
import os
import sys


def _find_repo_root():
    # Prefer an explicit override; else walk up to the dir holding navlink/.
    env = os.environ.get("VAYU_REPO_ROOT")
    if env and os.path.isdir(os.path.join(env, "navlink", "generated", "python")):
        return env
    d = os.path.abspath(os.path.dirname(__file__))
    while True:
        if os.path.isdir(os.path.join(d, "navlink", "generated", "python")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            raise RuntimeError("could not locate repo root (navlink/ not found)")
        d = parent


_ROOT = _find_repo_root()
for _p in (os.path.join(_ROOT, "navlink", "sim"),
           os.path.join(_ROOT, "navlink", "generated", "python")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import frame as nlframe          # noqa: E402  navlink/sim/frame.py
import navlink_msgs as nlmsg     # noqa: E402  generated codec

# msgid -> message class (for decoding telemetry payloads).
MSG_BY_ID = {c.MSGID: c for c in vars(nlmsg).values()
             if isinstance(c, type) and hasattr(c, "MSGID") and hasattr(c, "unpack")}


def parse_frames(buf, telem, telem_counts):
    """Consume complete NavLink frames from `buf` (bytearray, trimmed in place),
    decoding each into `telem[name] = msg` and bumping `telem_counts[name]`.
    Identical resync/framing behaviour to the original SitlLab._parse_frames."""
    i = 0
    while i < len(buf):
        if buf[i] != nlframe.SYNC:
            i += 1
            continue
        if i + nlframe.HDR_LEN + 2 > len(buf):
            break
        plen = buf[i + 2]
        total = nlframe.HDR_LEN + plen + 2
        if i + total > len(buf):
            break
        d = nlframe.decode(bytes(buf[i:i + total]))
        if d.ok:
            cls = MSG_BY_ID.get(d.msgid)
            if cls:
                name = cls.__name__
                telem[name] = cls.unpack(d.payload)
                telem_counts[name] = telem_counts.get(name, 0) + 1
            i += total
        else:
            i += 1                # resync on the next SYNC byte
    del buf[:i]
