#!/usr/bin/env python3
"""NavLink v2 test runner.

Pipeline:
  1. regenerate codecs from dialect.json
  2. run the Python unit tests (tests/test_codec.py)
  3. compile + run the C test (tests/test_c.c)
  4. assert C and Python agree: CRC_EXTRA + wire_size for every message, and
     packed bytes for fixed-value vectors

Exits non-zero on any failure. No third-party deps; needs a C compiler (cc/gcc).
"""
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
GEN_C = os.path.join(ROOT, "generated", "c")
GEN_PY = os.path.join(ROOT, "generated", "python")


def step(msg):
    print(f"\n=== {msg} ===")


def run(cmd, **kw):
    print("$", " ".join(cmd))
    return subprocess.run(cmd, **kw)


def main():
    ok = True

    step("1. regenerate from dialect.json")
    r = run([sys.executable, os.path.join(ROOT, "generate.py")])
    if r.returncode != 0:
        sys.exit("generation failed")

    step("2. Python unit tests")
    r = run([sys.executable, os.path.join(HERE, "test_codec.py")])
    ok &= r.returncode == 0

    step("3. compile + run C test")
    cc = os.environ.get("CC") or shutil.which("cc") or shutil.which("gcc")
    if not cc:
        sys.exit("no C compiler (cc/gcc) found")
    exe = os.path.join(HERE, "_test_c.bin")
    r = run([cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", GEN_C,
             os.path.join(HERE, "test_c.c"), os.path.join(GEN_C, "navlink_msgs.c"),
             os.path.join(GEN_C, "navlink_parity.c"), "-o", exe])
    if r.returncode != 0:
        sys.exit("C compile failed")
    r = run([exe], capture_output=True, text=True)
    sys.stderr.write(r.stderr)
    if r.returncode != 0:
        sys.exit("C test asserts failed")
    c_lines = r.stdout.splitlines()

    step("4. cross-language parity")
    sys.path.insert(0, ROOT)        # generate.py
    sys.path.insert(0, GEN_PY)      # navlink_msgs.py
    import json
    import generate
    import navlink_msgs as nl
    with open(os.path.join(ROOT, "dialect.json")) as fh:
        dialect = json.load(fh)
    by_id = {m["msgid"]: m for m in dialect["messages"]}

    c_table = {}   # msgid -> (crc_extra, wire_size)
    c_bytes = {}   # name -> payload hex
    c_frames = {}  # name -> full-frame hex
    for ln in c_lines:
        parts = ln.split()
        if parts[0] == "TABLE":
            c_table[int(parts[1])] = (int(parts[2]), int(parts[3]))
        elif parts[0] == "BYTES":
            c_bytes[parts[1]] = parts[2]
        elif parts[0] == "FRAME":
            c_frames[parts[1]] = parts[2]

    # 4a. table parity for every message
    mismatches = 0
    for msgid, cls in nl.MSGID_TO_CLASS.items():
        cce, cws = c_table.get(msgid, (None, None))
        if (cce, cws) != (cls.CRC_EXTRA, cls.WIRE_SIZE):
            print(f"  MISMATCH {cls.__name__} (msgid {msgid}): "
                  f"C=({cce},{cws}) Py=({cls.CRC_EXTRA},{cls.WIRE_SIZE})")
            mismatches += 1
    if len(c_table) != len(nl.MSGID_TO_CLASS):
        print(f"  MISMATCH message count: C={len(c_table)} Py={len(nl.MSGID_TO_CLASS)}")
        mismatches += 1
    if mismatches == 0:
        print(f"  table parity OK for {len(c_table)} messages (CRC_EXTRA + wire_size)")
    ok &= mismatches == 0

    # 4b. byte-level parity for EVERY message, using the shared canonical values
    byte_mismatch = 0
    for msgid, cls in nl.MSGID_TO_CLASS.items():
        obj = cls()
        for (name, _, _), v in zip(cls._FIELDS, generate.canonical_values(by_id[msgid])):
            setattr(obj, name, v)
        py_hex = obj.pack().hex()
        nm = by_id[msgid]["name"]
        if py_hex != c_bytes.get(nm):
            print(f"  BYTE MISMATCH {nm}:\n    C ={c_bytes.get(nm)}\n    Py={py_hex}")
            byte_mismatch += 1
    if byte_mismatch == 0:
        print(f"  byte parity OK for {len(nl.MSGID_TO_CLASS)} messages (canonical values)")
    ok &= byte_mismatch == 0

    # 4c. full-frame parity (encoder): C navlink_*_encode vs Python nl.encode, seq=7
    frame_mismatch = 0
    for msgid, cls in nl.MSGID_TO_CLASS.items():
        obj = cls()
        for (name, _, _), v in zip(cls._FIELDS, generate.canonical_values(by_id[msgid])):
            setattr(obj, name, v)
        py_hex = nl.encode(obj, seq=7, sysid=1, compid=1).hex()
        nm = by_id[msgid]["name"]
        if py_hex != c_frames.get(nm):
            print(f"  FRAME MISMATCH {nm}:\n    C ={c_frames.get(nm)}\n    Py={py_hex}")
            frame_mismatch += 1
    if frame_mismatch == 0:
        print(f"  frame parity OK for {len(nl.MSGID_TO_CLASS)} messages (encoder)")
    ok &= frame_mismatch == 0

    os.path.exists(exe) and os.remove(exe)

    step("5. simulator smoke (2s clean link)")
    r = run([sys.executable, os.path.join(ROOT, "sim", "sim.py"),
             "--duration", "2", "--seed", "1", "--json"], capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stderr)
        print("  sim failed to run")
        ok = False
    else:
        rep = json.loads(r.stdout)
        g = rep.get("gcs") or {}
        sim_ok = (g.get("rx_frames", 0) > 50 and g.get("est_loss_pct") == 0.0
                  and g.get("crc_errors") == 0 and g["ping"]["count"] > 0)
        print(f"  rx_frames={g.get('rx_frames')} loss={g.get('est_loss_pct')}% "
              f"crc_errors={g.get('crc_errors')} ping={g.get('ping', {}).get('count')} -> "
              f"{'OK' if sim_ok else 'FAIL'}")
        ok &= sim_ok

    step("6. xfer loopback (Python client <-> server, real frames)")
    r = run([sys.executable, os.path.join(HERE, "test_xfer_loopback.py")],
            capture_output=True, text=True)
    sys.stdout.write(r.stdout)
    if r.returncode != 0:
        sys.stderr.write(r.stderr)
        ok = False

    step("RESULT")
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
