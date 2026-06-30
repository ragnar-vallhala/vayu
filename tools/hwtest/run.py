#!/usr/bin/env python3
"""run.py — on-hardware bench (Suite B) host runner.

Builds the hwtest firmware, flashes it, opens the live link, collects the
NavLink test-dialect results (HW_TEST_BEGIN / RESULT / DONE), and prints a
pytest-style summary so `vayu.sh` can parse it into the unified table.

Auto-skips cleanly when no FC is configured/reachable (CI and laptop runs stay
green): with no transport set it prints "0 passed, 0 failed, N skipped" and
exits 0. Flashing is gated behind VAYU_HW_ALLOW_FLASH=1 so an attached board is
never reflashed unexpectedly.

Env:
    VAYU_FC_TRANSPORT = udp | serial   (unset => skip)
    VAYU_FC_UDP_PORT  = 14555
    VAYU_FC_SERIAL    = /dev/ttyUSB0
    VAYU_FC_BAUD      = 460800
    VAYU_HW_ALLOW_FLASH = 1            (required to flash)
    VAYU_HW_PROPS_OFF   = 1            (required before any PWM/ESC check; C2+)

See firmware/docs/plans/on-hardware-test-and-coverage.md (C1).
"""
import argparse
import os
import struct
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BUILD = REPO / "firmware" / "build_hwtest"

# Test-dialect msgids (vendor/test half; must match navlink/dialect.json).
HW_TEST_BEGIN = 0x800000
HW_TEST_RESULT = 0x800001
HW_TEST_DONE = 0x800002


# ---------------------------------------------------------------------------
# Build / flash
# ---------------------------------------------------------------------------
def build() -> bool:
    print("[hwtest] configuring + building hwtest firmware...")
    cfg = subprocess.run(
        ["cmake", "-S", str(REPO / "firmware"), "-B", str(BUILD),
         "-DVAYU_HW_TEST=ON", "-DNAVHAL=ON", "-DEXTERNAL_LINKER=ON"],
        capture_output=True, text=True)
    if cfg.returncode != 0:
        print(cfg.stdout[-2000:], cfg.stderr[-2000:], file=sys.stderr)
        return False
    bld = subprocess.run(["cmake", "--build", str(BUILD), "--target", "main", "-j"],
                         capture_output=True, text=True)
    if bld.returncode != 0:
        print(bld.stdout[-2000:], bld.stderr[-2000:], file=sys.stderr)
        return False
    return True


def flash() -> bool:
    if os.environ.get("VAYU_HW_ALLOW_FLASH") != "1":
        print("[hwtest] VAYU_HW_ALLOW_FLASH != 1 — not flashing (using whatever "
              "is on the board).")
        return True
    elf = BUILD / "hwtest"
    binf = BUILD / "hwtest.bin"
    subprocess.run(["arm-none-eabi-objcopy", "-O", "binary", str(elf), str(binf)],
                   check=True)
    print("[hwtest] flashing hwtest.bin...")
    rc = subprocess.run(["st-flash", "--connect-under-reset", "write",
                         str(binf), "0x08000000"]).returncode
    return rc == 0


# ---------------------------------------------------------------------------
# Transport + collect  (decode HW_TEST_* straight from the frame payloads)
# ---------------------------------------------------------------------------
def fc_available() -> bool:
    return os.environ.get("VAYU_FC_TRANSPORT") in ("udp", "serial")


def _iter_frames(transport):
    """Yield (msgid, payload) from the live link. Minimal NavLink v2 framing:
    sync(0x56) ver len flags seq sysid compid msgid[3] payload crc16."""
    buf = bytearray()
    for chunk in transport:
        buf += chunk
        while True:
            i = buf.find(0x56)
            if i < 0:
                buf.clear()
                break
            if len(buf) - i < 10:
                if i:
                    del buf[:i]
                break
            ln = buf[i + 2]
            total = 10 + ln + 2
            if len(buf) - i < total:
                if i:
                    del buf[:i]
                break
            frame = bytes(buf[i:i + total])
            del buf[:i + total]
            msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
            yield msgid, frame[10:10 + ln]


def collect(transport, timeout=30.0):
    """Drive _iter_frames until HW_TEST_DONE or timeout. Returns (p, f, s)."""
    passed = failed = skipped = 0
    deadline = time.time() + timeout
    for msgid, pl in transport:
        if time.time() > deadline:
            print("[hwtest] timeout waiting for HW_TEST_DONE", file=sys.stderr)
            break
        if msgid == HW_TEST_BEGIN and len(pl) >= 2:
            (total,) = struct.unpack_from("<H", pl, 0)
            print(f"[hwtest] suite begin: {total} checks")
        elif msgid == HW_TEST_RESULT and len(pl) >= 39:
            cid, name, ok, val = struct.unpack_from("<H24sB f", pl, 0)
            name = name.split(b"\0")[0].decode("ascii", "replace")
            tag = {1: "PASS", 0: "FAIL", 2: "SKIP"}.get(ok, "?")
            print(f"  [{tag}] {name:<22} {val:g}")
        elif msgid == HW_TEST_DONE and len(pl) >= 6:
            passed, failed, skipped = struct.unpack_from("<HHH", pl, 0)
            break
    return passed, failed, skipped


def open_transport():
    """Return an iterator of byte chunks for the configured transport, or None.
    Kept minimal here; the shared vayu_headless.transport.live seam (UDP de-
    coalesce + serial) is the C2 home for this."""
    kind = os.environ.get("VAYU_FC_TRANSPORT")
    if kind == "udp":
        import socket
        port = int(os.environ.get("VAYU_FC_UDP_PORT", "14555"))
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind(("", port))
        s.sendto(b"GCS-HELLO", ("255.255.255.255", port))
        s.settimeout(2.0)

        def gen():
            while True:
                try:
                    yield s.recvfrom(2048)[0]
                except socket.timeout:
                    yield b""
        return gen()
    if kind == "serial":
        import serial  # type: ignore
        ser = serial.Serial(os.environ["VAYU_FC_SERIAL"],
                            int(os.environ.get("VAYU_FC_BAUD", "460800")),
                            timeout=1.0)
        return iter(lambda: ser.read(256), b"")
    return None


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description="on-hardware bench runner")
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--timeout", type=float, default=30.0)
    args = ap.parse_args()

    if not fc_available():
        # Count the checks in the registry so the skip total is honest.
        reg = (REPO / "firmware" / "tests" / "onboard" / "hwtest_main.c").read_text()
        n = reg.count("{\"") if "hwtest_registry" in reg else 0
        print("[hwtest] no FC configured (VAYU_FC_TRANSPORT unset) — skipping.")
        print(f"0 passed, 0 failed, {max(n,1)} skipped")
        return 0

    if not args.no_build and not build():
        print("[hwtest] build failed", file=sys.stderr)
        return 1
    if not flash():
        print("[hwtest] flash failed", file=sys.stderr)
        return 1

    transport = _iter_frames(open_transport())
    p, f, s = collect(transport, timeout=args.timeout)
    print(f"{p} passed, {f} failed, {s} skipped")
    return 1 if f else 0


if __name__ == "__main__":
    sys.exit(main())
