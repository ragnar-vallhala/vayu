#!/usr/bin/env python3
"""
ibus_inject.py — host-side bridge that pushes FlySky iBus frames into the
emulated USART6 of a running Renode `vayu` machine.

Wire-up
    Renode exposes USART6 as the host PTY /tmp/vayu_usart6.pts (set up
    by tools/sim_renode/vayu.resc). When this script writes bytes to
    that PTY, they appear on the firmware's USART6 RX, get DMA'd into
    the rc_ibus_task's circular buffer, and feed the real ibus_parse
    state machine in src/comm/ibus.c — same code path as on metal
    against an FS-iA6B receiver.

Input sources
    --source=/dev/ttyUSB0   read sim_bridge's CSV output (one
                            "ch1,ch2,...,chN" line per frame)
    --source=test           generate a synthetic stick sweep without
                            needing the Arduino in the loop. Useful for
                            CI / when the FS-i6 transmitter is off.

Frame format (per FlySky iBus servo packet)
    0x20  length = 32 bytes
    0x40  command = servo data
    bytes 2..29   14 channels, little-endian uint16, typ. 1000..2000 us
    bytes 30..31  LE checksum = 0xFFFF - sum(bytes 0..29)

Default rate: 50 Hz (140 Hz on a real iA6B, but vayu's parser tolerates
either; lower rate keeps the host log readable).
"""
from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import time
from pathlib import Path

IBUS_START = 0x20
IBUS_CMD_CHANNELS = 0x40
IBUS_MAX_CHANNELS = 14
IBUS_PACKET_SIZE = 32   # 2 header + 28 channel data + 2 checksum
DEFAULT_PTY = "/tmp/vayu_usart6.fifo"
DEFAULT_RATE_HZ = 50


def encode_frame(channels: list[int]) -> bytes:
    """Build a 32-byte iBus servo packet from up to 14 channel values."""
    chs = list(channels[:IBUS_MAX_CHANNELS])
    chs += [1500] * (IBUS_MAX_CHANNELS - len(chs))   # pad unused to centre

    # Clamp into the iBus range. iBus pulses are unsigned 16-bit; FS-i6
    # endpoints are ~1000..2000 but the protocol allows the full range.
    chs = [max(0, min(0xFFFF, int(c))) for c in chs]

    body = bytes([IBUS_START, IBUS_CMD_CHANNELS])
    body += b"".join(struct.pack("<H", c) for c in chs)
    assert len(body) == 30, len(body)

    checksum = (0xFFFF - sum(body)) & 0xFFFF
    return body + struct.pack("<H", checksum)


# ---- input sources -------------------------------------------------------

def source_serial(path: str):
    """Yield channel-value lists, one per sim_bridge CSV frame."""
    import serial            # pyserial; only required for the --source=<dev>
    port = serial.Serial(path, 115200, timeout=1.0)
    while True:
        raw = port.readline()
        if not raw:
            continue
        line = raw.decode(errors="ignore").strip()
        if not line or line.startswith("#") or line == "NO_SIGNAL":
            continue
        try:
            yield [int(x) for x in line.split(",")]
        except ValueError:
            continue


def source_test():
    """A predictable synthetic sweep: throttle ramps 1000→2000 over 4 s,
    roll and pitch make a slow circle, yaw stays centred, CH5 (arm) high."""
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        roll  = 1500 + int(400 * math.sin(2 * math.pi * 0.25 * t))
        pitch = 1500 + int(400 * math.cos(2 * math.pi * 0.25 * t))
        thr   = 1000 + int(500 + 500 * math.sin(2 * math.pi * 0.125 * t))
        yaw   = 1500
        sw_a  = 2000   # arm
        sw_b  = 1000
        yield [roll, pitch, thr, yaw, sw_a, sw_b]


def source_arm_seq():
    """Arming-aware stick sequence for the M3 closed-loop smoke test.

    Phases (sw_a = CH5 = arm switch, throttle = CH3):
        0.0–5.0 s    disarmed     sw_a=1000, thr=1000  -> vayu in STANDBY
        5.0–10.0 s   arm @ low thr sw_a=2000, thr=1000 -> STANDBY -> ARMED
        10.0–15.0 s  throttle ramp thr 1000 -> 1300 linear
        15.0+        hold          sw_a=2000, thr=1300

    The long disarm + arm-at-idle phases give vayu's iBus parser plenty
    of time to land at least one frame in each phase, which avoids a
    race where the first parsed packet sees sw_a high + throttle already
    above the 1100 arming threshold (-> rc_task transitions STANDBY ->
    FAILSAFE instead of ARMED).

    Roll / pitch / yaw stay centred so the controller has zero attitude
    setpoint — any motor reaction is the loop closing, not a stick command.
    """
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        sw_b  = 1000
        yaw   = 1500
        roll  = 1500
        pitch = 1500
        if t < 5.0:
            sw_a = 1000
            thr  = 1000
        elif t < 10.0:
            sw_a = 2000
            thr  = 1000
        elif t < 15.0:
            sw_a = 2000
            thr  = 1000 + int(60 * (t - 10.0))  # 1000 -> 1300
        else:
            sw_a = 2000
            thr  = 1300
        yield [roll, pitch, thr, yaw, sw_a, sw_b]


# ---- main loop -----------------------------------------------------------

def run(pty_path: str, source, rate_hz: float, verbose: bool) -> None:
    period = 1.0 / rate_hz
    # The target may be a PTY (Renode CreateUartPtyTerminal) or a FIFO
    # (tools/sim_renode/usart6_mock.py). For PTY, O_RDWR is fine; for a
    # FIFO, O_WRONLY is the right choice — and opening it blocks until
    # the reader (the Renode peripheral) opens the read end.
    import stat
    mode = os.stat(pty_path).st_mode
    if stat.S_ISFIFO(mode):
        fd = os.open(pty_path, os.O_WRONLY)
    else:
        fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
    try:
        last_log = 0.0
        next_send = time.monotonic()
        current = [1500, 1500, 1000, 1500, 1000, 1000]
        gen = iter(source)

        while True:
            # Drain new channels non-blockingly (whichever is most recent wins).
            try:
                current = next(gen)
            except StopIteration:
                pass

            frame = encode_frame(current)
            os.write(fd, frame)

            now = time.monotonic()
            if verbose and now - last_log >= 1.0:
                last_log = now
                summary = " ".join(f"{c:>4d}" for c in current[:8])
                print(f"sent {len(frame)}B  ch1-8: {summary}", flush=True)

            next_send += period
            sleep_for = next_send - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_send = time.monotonic()       # drifted; resync
    finally:
        os.close(fd)


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pty", default=DEFAULT_PTY,
                    help=f"path to Renode USART6 PTY (default {DEFAULT_PTY})")
    ap.add_argument("--source", default="test",
                    help='"test" for synthetic stick sweep, or a serial device '
                         'path like /dev/ttyUSB0 to read sim_bridge CSV')
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ,
                    help=f"frames per second (default {DEFAULT_RATE_HZ})")
    ap.add_argument("--quiet", action="store_true",
                    help="suppress 1 Hz progress prints")
    args = ap.parse_args(argv)

    if not Path(args.pty).exists():
        print(f"ibus_inject: {args.pty} does not exist — is Renode running with "
              f"tools/sim_renode/vayu.resc?", file=sys.stderr)
        return 2

    if args.source == "test":
        src = source_test()
    elif args.source == "arm":
        src = source_arm_seq()
    else:
        src = source_serial(args.source)

    try:
        run(args.pty, src, args.rate, verbose=not args.quiet)
    except KeyboardInterrupt:
        print("\nibus_inject: stopped", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
