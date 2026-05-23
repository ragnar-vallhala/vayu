#!/usr/bin/env python3
"""
sim_rc_inject.py - Drive vayu's RC input by writing sim_rc_channels[]
directly into RAM via the Renode monitor.

Bypasses the USART6/DMA/ibus-parser pipeline (which has a half-of-each-
frame-dropped bug in the current usart6_mock implementation). Vayu's
rc_ibus_task under VAYU_SIM checks `sim_rc_enabled` first and uses
`sim_rc_channels[]` when set, so a host-side write to those globals
substitutes for a real iBus stream.

Symbol addresses (re-check after every rebuild with
`arm-none-eabi-readelf -s build/main | grep sim_rc_`):
    sim_rc_enabled   @ 0x20001a30  (uint8)
    sim_rc_channels  @ 0x20000550  (14 * uint16, little-endian)
    _system_current_status @ 0x2000070e (uint16-ish)

Usage:
    python3 tools/sim_renode/sim_rc_inject.py [--cmd-fifo /tmp/renode_cmd.fifo]
                                              [--source arm]

Talks to Renode through its monitor command FIFO (the same one your
boot script feeds `include @tools/sim_renode/vayu.resc; start` into).
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
import time


# ----- defaults (verify after every firmware rebuild) ------------------
DEFAULT_ENABLED_ADDR  = 0x20001a30
DEFAULT_CHANNELS_ADDR = 0x20000550
DEFAULT_STATE_ADDR    = 0x2000070e
DEFAULT_FIFO          = "/tmp/renode_cmd.fifo"

STATE_NAMES = {
    0x1: "UNINITIALIZED", 0x2: "INIT", 0x4: "STANDBY", 0x8: "PREARM",
    0x10: "ARMED", 0x20: "IN_AIR", 0x40: "FAILSAFE", 0x80: "TERMINATED",
    0x100: "CALIBRATING",
}


def source_arm_seq():
    """Phases:
        0..3 s    disarmed (sw_a=1000), thr=1000
        3..6 s    armed @ low thr (sw_a=2000, thr=1000) -> STANDBY -> ARMED
        6..11 s   throttle ramp 1000 -> 1300
        11+       hold thr=1300
    """
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        roll = pitch = yaw = 1500
        sw_b = 1000
        if t < 3.0:
            sw_a, thr = 1000, 1000
        elif t < 6.0:
            sw_a, thr = 2000, 1000
        elif t < 11.0:
            sw_a, thr = 2000, 1000 + int(60 * (t - 6.0))
        else:
            sw_a, thr = 2000, 1300
        # pad to 14 channels (the firmware reads all 14)
        yield [roll, pitch, thr, yaw, sw_a, sw_b] + [1500] * 8


def write_channels(fifo, addr, channels):
    """Write 14 LE u16 channels (= 28 bytes) starting at addr."""
    pkt = b"".join(struct.pack("<H", c) for c in channels[:14])
    lines = []
    # Renode's sysbus has WriteWord for u16. Write each channel as a
    # single Word write — fewer monitor commands than per-byte writes.
    for i in range(14):
        c = channels[i]
        lines.append("sysbus WriteWord 0x%X 0x%X" % (addr + 2 * i, c & 0xFFFF))
    fifo.write("\n".join(lines) + "\n")
    fifo.flush()


def enable_sim_rc(fifo, addr):
    fifo.write("sysbus WriteByte 0x%X 0x01\n" % addr)
    fifo.flush()


def read_state(fifo, addr):
    """Schedule a state read; the result lands in renode_stdout.log."""
    fifo.write("sysbus ReadWord 0x%X\n" % addr)
    fifo.flush()


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cmd-fifo",       default=DEFAULT_FIFO)
    ap.add_argument("--enabled-addr",   type=lambda s: int(s, 0),
                    default=DEFAULT_ENABLED_ADDR)
    ap.add_argument("--channels-addr",  type=lambda s: int(s, 0),
                    default=DEFAULT_CHANNELS_ADDR)
    ap.add_argument("--state-addr",     type=lambda s: int(s, 0),
                    default=DEFAULT_STATE_ADDR)
    ap.add_argument("--source",         default="arm", choices=("arm",))
    ap.add_argument("--rate",           type=float, default=20.0,
                    help="channel write rate (Hz)")
    args = ap.parse_args(argv)

    if not os.path.exists(args.cmd_fifo):
        print("ERR: cmd FIFO %s does not exist (is renode running with "
              "stdin pointed at this fifo?)" % args.cmd_fifo, file=sys.stderr)
        return 2

    gen = iter(source_arm_seq())
    period = 1.0 / args.rate
    last_print = 0.0
    start = time.monotonic()

    with open(args.cmd_fifo, "w", buffering=1) as fifo:
        enable_sim_rc(fifo, args.enabled_addr)
        print("sim_rc_inject: enabled @ 0x%X" % args.enabled_addr,
              flush=True)
        while True:
            channels = next(gen)
            write_channels(fifo, args.channels_addr, channels)

            now = time.monotonic()
            if now - last_print >= 1.0:
                last_print = now
                read_state(fifo, args.state_addr)
                print("t=%.1fs  ch1-6: %s"
                      % (now - start, channels[:6]), flush=True)

            time.sleep(period)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        print("\nsim_rc_inject: stopped", file=sys.stderr)
        sys.exit(0)
