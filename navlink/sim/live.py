#!/usr/bin/env python3
"""Interactive NavLink v2 link console.

Runs an FC model and a GCS model (as threads, sharing the live link config so
impairments take effect immediately), then drops you at a prompt to tune the link
and inject commands while watching real-world stats update.

    python3 sim/live.py                 # start clean, then type `help`
    python3 sim/live.py --scenario lossy

At the `nlctl>` prompt:
A live colour dashboard pinned to the bottom of the terminal updates on its own.
Use ↑/↓ at the prompt to recall previous commands.

    status                 print a full stats snapshot inline
    clear                  wipe the command output and redraw the dashboard
    set [up|down] K V      tune a knob live: latency jitter loss dup reorder corrupt rate
    scenario NAME          apply a preset (clean/wifi/telemetry_radio/lossy/satellite)
    reset                  clear all impairments
    tx                     checklist — ↑/↓ move, space toggle, type digits set rate
                           (Backspace edits), ←/→ step rate, Enter apply, q cancel
    tx list                show telemetry streams (rate / on-off / sent)
    tx on|off <name|all>   enable/disable a stream (name match is a substring)
    tx rate <name|all> HZ  set a stream's rate in Hz
    arm | disarm | ping    inject a command / probe now
    pid KP KI KD           send a CMD_SET_PID
    quit
"""
import argparse
import os
import re
import select
import shutil
import socket
import sys
import termios
import threading
import time
import tty
from collections import deque

try:
    import readline  # noqa: F401  — enables ↑/↓ history + line editing in input()
except ImportError:
    readline = None

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import load_nl              # noqa: E402
from endpoint import FC, GCS, FC_SYS, COMP, udp_socket  # noqa: E402
from link import Link, LinkConfig       # noqa: E402
from sim import SCENARIOS                # noqa: E402

nl = load_nl()

KNOBS = {"latency": "latency_ms", "jitter": "jitter_ms", "loss": "loss",
         "dup": "dup", "reorder": "reorder", "corrupt": "corrupt", "rate": "rate_bytes"}


def free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def apply_scenario(cfg, name):
    preset = SCENARIOS[name]
    for field in ("latency_ms", "jitter_ms", "loss", "dup", "reorder", "corrupt", "rate_bytes"):
        setattr(cfg, field, preset.get(field, 0))


def cfg_str(cfg):
    on = {k: getattr(cfg, v) for k, v in KNOBS.items() if getattr(cfg, v)}
    return str(on) if on else "clean"


def cfg_compact(cfg):
    short = [("lat", cfg.latency_ms), ("jit", cfg.jitter_ms), ("loss", cfg.loss),
             ("dup", cfg.dup), ("reord", cfg.reorder), ("cor", cfg.corrupt),
             ("rate", cfg.rate_bytes)]
    parts = [f"{k}{v:g}" for k, v in short if v]
    return ",".join(parts) or "clean"


# ── colour + sparkline helpers ────────────────────────────────────────────────
RE_ANSI = re.compile(r"\x1b\[[0-9;]*m")
BLOCKS = "▁▂▃▄▅▆▇█"


def c(s, code):
    return f"\033[{code}m{s}\033[0m"


def vis_len(s):
    return len(RE_ANSI.sub("", s))


def fit(s, cols):
    """Pad/truncate to `cols` visible columns, ignoring ANSI codes."""
    if vis_len(s) <= cols:
        return s + " " * (cols - vis_len(s))
    out, n, i = [], 0, 0
    while i < len(s) and n < cols:
        m = RE_ANSI.match(s, i)
        if m:
            out.append(m.group())
            i = m.end()
            continue
        out.append(s[i])
        n += 1
        i += 1
    return "".join(out) + "\033[0m"


def spark(vals, hi=None):
    if not vals:
        return ""
    hi = (hi if hi is not None else max(vals)) or 1.0
    return "".join(BLOCKS[max(0, min(7, int(v / hi * 7)))] for v in vals)


def col_loss(p):
    return c(f"{p:g}%", 32 if p == 0 else 33 if p <= 2 else 31)


def col_count(n, bad=31):
    return c(str(n), 2 if n == 0 else bad)


def col_rtt(ms):
    if ms is None:
        return c("--", 2)
    return c(f"{ms:.0f}ms", 32 if ms < 50 else 33 if ms < 200 else 31)


class Dashboard:
    """Renders the multi-line live panel: RX/RTT sparklines, colour-coded link
    quality, link config, and the TX stream row."""
    HEIGHT = 6

    def __init__(self, fc, gcs, down, up, scenario):
        self.fc, self.gcs, self.down, self.up, self.scenario = fc, gcs, down, up, scenario
        self.rate_hist = deque(maxlen=48)
        self._t0 = time.monotonic()
        self._last_t = None
        self._last_rx = 0

    def lines(self, cols):
        r = self.gcs.report()
        now = time.monotonic()
        dt = (now - self._last_t) if self._last_t else 0.0
        inst = (r["rx_frames"] - self._last_rx) / dt if dt > 0 else 0.0
        self._last_t, self._last_rx = now, r["rx_frames"]
        self.rate_hist.append(inst)
        pg, cm = r["ping"], r["cmd"]
        rtts = self.gcs.ping_rtts[-48:]

        hz = c(f"{r['rx_rate_hz']:.0f} Hz", "1;36")
        kbps = f"{r['rx_throughput_Bps'] / 1000:5.1f} kB/s"
        rtt_val = pg.get("avg_ms") if pg.get("count") else None

        title = ("\033[1;7m" +
                 f" NavLink live · {self.scenario} · up {now - self._t0:4.0f}s".ljust(cols)[:cols] +
                 "\033[0m")
        l_rx = (f" {c('RX','1')}  {r['rx_frames']:>6} fr  {hz}  {kbps}   "
                + c(spark(self.rate_hist), "36"))
        rtt_tail = (f"   min/max {pg['min_ms']:.0f}/{pg['max_ms']:.0f} ms  lost {pg['lost']}/{pg['sent']}"
                    if pg.get("count") else "  (no replies yet)")
        l_rtt = f" {c('RTT','1')} {col_rtt(rtt_val):>10}  " + c(spark(rtts), "35") + rtt_tail
        l_q = (f" {c('LINK','1')} loss {col_loss(r['est_loss_pct'])}  "
               f"reord {col_count(r['reorders'], 33)}  crc {col_count(r['crc_errors'])}  "
               f"undec {col_count(r['undecodable'])}   "
               f"{c('CMD','1')} {cm['acked']}/{cm['sent']} ({cm['ack_pct']:g}%)")
        l_link = f" {c('DOWN','1;34')} {cfg_compact(self.down):<34} {c('UP','1;35')} {cfg_compact(self.up)}"
        table = self.fc.stream_table()
        off = [name for name, rate, en, _ in table if not (en and rate > 0)]
        non = len(table) - len(off)
        tail = c("all enabled", "32") if not off else c("off: " + ", ".join(off), "2")
        l_tx = f" {c('TX','1')}  {non}/{len(table)} on   {tail}    {c('(tx for checklist)', '2')}"
        return [title] + [fit(x, cols) for x in (l_rx, l_rtt, l_q, l_link, l_tx)]


class StatusBar:
    """Pins a multi-line dashboard to the bottom of the terminal via an ANSI
    scroll region; the prompt/output scroll above it. Disabled when stdout is not
    a TTY or the window is too short (so pipes/tests stay clean)."""

    def __init__(self, panel, interval=0.35):
        self.panel = panel
        self.height = panel.HEIGHT
        self.interval = interval
        self.enabled = sys.stdout.isatty() and os.environ.get("TERM", "") != "dumb"
        self._stop = threading.Event()
        self._thread = None
        self._rows = 0
        self._paused = False

    def pause(self):
        self._paused = True

    def resume(self):
        self._paused = False
        self.draw()

    def start(self):
        if not self.enabled:
            return
        rows = shutil.get_terminal_size().lines
        if rows < self.height + 3:
            self.enabled = False
            return
        self._rows = rows
        sys.stdout.write(f"\033[1;{rows - self.height}r")    # region = rows 1..N-H
        sys.stdout.write(f"\033[{rows - self.height};1H")    # cursor into region
        sys.stdout.flush()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        self.draw()
        while not self._stop.wait(self.interval):
            self.draw()

    def draw(self):
        if not self.enabled or self._paused:
            return
        size = shutil.get_terminal_size()
        cols = size.columns
        out = ["\0337"]                                      # save cursor
        if size.lines != self._rows:                         # terminal was resized
            self._rows = size.lines
            out.append(f"\033[1;{self._rows - self.height}r")  # re-anchor scroll region
        lines = self.panel.lines(cols)
        top = self._rows - self.height + 1
        for i, ln in enumerate(lines):
            out.append(f"\033[{top + i};1H\033[2K")
            out.append(ln)
        out.append("\0338")                                  # restore cursor
        sys.stdout.write("".join(out))
        sys.stdout.flush()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)
        if self.enabled:
            sys.stdout.write("\033[r")                       # reset scroll region
            for i in range(self.height):
                sys.stdout.write(f"\033[{self._rows - self.height + 1 + i};1H\033[2K")
            sys.stdout.flush()


RATE_LADDER = [0, 1, 2, 5, 10, 20, 25, 50, 100, 200]


def read_key(fd):
    """Read one keypress in cbreak mode. Returns a token: up/down/left/right,
    enter, shift_enter, space, backspace, esc, or the literal character."""
    ch = os.read(fd, 1)
    if ch in (b"\x7f", b"\x08"):
        return "backspace"
    if ch == b"\x1b":                                    # ESC — maybe a CSI sequence
        if not select.select([fd], [], [], 0.06)[0]:
            return "esc"
        ch2 = os.read(fd, 1)
        if ch2 not in (b"[", b"O"):
            return "esc"
        seq = b""
        while select.select([fd], [], [], 0.02)[0]:
            b = os.read(fd, 1)
            seq += b
            if b.isalpha() or b == b"~":
                break
        s = seq.decode("latin-1", "replace")
        arrow = {"A": "up", "B": "down", "C": "right", "D": "left"}.get(s)
        if arrow:
            return arrow
        if s in ("13;2u", "27;2;13~"):                   # Shift+Enter (CSI-u / modifyOtherKeys)
            return "shift_enter"
        return "esc"
    if ch in (b"\r", b"\n"):
        return "enter"
    if ch == b" ":
        return "space"
    return ch.decode("latin-1", "replace").lower()


def tx_checklist(fc, bar):
    """Interactive stream checklist: ↑/↓ move, space tick/untick, ←/→ change rate."""
    rows = fc.stream_table()
    n = len(rows)
    if not sys.stdin.isatty() or n == 0:                 # non-tty fallback (pipes/tests)
        for i, (name, rate, en, _) in enumerate(rows, 1):
            print(f"   {i:>2} [{'x' if en and rate > 0 else ' '}] {name:<16} {rate:g} Hz")
        return

    # Edit a local working copy; nothing touches the FC until Enter (apply).
    work = [{"name": name, "rate": rate, "on": en and rate > 0}
            for name, rate, en, _ in rows]
    sel = 0
    buf = ""                                             # in-progress typed rate
    print(c("  ↑/↓ move · space toggle · type digits set rate (⌫) · ←/→ step · "
            "Enter apply · q cancel", "1"))

    def flush():
        nonlocal buf
        if buf:
            try:
                work[sel]["rate"] = float(buf)
                work[sel]["on"] = float(buf) > 0
            except ValueError:
                pass
            buf = ""

    def render():
        parts = []
        for i, w in enumerate(work):
            on = w["on"] and w["rate"] > 0
            ratecol = f"{buf}_ Hz" if (i == sel and buf) else f"{w['rate']:g} Hz"
            txt = f" {'›' if i == sel else ' '} [{'x' if on else ' '}] {w['name']:<16} {ratecol:>9}"
            parts.append("\r\033[K" + (c(txt, "7") if i == sel
                                       else c(txt, "32") if on else c(txt, "2")))
        sys.stdout.write("\n".join(parts) + "\n")
        sys.stdout.flush()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    bar.pause()
    applied = False
    try:
        tty.setcbreak(fd)
        render()
        while True:
            k = read_key(fd)
            if k == "q":
                break                                    # cancel — discard the working copy
            elif k == "esc":
                if buf:
                    buf = ""                             # cancel the in-progress rate edit
                else:
                    break
            elif k == "enter":
                flush()
                applied = True
                break
            elif k in ("up", "down"):
                flush()
                sel = (sel + (1 if k == "down" else -1)) % n
            elif k in ("space", "shift_enter"):
                flush()
                work[sel]["on"] = not work[sel]["on"]
            elif k in ("left", "right"):
                flush()
                rate = work[sel]["rate"]
                idx = min(range(len(RATE_LADDER)), key=lambda j: abs(RATE_LADDER[j] - rate))
                idx = max(0, min(len(RATE_LADDER) - 1, idx + (1 if k == "right" else -1)))
                work[sel]["rate"] = RATE_LADDER[idx]
                work[sel]["on"] = RATE_LADDER[idx] > 0
            elif k == "backspace":
                buf = buf[:-1]
            elif k.isdigit() or (k == "." and "." not in buf):
                buf += k
            sys.stdout.write(f"\033[{n}A")               # back to top of the list
            render()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if applied:
            for w in work:
                fc.set_stream(w["name"], enabled=w["on"], rate=w["rate"])
        bar.resume()
    if applied:
        non = sum(1 for w in work if w["on"] and w["rate"] > 0)
        print(f"  applied — {non}/{n} streams enabled.")
    else:
        print("  cancelled — no changes.")


def status_line(gcs, down, up):
    r = gcs.report()
    pg, cm = r["ping"], r["cmd"]
    rtt = f"{pg['avg_ms']}ms" if pg.get("count") else "—"
    ack = f"{cm['acked']}/{cm['sent']}"
    return (f"rx {r['rx_frames']:>6} @ {r['rx_rate_hz']:>6}Hz {r['rx_throughput_Bps']:>7.0f}B/s | "
            f"loss {r['est_loss_pct']:>5}% reord {r['reorders']:>4} crc {r['crc_errors']:>3} | "
            f"rtt {rtt:>8} | cmd {ack:>7} | down={cfg_str(down)} up={cfg_str(up)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", choices=list(SCENARIOS), default="clean")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    fc_port, gcs_port = free_port(), free_port()
    fc_sock, gcs_sock = udp_socket(fc_port), udp_socket(gcs_port)
    down = LinkConfig(seed=args.seed)                # FC egress (downlink)
    up = LinkConfig(seed=args.seed)                  # GCS egress (uplink)
    apply_scenario(down, args.scenario)
    apply_scenario(up, args.scenario)

    fc_link = Link(fc_sock, ("127.0.0.1", gcs_port), down, 0)
    gcs_link = Link(gcs_sock, ("127.0.0.1", fc_port), up, 1)
    fc = FC(fc_sock, fc_link, {"seed": args.seed})
    gcs = GCS(gcs_sock, gcs_link, {})

    inf = float("inf")
    tfc = threading.Thread(target=fc.run, args=(inf,), daemon=True)
    tgcs = threading.Thread(target=gcs.run, args=(inf,), kwargs={"grace": 0.0}, daemon=True)
    tfc.start()
    tgcs.start()
    time.sleep(0.3)

    print(f"NavLink v2 live console — scenario '{args.scenario}', seed {args.seed}")
    print("FC and GCS running. Type 'help' for commands, 'quit' to stop.\n")

    bar = StatusBar(Dashboard(fc, gcs, down, up, args.scenario))
    bar.start()

    def do_set(args_):
        tgt = "both"
        if args_ and args_[0] in ("up", "down", "both"):
            tgt = args_.pop(0)
        if len(args_) != 2 or args_[0] not in KNOBS:
            print(f"  usage: set [up|down] <{'/'.join(KNOBS)}> <value>")
            return
        field, val = KNOBS[args_[0]], float(args_[1])
        for name, cfg in (("down", down), ("up", up)):
            if tgt in ("both", name):
                setattr(cfg, field, val)
        print(f"  {tgt}.{args_[0]} = {val}")

    try:
        while True:
            try:
                line = input("nlctl> ").strip()
            except EOFError:
                break
            if not line:
                continue
            cmd, *rest = line.split()
            if cmd in ("quit", "exit", "q"):
                break
            elif cmd in ("help", "?"):
                print(__doc__[__doc__.index("At the"):])
            elif cmd in ("status", "s", "stat"):
                print("  " + status_line(gcs, down, up))
            elif cmd in ("clear", "clean", "cls"):
                if sys.stdout.isatty():
                    sys.stdout.write("\033[2J\033[1;1H")      # wipe screen, cursor to top
                    sys.stdout.flush()
                    bar.draw()                                # repaint dashboard at bottom
                    sys.stdout.write("\033[1;1H")             # prompt resumes at the top
                    sys.stdout.flush()
            elif cmd == "set":
                do_set(rest)
            elif cmd == "scenario":
                if not rest or rest[0] not in SCENARIOS:
                    print(f"  scenarios: {', '.join(SCENARIOS)}")
                else:
                    apply_scenario(down, rest[0])
                    apply_scenario(up, rest[0])
                    print(f"  applied '{rest[0]}': down={cfg_str(down)} up={cfg_str(up)}")
            elif cmd == "reset":
                apply_scenario(down, "clean")
                apply_scenario(up, "clean")
                print("  impairments cleared")
            elif cmd == "arm":
                gcs.send_command(nl.CmdArm, force=0)
                print("  → CMD_ARM")
            elif cmd == "disarm":
                gcs.send_command(nl.CmdDisarm, force=0)
                print("  → CMD_DISARM")
            elif cmd == "pid":
                if len(rest) != 3:
                    print("  usage: pid <kp> <ki> <kd>")
                else:
                    kp, ki, kd = (float(x) for x in rest)
                    gcs.send_command(nl.CmdSetPid, controller=0, axis=0,
                                     kp=kp, ki=ki, kd=kd, kff=0.0)
                    print(f"  → CMD_SET_PID kp={kp} ki={ki} kd={kd}")
            elif cmd == "ping":
                gcs.send_ping()
                print("  → PING")
            elif cmd == "tx":
                if not rest or rest[0] in ("select", "menu", "choose"):
                    tx_checklist(fc, bar)                          # interactive checklist
                elif rest[0] == "list":
                    print("  stream             rate    state    sent")
                    for name, rate, en, cnt in fc.stream_table():
                        print(f"  {name:<17} {rate:>5g}Hz  {'on' if en else 'off':<5}  {cnt:>7}")
                elif rest[0] in ("on", "off") and len(rest) >= 2:
                    ch = fc.set_stream(rest[1], enabled=(rest[0] == "on"))
                    print(f"  {rest[0]}: {', '.join(ch) or 'no match'}")
                elif rest[0] in ("on", "off"):                # no name → checklist
                    tx_checklist(fc, bar)
                elif rest[0] == "rate" and len(rest) >= 3:
                    ch = fc.set_stream(rest[1], rate=float(rest[2]))
                    print(f"  rate {rest[2]}Hz: {', '.join(ch) or 'no match'}")
                else:
                    print("  usage: tx (checklist) | tx list | tx on|off <name|all> | tx rate <name|all> <hz>")
            else:
                print(f"  unknown command '{cmd}' (try 'help')")
    except KeyboardInterrupt:
        pass
    finally:
        bar.stop()

    print("\nstopping…")
    fc.stop.set()
    gcs.stop.set()
    tfc.join(timeout=2)
    tgcs.join(timeout=2)
    fc_link.close()
    gcs_link.close()
    fc_sock.close()
    gcs_sock.close()

    r = gcs.report()
    print("─" * 60)
    print(f"final: {status_line(gcs, down, up)}")
    print(f"by msgid: " + ", ".join(f"{n}={c}" for n, c in
                                    sorted(r["by_msgid"].items(), key=lambda kv: -kv[1])))
    print("─" * 60)


if __name__ == "__main__":
    main()
