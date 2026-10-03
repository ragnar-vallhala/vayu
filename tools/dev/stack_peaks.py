#!/usr/bin/env python3
"""Measure every vaios task's stack high-water mark on a live board.

WHY THIS EXISTS. Task stacks in firmware/src/main.c are right-sized against
measured peaks, and no host test can see that budget: the vaios 0.2.0 bump cost
up to +240 B on the paths that reach the kernel console and panicked the FC at
boot on flush_task, having passed every build and all 73 host tests. So the
sizes have to be re-measured on hardware after any vaios or NavHAL bump, and
this is the thing that does it.

HOW IT WORKS. vaios paints each task's whole block with V_PERF_STACK_FILL at
create (kernel/task.c, under VAIOS_MODULE_PERF), so the untouched middle still
holds the sentinel. Scanning up from the block base, the first non-sentinel word
is the deepest the stack ever reached. That is the same calculation
v_perf_task_stats does on the target -- done here from a plain SRAM dump, which
gets every task at once and needs no telemetry link, no instrumentation and no
running GCS.

Tasks are found by scanning SRAM for TCB_MAGIC ("TCB!") rather than by walking
the kernel's ready/blocked/delayed lists, so a task parked anywhere still shows
up and the layout of the list links does not matter.

USAGE
    # dump and measure in one go (board must have been RUNNING a while)
    tools/dev/stack_peaks.py --port 1-1.3 --run 75

    # measure an SRAM dump taken some other way
    tools/dev/stack_peaks.py --sram dump.bin

Let it run long enough to exercise the paths you care about: a peak is only as
deep as the code that actually executed. Tasks whose deep path needs a real
download, a calibration or an armed flight are NOT covered by an idle bench run
-- grow those from the recorded numbers, never shrink them to a bench peak.
"""
import argparse, bisect, os, struct, subprocess, sys, tempfile

SRAM_BASE, SRAM_SIZE, FLASH_BASE = 0x20000000, 0x18000, 0x08000000
TCB_MAGIC, STACK_FILL = 0x54434221, 0xC5C5C5C5
# struct Task_Control_Block. These are FALLBACKS -- tcb_offsets() below reads the
# real layout out of the ELF's debug info, because the layout moves when a feature
# is switched on, not only when vaios is bumped: enabling VAIOS_MPU_STACK_GUARD
# adds mpu_guard[2] at offset 120 and pushes `magic` to 132, so a hardcoded
# MAGIC_OFF of 120 silently matches a guard word instead and the scan reports
# "no TCBs found" on a perfectly healthy board.
OFF = dict(sp=0, mem_block=4, entry=12, stack_size=16, task_id=20, name=28)
MAGIC_OFF = 120


def tcb_offsets(elf):
    """Field offsets from the ELF's DWARF, falling back to the constants above."""
    off, magic = dict(OFF), MAGIC_OFF
    try:
        out = subprocess.run(
            ["gdb-multiarch", "-batch", "-ex",
             "ptype /o struct Task_Control_Block", elf],
            capture_output=True, text=True, timeout=60).stdout
    except Exception:
        return off, magic
    import re as _re
    for line in out.splitlines():
        m = _re.match(r"/\*\s*(\d+)\s*\|\s*\d+\s*\*/\s+.*?\b(\w+)\s*(?:\[\d+\])?;", line)
        if not m:
            continue
        pos, nm = int(m.group(1)), m.group(2)
        if nm == "magic":
            magic = pos
        elif nm in off:
            off[nm] = pos
    return off, magic
# What a stack has to clear above its measured peak.
MPU_GUARD = 32   # VAIOS_MPU_GUARD_SIZE: the no-access region at the block base.
                 # Touching it faults in hardware, so this is the hard floor.
FP_FRAME = 132   # an FP exception frame, if a preemption lands at peak depth
LOG_DEPTH = 172  # vayu_log's worst-case depth, measured from the disassembly.
                 # Only matters for a task that CAN reach it -- and a task that
                 # logs on an error path only does not show that depth in a bench
                 # peak, which is how motor_task's 704 B came to be too small.
#
# The software watermark's 256 B threshold used to be in this sum. It is gone:
# TASK_STACK_WATERMARK_ENABLE is off, because the MPU guard region replaces it.


def openocd_dump(port, run_s, out):
    cmd = ["openocd", "-f", "interface/stlink.cfg",
           "-c", f"adapter usb location {port}",
           "-c", "transport select hla_swd", "-f", "target/stm32f4x.cfg",
           "-c", f"init; sleep {int(run_s * 1000)}; halt; "
                 f"dump_image {out} {hex(SRAM_BASE)} {hex(SRAM_SIZE)}; resume; exit"]
    r = subprocess.run(cmd, capture_output=True, text=True,
                       timeout=run_s + 120)
    if r.returncode != 0 or not os.path.exists(out):
        sys.exit("openocd dump failed:\n" + r.stderr[-2000:])


def symbolizer(elf):
    """Nearest preceding function symbol, for a task with no name set."""
    try:
        out = subprocess.run(["arm-none-eabi-nm", "-n", elf],
                             capture_output=True, text=True, check=True).stdout
    except Exception:
        return lambda a: "?"
    syms = sorted((int(p[0], 16), p[2]) for p in
                  (l.split() for l in out.splitlines())
                  if len(p) == 3 and p[1] in "tTwW")
    addrs = [a for a, _ in syms]

    def f(a):
        i = bisect.bisect_right(addrs, a & ~1) - 1
        return syms[i][1] if i >= 0 else "?"
    return f


def measure(sram, flash, sym, fo=OFF, magic_off=MAGIC_OFF):
    u32 = lambda b, o: struct.unpack_from("<I", b, o)[0]

    def cstr(ptr):
        if not flash or not (FLASH_BASE <= ptr < FLASH_BASE + len(flash)):
            return ""
        o = ptr - FLASH_BASE
        end = flash.find(b"\0", o)
        return flash[o:end].decode("ascii", "replace") if end >= 0 else ""

    rows = []
    for off in range(0, len(sram) - 4, 4):
        if u32(sram, off) != TCB_MAGIC:
            continue
        base = off - magic_off
        if base < 0:
            continue
        mem = u32(sram, base + fo["mem_block"])
        size = u32(sram, base + fo["stack_size"])
        # Reject a stray magic-looking word: a real TCB points its block into
        # SRAM and carries a plausible size.
        if not (SRAM_BASE <= mem < SRAM_BASE + len(sram)) or not 0 < size <= 16384:
            continue
        mo = mem - SRAM_BASE
        if mo + size > len(sram):
            continue
        words = size // 4
        j = 0
        while j < words and u32(sram, mo + j * 4) == STACK_FILL:
            j += 1
        rows.append(dict(
            name=cstr(u32(sram, base + fo["name"])) or sym(u32(sram, base + fo["entry"])),
            tid=u32(sram, base + fo["task_id"]), size=size,
            peak=(words - j) * 4, saturated=(j == 0)))
    return sorted(rows, key=lambda r: -r["peak"])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="ST-Link USB port path, e.g. 1-1.3 "
                                   "(fingerprint it first -- these adapters share serials)")
    ap.add_argument("--run", type=float, default=75.0,
                    help="seconds to let the board run before dumping (default 75)")
    ap.add_argument("--sram", help="use an existing SRAM dump instead of taking one")
    ap.add_argument("--elf", default="firmware/build/main",
                    help="ELF for symbol names (default firmware/build/main)")
    ap.add_argument("--bin", default="firmware/build/main.bin",
                    help="flash image, to read task name strings")
    ap.add_argument("--check", action="store_true",
                    help="exit non-zero if any task is within the guard band + "
                         "an FP exception frame")
    a = ap.parse_args()

    tmp = None
    if a.sram:
        path = a.sram
    elif a.port:
        tmp = tempfile.NamedTemporaryFile(suffix=".bin", delete=False)
        tmp.close()
        path = tmp.name
        print(f"running the board for {a.run:g}s, then dumping SRAM...")
        openocd_dump(a.port, a.run, path)
    else:
        sys.exit("need --port (to dump) or --sram (to read a dump)")

    sram = open(path, "rb").read()
    flash = open(a.bin, "rb").read() if os.path.exists(a.bin) else b""
    fo, magic_off = tcb_offsets(a.elf) if os.path.exists(a.elf) else (OFF, MAGIC_OFF)
    rows = measure(sram, flash,
                   symbolizer(a.elf) if os.path.exists(a.elf) else (lambda x: "?"),
                   fo, magic_off)
    if tmp:
        os.unlink(tmp.name)

    if not rows:
        sys.exit("no TCBs found. Is VAIOS_MODULE_PERF on (it paints the stacks), "
                 "and was this dump taken from a running board?")

    print(f"{'task':<18}{'size':>7}{'peak':>7}{'free':>7}{'use%':>6}  verdict")
    worst = []
    for r in rows:
        free = r["size"] - r["peak"]
        if r["saturated"]:
            v = "SATURATED -- peak unmeasurable, grow it and re-run"
        elif free < MPU_GUARD:
            v = f"FAULTS -- inside the {MPU_GUARD} B MPU guard region"
        elif free < MPU_GUARD + FP_FRAME:
            v = f"thin -- under guard + FP frame ({MPU_GUARD + FP_FRAME} B)"
        elif free < MPU_GUARD + FP_FRAME + LOG_DEPTH:
            # Not a fault: correct iff this task cannot reach vayu_log. Check the
            # source for vayu_log / VAYU_ASSERT / PANIC and for an indirect call
            # that could reach one -- a static bl-edge scan misses function
            # pointers, which is exactly how motor_task reaches it.
            v = "ok ONLY if this task cannot log"
        else:
            v = "ok"
        if v.startswith(("FAULTS", "thin", "SATURATED")):
            worst.append(r["name"])
        print(f"{r['name']:<18}{r['size']:>7}{r['peak']:>7}{free:>7}"
              f"{100 * r['peak'] / r['size']:>5.0f}%  {v}")

    print(f"\n{len(rows)} tasks; allocated {sum(r['size'] for r in rows)} B, "
          f"peak {sum(r['peak'] for r in rows)} B")
    if worst:
        print("needs attention:", ", ".join(worst))
        print(f"size each at >= peak + {MPU_GUARD + FP_FRAME} B, and + "
              f"{LOG_DEPTH} B more if it can log. Sizes must be POWERS OF TWO "
              f"with the MPU stack guard on.")
    else:
        print(f"every task clears guard + FP frame ({MPU_GUARD + FP_FRAME} B)")
    return 1 if (a.check and worst) else 0


if __name__ == "__main__":
    sys.exit(main())
