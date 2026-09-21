#!/usr/bin/env python3
# Copyright (C) 2026 NAVRobotec Pvt Ltd
# Author: Ragnar Vallhala
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Static allocation budget for the firmware: what RAM is spent, and where.

Answers the question that a runtime `heap_peak` only answers after the fact --
"will this allocation fit?" -- by reading the linked ELF instead of the source,
so macros, sizeof() and the optimiser are already resolved.

Three regions share the 96 KiB of SRAM on this part and the linker only sees
two of them:

    .data + .bss   the linker knows, and will not overflow silently
    HEAP_SIZE      memset at boot from _heap_start; the linker does NOT know,
                   so a heap that runs past the top of RAM corrupts memory and
                   HardFaults with nothing said
    MSP            grows DOWN from _estack into whatever is left

Every task stack is a heap allocation (`task_create` does v_malloc(sizeof(TCB))
plus v_malloc(stack_size)), so the interesting number is not the static total
but the heap's worst case. This walks the call graph from the roots, finds every
reachable `bl v_malloc`, recovers its size argument from the instruction that
loaded it, and separates what is spent at boot from what a later command can
still ask for.

Usage:
    tools/dev/alloc_budget.py                          # firmware/build/main
    tools/dev/alloc_budget.py --elf firmware/build-notch/main
    tools/dev/alloc_budget.py --top 25 --show-unresolved
"""
import argparse
import collections
import os
import re
import subprocess
import sys

RAM_TOP = 0x20018000          # STM32F401RE: 96 KiB from 0x20000000
PREFIX = "arm-none-eabi-"

# Thumb immediates that land a constant in a register, as objdump prints them.
RE_IMM = re.compile(r"\b(?:movs?|mov\.w|movw)\s+r(\d+),\s*#(\d+)")
RE_CALL = re.compile(r"\bbl(?:\.w)?\s+[0-9a-f]+\s+<([^>+]+)")
BLOCK_HDR = 16                # sizeof(Heap_Mem_Block); payload is 8B-aligned
RE_FUNC = re.compile(r"^([0-9a-f]+)\s+<([^>]+)>:")
RE_INSN = re.compile(r"^\s*([0-9a-f]+):\s")

# size argument register, by allocating function
ALLOC_ARG = {"v_malloc": 0, "task_create": 2, "task_create_named": 2}


def run(*a):
    return subprocess.run(a, capture_output=True, text=True, check=True).stdout


def symbols(elf):
    """-> {name: (addr, size, section_letter)} for data/bss symbols."""
    out = {}
    for line in run(PREFIX + "nm", "-S", "--size-sort", "-td", elf).splitlines():
        p = line.split()
        if len(p) < 4:
            continue
        addr, size, sect, name = p[0], p[1], p[2], " ".join(p[3:])
        if sect.lower() in ("b", "d"):
            out[name] = (int(addr), int(size), sect.lower())
    return out


def linker_syms(elf):
    out = {}
    for line in run(PREFIX + "nm", "-td", elf).splitlines():
        p = line.split()
        if len(p) >= 3 and p[2] in ("_heap_start", "_estack", "_ebss", "_sbss"):
            out[p[2]] = int(p[0])
    return out


def disassemble(elf):
    """-> (call graph {caller: {callee}}, alloc sites [(caller, callee, size)]).

    Sizes come from a short backward scan for the last immediate written to the
    argument register. Anything computed at runtime stays None and is reported
    as unresolved rather than guessed at -- a budget that quietly invents a
    number is worse than one that admits a hole."""
    text = run(PREFIX + "objdump", "-d", "--no-show-raw-insn", elf)
    graph = collections.defaultdict(collections.Counter)
    sites = []
    cur = None
    window = []                      # recent (reg, value) writes in this function
    for line in text.splitlines():
        m = RE_FUNC.match(line)
        if m:
            cur = m.group(2)
            window = []
            continue
        if cur is None or not RE_INSN.match(line):
            continue
        mi = RE_IMM.search(line)
        if mi:
            window.append((int(mi.group(1)), int(mi.group(2))))
            if len(window) > 24:
                window.pop(0)
        mc = RE_CALL.search(line)
        if mc:
            callee = mc.group(1).strip()
            graph[cur][callee] += 1
            if callee in ALLOC_ARG:
                reg = ALLOC_ARG[callee]
                size = next((v for r, v in reversed(window) if r == reg), None)
                sites.append((cur, callee, size))
            window = []              # a call clobbers r0-r3
    return graph, sites


def reachable(graph, roots):
    seen, stack = set(), list(roots)
    while stack:
        f = stack.pop()
        if f in seen:
            continue
        seen.add(f)
        stack.extend(graph.get(f, {}))
    return seen


def multiplicity(graph, roots, live):
    """How many times each function can be entered, counting STATIC call sites.

    A site counted once per function is a lower bound, and a budget that
    under-reports is the dangerous direction: init_axis() holds four allocations
    but runs three times, and task_create_named()'s TCB malloc runs once per
    task. Multiplying by the number of call sites on the path fixes both.

    Loops are NOT modelled -- a site inside a `for` still counts once per call
    site -- so this remains a lower bound, just a much tighter one."""
    mult = {r: 1 for r in roots}
    order, seen = [], set()

    def visit(f, path):
        if f in path:          # recursion: do not multiply forever
            return
        if f in seen:
            return
        seen.add(f)
        for callee in graph.get(f, {}):
            visit(callee, path | {f})
        order.append(f)

    for r in roots:
        visit(r, frozenset())
    for f in reversed(order):          # callers before callees
        for callee, n in graph.get(f, {}).items():
            if callee in live:
                mult[callee] = mult.get(callee, 0) + n * mult.get(f, 1)
    return mult


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(os.path.dirname(here))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--elf", default=os.path.join(root, "firmware/build/main"))
    ap.add_argument("--heap-size", type=lambda s: int(s, 0), default=None,
                    help="HEAP_SIZE; default reads it from vaios_app_config.h")
    ap.add_argument("--ram-top", type=lambda s: int(s, 0), default=RAM_TOP)
    ap.add_argument("--top", type=int, default=15)
    ap.add_argument("--fit", type=int, default=0, metavar="BYTES",
                    help="would a runtime allocation of BYTES fit after boot? "
                         "(adds the 16 B header and 8 B alignment for you)")
    ap.add_argument("--measured", type=int, default=0,
                    help="live heap_peak_bytes from PerfGlobal, to compare against")
    ap.add_argument("--watermark", type=int, default=1024,
                    help="HEAP_WATERMARK_THRESHOLD; v_malloc panics past this")
    ap.add_argument("--show-unresolved", action="store_true")
    a = ap.parse_args()

    if not os.path.exists(a.elf):
        sys.exit("no ELF at %s -- build first" % a.elf)

    heap_size = a.heap_size
    if heap_size is None:
        cfg = os.path.join(root, "firmware/include/vaios_app_config.h")
        m = re.search(r"^#define\s+HEAP_SIZE\s+(0x[0-9A-Fa-f]+|\d+)", open(cfg).read(), re.M)
        heap_size = int(m.group(1), 0) if m else 0

    syms = symbols(a.elf)
    lnk = linker_syms(a.elf)
    graph, sites = disassemble(a.elf)

    bss = sum(s for _, s, k in syms.values() if k == "b")
    data = sum(s for _, s, k in syms.values() if k == "d")
    heap_start = lnk.get("_heap_start", 0)
    top = lnk.get("_estack", a.ram_top)

    print("=" * 72)
    print("RAM MAP   %s" % os.path.relpath(a.elf, root))
    print("=" * 72)
    print("  .data + .bss      %8d B" % (bss + data))
    print("  _heap_start       0x%08X" % heap_start)
    print("  + HEAP_SIZE       %8d B  (0x%X)" % (heap_size, heap_size))
    print("  = heap end        0x%08X" % (heap_start + heap_size))
    print("  top of RAM        0x%08X" % top)
    slack = top - (heap_start + heap_size)
    print("  MSP + slack       %8d B   %s" % (slack, "OK" if slack > 0 else "*** OVERFLOW ***"))

    # roots: main plus every task entry (a task's stack is its own allocation,
    # so a task body is reachable even though nothing calls it directly)
    roots = ["main"] + [f for f in graph if f.endswith("_task")]
    live = reachable(graph, roots)
    boot = reachable(graph, ["main"])
    mult = multiplicity(graph, roots, live)

    print()
    print("=" * 72)
    print("HEAP ALLOCATION SITES (reachable from main or a task entry)")
    print("=" * 72)
    tot_boot = tot_run = 0
    unresolved = []
    rows = []
    for caller, callee, size in sites:
        if caller not in live:
            continue
        when = "boot" if caller in boot else "run"
        if size is None:
            unresolved.append((caller, callee, when))
            continue
        n = max(mult.get(caller, 1), 1)
        cost = n * (((size + 7) & ~7) + BLOCK_HDR)   # 8B-aligned payload + header
        rows.append((when, caller, callee, size, n, cost))
        if when == "boot":
            tot_boot += cost
        else:
            tot_run += cost
    rows.sort(key=lambda r: -r[5])
    print("  when   bytes  x   total  via                  in")
    for when, caller, callee, size, n, cost in rows[:a.top]:
        print("  %-4s %6d  %2d %7d  %-20s %s" % (when, size, n, cost, callee, caller))
    if len(rows) > a.top:
        print("  ... %d more resolved sites" % (len(rows) - a.top))

    print()
    print("  (sizes include the 16 B Heap_Mem_Block header and 8 B payload alignment)")
    print("  boot-time              %8d B" % tot_boot)
    print("  runtime worst case     %8d B   (every conditional path taken)" % tot_run)
    print("  total demand           %8d B" % (tot_boot + tot_run))
    print("  HEAP_SIZE              %8d B" % heap_size)
    # v_malloc panics with "Heap watermark exceeded" once a FAILED allocation
    # finds usage past this line, so the usable heap is smaller than HEAP_SIZE.
    usable = heap_size - a.watermark
    print("  usable (watermark %4d) %8d B" % (a.watermark, usable))
    print()
    print("  after boot             %8d B free   %s"
          % (usable - tot_boot,
             "OK" if tot_boot <= usable else "*** BOOT DOES NOT FIT ***"))
    print("  if every runtime path coincided: %d B free   %s"
          % (usable - tot_boot - tot_run,
             "OK" if tot_boot + tot_run <= usable else "OVER (see note)"))
    print()
    print("  NOTE: the runtime figure is an upper bound -- it assumes every")
    print("  conditional allocation is live at once (calibration AND an xfer AND")
    print("  ...), which they are not. Read 'after boot' as the real headroom and")
    print("  check individual runtime asks against it.")
    if a.measured:
        d = tot_boot - a.measured
        print()
        print("  measured heap peak     %8d B   (static estimate is %+d, %+.1f%%)"
              % (a.measured, d, 100.0 * d / a.measured))
    if unresolved:
        print("  UNRESOLVED sites     %8d      (size computed at runtime; not counted)"
              % len(unresolved))
        if a.show_unresolved:
            for caller, callee, when in unresolved:
                print("      %-4s %-20s %s" % (when, callee, caller))

    if a.fit:
        need = ((a.fit + 7) & ~7) + BLOCK_HDR
        free = usable - tot_boot
        print()
        print("  FIT CHECK: an allocation of %d B costs %d B with header/alignment"
              % (a.fit, need))
        print("             free after boot is %d B  ->  %s"
              % (free, "FITS, %d B to spare" % (free - need) if need <= free
                 else "DOES NOT FIT, short by %d B" % (need - free)))

    print()
    print("=" * 72)
    print("LARGEST STATIC OBJECTS")
    print("=" * 72)
    for name, (_, size, kind) in sorted(syms.items(), key=lambda kv: -kv[1][1])[:a.top]:
        print("  %8d B  .%s  %s" % (size, "bss" if kind == "b" else "data", name))
    return 0


if __name__ == "__main__":
    sys.exit(main())
