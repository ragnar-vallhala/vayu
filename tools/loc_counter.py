#!/usr/bin/env python3
"""Project-wide LOC counter for the Vayu stack.

Reports source lines (blank- and comment-stripped) per component and a combined
total:

  - vayu      : the firmware / flight stack (src/, include/)            — C
  - navlink   : the wire-protocol codec + generator (navlink/)          — Python, JSON, C
  - navigator : the ground-control station (software/src, software/tests) — C++

Generated code (navlink/generated, */navlink_gen), build trees, vendored
submodules and virtualenvs are excluded — only hand-written source is counted.

Usage:
    python3 tools/loc_counter.py [--files] [--root <project-root>]

    --files   also list every file with its LOC, grouped by component
    --root    override the project root (default: the parent of tools/)
"""
import argparse
import os
import sys

# ── component map ────────────────────────────────────────────────────────────
# Each component is (name, source-roots, extensions, group). Paths are relative
# to the project root; edit here to re-scope a component. The "own" group is
# first-party code; "vendored" is vendored submodules (reported and totalled
# separately so the first-party size stays legible). Nested extern/, build trees
# and .git are pruned by EXCLUDE_DIRS, so vaios never double-counts NavHAL.
CC = {".c", ".h", ".cpp", ".cc", ".cxx", ".hpp", ".hh"}
COMPONENTS = [
    ("vayu (firmware)",  ["src", "include"],                  {".c", ".h"},               "own"),
    ("navlink (codec)",  ["navlink"],                         {".py", ".json", ".c", ".h"}, "own"),
    ("navigator (GCS)",  ["software/src", "software/tests"],  CC,                         "own"),
    ("vaios (RTOS)",     ["extern/vaios"],                    CC,                         "own"),
    ("navhal (HAL)",     ["extern/vaios/extern/NavHAL"],      CC,                         "own"),
]

# Never descend into these (derived output, build trees, vendored deps).
EXCLUDE_DIRS = {
    "build", "build_flash", "build_sitl", "build_vsim", "build_fw_navlink",
    "extern", "venv", ".venv", "__pycache__", ".git", "generated",
    "navlink_gen", "node_modules",
}

C_LIKE = {".c", ".h", ".cpp", ".cc", ".cxx", ".hpp", ".hh", ".hxx"}


# ── per-language line counters ───────────────────────────────────────────────
def count_c_like(lines):
    """Source lines for C/C++: drop blanks, // line comments and /* */ blocks."""
    loc = 0
    in_block = False
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if in_block:
            if "*/" in line:
                in_block = False
                line = line.split("*/", 1)[1].strip()
                if not line:
                    continue
            else:
                continue
        # may contain one or more comment fragments; collapse the common cases
        while "/*" in line:
            before, rest = line.split("/*", 1)
            if "*/" in rest:
                line = (before + " " + rest.split("*/", 1)[1]).strip()
            else:
                line = before.strip()
                in_block = True
                break
        if not line:
            continue
        if line.startswith("//"):
            continue
        if "//" in line:
            line = line.split("//", 1)[0].strip()
            if not line:
                continue
        loc += 1
    return loc


def count_python(lines):
    """Source lines for Python: drop blanks, # comments and triple-quoted blocks.

    Triple-quoted blocks are treated as non-code (docstrings) — a pragmatic
    heuristic; a triple-quoted string used as an expression is rare in this tree."""
    loc = 0
    triple = None  # active triple-quote delimiter, or None
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if triple:
            if triple in line:
                line = line.split(triple, 1)[1].strip()
                triple = None
                if not line:
                    continue
            else:
                continue
        if line.startswith("#"):
            continue
        for delim in ('"""', "'''"):
            if line.startswith(delim):
                rest = line[3:]
                if delim in rest:          # opens and closes on one line
                    line = rest.split(delim, 1)[1].strip()
                else:
                    triple = delim
                    line = ""
                break
        if not line:
            continue
        if "#" in line:                    # strip trailing comment (heuristic)
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
        loc += 1
    return loc


def count_plain(lines):
    """Non-blank lines (JSON and other data — no comment syntax)."""
    return sum(1 for line in lines if line.strip())


def count_file(path):
    ext = os.path.splitext(path)[1].lower()
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()
    except OSError as e:
        print(f"  (skip {path}: {e})", file=sys.stderr)
        return 0
    if ext in C_LIKE:
        return count_c_like(lines)
    if ext == ".py":
        return count_python(lines)
    return count_plain(lines)


# ── scanning ─────────────────────────────────────────────────────────────────
def scan_component(root, dirs, exts):
    """Return (total_loc, file_count, per_ext{ext:loc}, files[(path,loc)])."""
    total, count = 0, 0
    per_ext, files = {}, []
    for rel in dirs:
        base = os.path.join(root, rel)
        if not os.path.isdir(base):
            print(f"  (missing dir: {rel})", file=sys.stderr)
            continue
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [d for d in dirnames if d not in EXCLUDE_DIRS]
            for fn in filenames:
                ext = os.path.splitext(fn)[1].lower()
                if ext not in exts:
                    continue
                full = os.path.join(dirpath, fn)
                loc = count_file(full)
                total += loc
                count += 1
                per_ext[ext] = per_ext.get(ext, 0) + loc
                files.append((os.path.relpath(full, root), loc))
    return total, count, per_ext, files


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_root = os.path.dirname(here)
    ap = argparse.ArgumentParser(description="Vayu project LOC counter")
    ap.add_argument("--files", action="store_true", help="list every file with its LOC")
    ap.add_argument("--root", default=default_root, help="project root (default: parent of tools/)")
    args = ap.parse_args()
    root = os.path.abspath(args.root)

    print(f"Project root: {root}\n")
    rows = []
    for name, dirs, exts, group in COMPONENTS:
        total, count, per_ext, files = scan_component(root, dirs, exts)
        rows.append((name, count, total, per_ext, group))
        if args.files:
            print(f"── {name} ──")
            for path, loc in sorted(files, key=lambda x: x[1], reverse=True):
                print(f"  {loc:6d}  {path}")
            print()

    width = max([len(n) for n, *_ in rows] + [len("TOTAL (incl. vendored)")])
    w = width + 34
    own = [r for r in rows if r[4] == "own"]
    vendored = [r for r in rows if r[4] == "vendored"]

    def line(name, count, loc, bd=""):
        print(f"{name.ljust(width)}  {count:6d}  {loc:8d}   {bd}")

    def bd_of(per_ext):
        return ", ".join(f"{e.lstrip('.')}:{loc}" for e, loc in
                         sorted(per_ext.items(), key=lambda x: x[1], reverse=True))

    print("=" * w)
    print(f"{'COMPONENT'.ljust(width)}  {'FILES':>6}  {'LOC':>8}   breakdown")
    print("-" * w)
    for name, count, total, per_ext, _ in own:
        line(name, count, total, bd_of(per_ext))
    own_f, own_l = sum(r[1] for r in own), sum(r[2] for r in own)
    # The vendored split only appears when something is actually tagged vendored;
    # otherwise the report is a flat list with one grand total.
    if vendored:
        print("-" * w)
        line("subtotal (first-party)", own_f, own_l)
        print()
        for name, count, total, per_ext, _ in vendored:
            line(name, count, total, bd_of(per_ext))
    print("-" * w)
    line("TOTAL", own_f + sum(r[1] for r in vendored),
         own_l + sum(r[2] for r in vendored))
    print("=" * w)


if __name__ == "__main__":
    main()
