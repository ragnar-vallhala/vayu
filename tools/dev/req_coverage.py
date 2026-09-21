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
"""req_coverage.py — reverse traceability ("code without a requirement").

`trace.py` answers the FORWARD question: for each requirement, which files
implement/verify it. This tool answers the REVERSE question, at function
granularity: which functions in firmware/src carry no `@implements` link to a
requirement at all — the orphan code a forward-only matrix can never surface.

A function definition is COVERED when an `@implements MOD-SUB-NNN` tag is
attached to it — either in its own doxygen block / body, or on its declaration
in a header (matched by name, the way `trace.py` already credits headers as
implementers). A function is EXEMPT when its block carries `@noreq <reason>`
(genuine infrastructure/boilerplate with no requirement to trace to). Everything
else is an ORPHAN.

The C "parser" is a pragmatic brace/semicolon scanner over a comment- and
string-masked copy of each file. It is heuristic, not a real frontend: it is
tuned to this codebase's style (top-level K&R-ish definitions) and errs toward
*reporting* a function rather than silently dropping it.

CLI:
    tools/dev/req_coverage.py                 Human report (summary + per-module).
    tools/dev/req_coverage.py --list          Also list every orphan function.
    tools/dev/req_coverage.py --module CTRL    Restrict to one module dir.
    tools/dev/req_coverage.py --json           Machine-readable dump.
    tools/dev/req_coverage.py --check --min 60 CI gate: fail if function
                                               coverage < 60% (exempt excluded).

@implements CONV-03
"""

import argparse
import json
import re
import sys
from pathlib import Path

# Reuse trace.py's requirements + tag parsing for the forward-side summary so
# the two tools never disagree about what a requirement or a tag is.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import trace as fwd  # noqa: E402

REPO_ROOT = fwd.REPO_ROOT
SRC_ROOT = REPO_ROOT / "firmware" / "src"
INC_ROOT = REPO_ROOT / "firmware" / "include"

# Directory under firmware/src -> canonical requirement module prefix. Used only
# for grouping/labelling; unmapped dirs fall back to their uppercased name.
DIR_MODULE = {
    "actuator": "ACT",
    "calib": "SNS",
    "comm": "COMM",
    "control": "CTRL",
    "est": "EST",
    "logger": "LOG",
    "sensor": "SNS",
    "storage": "LOG",
    "sys": "SYS",
    "maths": "MATH",
    "utils": "UTIL",
}

C_KEYWORDS = {
    "if", "for", "while", "switch", "do", "else", "return", "sizeof",
    "typedef", "struct", "enum", "union", "case", "default", "goto",
    "break", "continue", "static", "const", "volatile", "register",
}

NOREQ_RE = re.compile(r"@noreq\b")


# ----------------------------------------------------------------------------
# Comment/string masking
# ----------------------------------------------------------------------------

def mask(src: str) -> str:
    """Return src with comments and string/char literals blanked to spaces
    (newlines preserved, length preserved) so braces/parens can be counted
    without tripping over literal or commented-out code. Preprocessor lines
    are blanked too — function-like macros otherwise look like definitions."""
    out = list(src)
    n = len(src)
    i = 0
    state = "code"
    while i < n:
        c = src[i]
        nxt = src[i + 1] if i + 1 < n else ""
        if state == "code":
            if c == "/" and nxt == "/":
                out[i] = out[i + 1] = " "
                i += 2
                state = "line"
                continue
            if c == "/" and nxt == "*":
                out[i] = out[i + 1] = " "
                i += 2
                state = "block"
                continue
            if c == '"':
                out[i] = " "
                i += 1
                state = "str"
                continue
            if c == "'":
                out[i] = " "
                i += 1
                state = "chr"
                continue
            i += 1
        elif state == "line":
            if c == "\n":
                state = "code"
            else:
                out[i] = " "
            i += 1
        elif state == "block":
            if c == "*" and nxt == "/":
                out[i] = out[i + 1] = " "
                i += 2
                state = "code"
                continue
            if c != "\n":
                out[i] = " "
            i += 1
        elif state in ("str", "chr"):
            if c == "\\":
                out[i] = " "
                if i + 1 < n and src[i + 1] != "\n":
                    out[i + 1] = " "
                i += 2
                continue
            if (state == "str" and c == '"') or (state == "chr" and c == "'"):
                out[i] = " "
                i += 1
                state = "code"
                continue
            if c != "\n":
                out[i] = " "
            i += 1

    # Blank preprocessor lines (respecting backslash-continuation).
    masked = "".join(out)
    lines = masked.split("\n")
    cont = False
    for idx, ln in enumerate(lines):
        if cont or ln.lstrip().startswith("#"):
            cont = ln.rstrip().endswith("\\")
            lines[idx] = " " * len(ln)
    return "\n".join(lines)


# ----------------------------------------------------------------------------
# Function/declaration extraction
# ----------------------------------------------------------------------------

def _line_starts(src: str):
    """Offsets where each line begins (for offset -> line-number lookup)."""
    starts = [0]
    for m in re.finditer("\n", src):
        starts.append(m.end())
    return starts


def _line_of(starts, off: int) -> int:
    # binary search; lines are 1-based
    lo, hi = 0, len(starts) - 1
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if starts[mid] <= off:
            lo = mid
        else:
            hi = mid - 1
    return lo + 1


def _sig_name(sig: str):
    """Given the text of a top-level segment ending at `{` or `;`, return the
    function name if it looks like a definition/declaration, else None."""
    s = " ".join(sig.split())
    if not s or "(" not in s or ")" not in s:
        return None
    # Aggregate initialiser / assignment, not a function: `... = {` / `x = f();`
    if "=" in s.split("(")[0]:
        return None
    # Name is the identifier immediately before the first parameter `(`.
    m = re.match(r"^(.*?)([A-Za-z_]\w*)\s*\(", s)
    if not m:
        return None
    name = m.group(2)
    if name in C_KEYWORDS:
        return None
    # A bare `(` opening a grouping rather than a call/params (e.g. `return (a)`)
    # is already excluded by the keyword set and the `=` check above.
    return name


def _name_line(starts, masked, seg_start, seg_end, name):
    """Line of the signature (the `name(` token), so reports/LOC anchor on code
    rather than the doc-comment that precedes it. Comments are masked out, so the
    first `name(` in the segment is the real signature position."""
    m = re.search(r"\b" + re.escape(name) + r"\s*\(", masked[seg_start:seg_end])
    off = seg_start + (m.start() if m else 0)
    return _line_of(starts, off)


def extract_entities(src: str, masked: str):
    """Yield dicts for each top-level function definition and declaration:
    {name, kind: 'def'|'decl', start_line, end_line}."""
    starts = _line_starts(src)
    n = len(masked)
    depth = 0
    seg_start = 0
    i = 0
    pending = None  # (name, sig_line) once we open a def body
    ents = []
    while i < n:
        c = masked[i]
        if c == "{":
            if depth == 0:
                name = _sig_name(masked[seg_start:i])
                pending = ((name, _name_line(starts, masked, seg_start, i, name))
                           if name else None)
            depth += 1
        elif c == "}":
            depth -= 1
            if depth <= 0:
                depth = 0
                if pending and pending[0]:
                    ents.append({
                        "name": pending[0],
                        "kind": "def",
                        "start_line": pending[1],
                        "end_line": _line_of(starts, i),
                    })
                pending = None
                seg_start = i + 1
        elif c == ";":
            if depth == 0:
                name = _sig_name(masked[seg_start:i])
                if name:
                    ents.append({
                        "name": name,
                        "kind": "decl",
                        "start_line": _name_line(starts, masked, seg_start, i, name),
                        "end_line": _line_of(starts, i),
                    })
                seg_start = i + 1
        i += 1
    return ents


def tags_for_entity(src_lines, masked_lines, ent):
    """Collect (@implements ids, has_noreq) attached to an entity: its adjacent
    doc-comment block immediately above plus, for defs, its body."""
    start = ent["start_line"]  # 1-based
    # Walk upward over the contiguous comment block directly above (allow the
    # block to sit flush against the signature; a single blank line is fine).
    text_parts = []
    j = start - 1  # line index just above (1-based)
    blanks = 0
    while j >= 1:
        orig = src_lines[j - 1]
        masked = masked_lines[j - 1]
        is_comment = orig.strip() != "" and masked.strip() == ""
        if is_comment:
            text_parts.append(orig)
            j -= 1
            continue
        if orig.strip() == "" and blanks == 0:
            blanks += 1
            j -= 1
            continue
        break
    if ent["kind"] == "def":
        text_parts.extend(src_lines[start - 1:ent["end_line"]])
    blob = "\n".join(text_parts)
    ids = set()
    for m in fwd.IMPL_RE.finditer(blob):
        ids.update(fwd.ID_RE.findall(m.group(1)))
    return ids, bool(NOREQ_RE.search(blob))


# ----------------------------------------------------------------------------
# Analysis
# ----------------------------------------------------------------------------

def analyse():
    """Build per-function coverage over firmware/src, crediting header tags."""
    # Pass 1: harvest every name that carries an @implements anywhere (src or
    # include, def or decl) — and names explicitly exempted with @noreq.
    tagged_names = {}   # name -> set(ids)
    noreq_names = set()
    parsed = {}         # path -> (src_lines, masked_lines, entities)
    for root in (SRC_ROOT, INC_ROOT):
        for path in sorted(root.rglob("*")):
            if path.suffix not in (".c", ".h"):
                continue
            src = path.read_text(encoding="utf-8", errors="replace")
            masked = mask(src)
            ents = extract_entities(src, masked)
            sl, ml = src.split("\n"), masked.split("\n")
            parsed[path] = (sl, ml, ents)
            for ent in ents:
                ids, noreq = tags_for_entity(sl, ml, ent)
                if ids:
                    tagged_names.setdefault(ent["name"], set()).update(ids)
                if noreq:
                    noreq_names.add(ent["name"])

    # Pass 2: classify every DEFINITION under firmware/src.
    funcs = []
    for path, (sl, ml, ents) in parsed.items():
        if SRC_ROOT not in path.parents and path.parent != SRC_ROOT:
            continue
        rel = path.relative_to(REPO_ROOT)
        parts = path.relative_to(SRC_ROOT).parts
        moddir = parts[0] if len(parts) > 1 else "(root)"
        for ent in ents:
            if ent["kind"] != "def":
                continue
            name = ent["name"]
            ids = set(tagged_names.get(name, set()))
            exempt = name in noreq_names
            loc = ent["end_line"] - ent["start_line"] + 1
            funcs.append({
                "file": str(rel),
                "module": moddir,
                "modprefix": DIR_MODULE.get(moddir, moddir.upper()),
                "name": name,
                "line": ent["start_line"],
                "loc": loc,
                "ids": sorted(ids),
                "covered": bool(ids),
                "exempt": exempt,
            })
    return funcs


def summarise(funcs):
    """Roll up coverage per module and overall (exempt excluded from the base)."""
    mods = {}
    for f in funcs:
        m = mods.setdefault(f["module"], {
            "modprefix": f["modprefix"],
            "total": 0, "covered": 0, "exempt": 0,
            "loc": 0, "orphan_loc": 0,
        })
        m["total"] += 1
        m["loc"] += f["loc"]
        if f["exempt"]:
            m["exempt"] += 1
        elif f["covered"]:
            m["covered"] += 1
        else:
            m["orphan_loc"] += f["loc"]
    return mods


def pct(num, den):
    return 100.0 * num / den if den else 100.0


# ----------------------------------------------------------------------------
# Output
# ----------------------------------------------------------------------------

def forward_summary():
    """Requirement -> implementer counts, straight from trace.py."""
    reqs = fwd.parse_requirements(fwd.REQUIREMENTS_MD)
    impls, verifs, _ = fwd.walk_tags(fwd.OWNED_ROOTS + fwd.VENDOR_ROOTS)
    active = [r for r, m in reqs.items() if m["status"] == fwd.ACTIVE]
    with_impl = sum(1 for r in active if r in impls)
    with_ver = sum(
        1 for r in active
        if r in verifs or (r in impls and all(fwd.is_vendor(p) for p in impls[r]))
    )
    return len(active), with_impl, with_ver


def human_report(funcs, mods, module_filter, list_orphans):
    n_active, with_impl, with_ver = forward_summary()
    base = [f for f in funcs if not f["exempt"]]
    covered = sum(1 for f in base if f["covered"])
    total_loc = sum(f["loc"] for f in funcs)
    orphan_loc = sum(f["loc"] for f in base if not f["covered"])
    exempt = sum(1 for f in funcs if f["exempt"])

    print("Vayu — requirement coverage (bidirectional traceability)\n")
    print("Forward  (requirement -> code), via trace.py:")
    print(f"  active requirements with an implementer : {with_impl}/{n_active} "
          f"({pct(with_impl, n_active):.0f}%)")
    print(f"  active requirements with a verifier     : {with_ver}/{n_active} "
          f"({pct(with_ver, n_active):.0f}%)")
    print("\nReverse  (code -> requirement), firmware/src function definitions:")
    print(f"  functions traced to a requirement       : {covered}/{len(base)} "
          f"({pct(covered, len(base)):.0f}%)")
    print(f"  orphan functions (no @implements)       : {len(base) - covered}")
    print(f"  exempt (@noreq)                         : {exempt}")
    print(f"  orphan LOC / total def LOC              : {orphan_loc}/{total_loc} "
          f"({pct(orphan_loc, total_loc):.0f}%)")

    print("\nPer-module (reverse):")
    print(f"  {'module':<10} {'mod':<5} {'covered':>9}  {'orphan':>6}  "
          f"{'exempt':>6}  {'cov%':>5}  {'orphanLOC':>9}")
    for name in sorted(mods):
        if module_filter and module_filter not in (name, mods[name]["modprefix"]):
            continue
        m = mods[name]
        cov_base = m["total"] - m["exempt"]
        orphan = cov_base - m["covered"]
        print(f"  {name:<10} {m['modprefix']:<5} "
              f"{str(m['covered'])+'/'+str(cov_base):>9}  {orphan:>6}  "
              f"{m['exempt']:>6}  {pct(m['covered'], cov_base):>4.0f}%  "
              f"{m['orphan_loc']:>9}")

    if list_orphans:
        print("\nOrphan functions (no requirement; largest first):")
        orphans = [f for f in base if not f["covered"]]
        if module_filter:
            orphans = [f for f in orphans
                       if module_filter in (f["module"], f["modprefix"])]
        for f in sorted(orphans, key=lambda x: (-x["loc"], x["file"])):
            print(f"  {f['file']}:{f['line']:<5} {f['name']:<34} "
                  f"{f['loc']:>4} LOC  [{f['modprefix']}]")
    print()


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        prog="req_coverage.py",
        description="Reverse traceability: code without a requirement.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--list", action="store_true",
                    help="List every orphan function (largest first).")
    ap.add_argument("--module", metavar="MOD",
                    help="Restrict report to one module dir (e.g. control) "
                         "or module prefix (e.g. CTRL).")
    ap.add_argument("--json", action="store_true",
                    help="Emit a machine-readable JSON dump and exit.")
    ap.add_argument("--check", action="store_true",
                    help="CI gate: exit non-zero if coverage below --min.")
    ap.add_argument("--min", type=float, default=0.0, metavar="PCT",
                    help="Minimum function coverage %% for --check (default 0).")
    args = ap.parse_args()

    funcs = analyse()
    mods = summarise(funcs)

    if args.json:
        base = [f for f in funcs if not f["exempt"]]
        covered = sum(1 for f in base if f["covered"])
        print(json.dumps({
            "functions": funcs,
            "summary": {
                "total_defs": len(funcs),
                "base": len(base),
                "covered": covered,
                "orphans": len(base) - covered,
                "exempt": sum(1 for f in funcs if f["exempt"]),
                "coverage_pct": round(pct(covered, len(base)), 1),
            },
        }, indent=2))
        return 0

    human_report(funcs, mods, args.module, args.list or args.check)

    if args.check:
        base = [f for f in funcs if not f["exempt"]]
        covered = sum(1 for f in base if f["covered"])
        cov = pct(covered, len(base))
        if cov < args.min:
            print(f"FAIL: function coverage {cov:.0f}% < required {args.min:.0f}%",
                  file=sys.stderr)
            return 1
        print(f"OK: function coverage {cov:.0f}% >= {args.min:.0f}%",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
