#!/usr/bin/env python3
"""trace.py — traceability gate for vayu firmware (CONV-03).

Walks the vayu-owned source trees (firmware/src, firmware/include, tools/) and the
vendored subtrees (extern/vaios/**, including extern/vaios/extern/NavHAL/**)
for `@implements` and `@verifies` doc tags, cross-references them
against the MOD-SUB-NNN rows in docs/reference/requirements.md, and
emits docs/reference/trace.md.

Per docs/reference/requirements.md §5.3 — vendor exemption:
    vendor subtrees count for `@implements` discovery
    vendor subtrees do *not* count for `@verifies`; requirements
    implemented only in vendor code with no in-repo verifier are
    marked `verified-upstream` rather than failing.

CLI:
    tools/dev/trace.py                  Regenerate docs/reference/trace.md.
    tools/dev/trace.py --check          CI gate. Unknown ID fails (Phase 1).
                                    Missing implementer/verifier warns.
    tools/dev/trace.py --check --strict Phase-5 mode: missing implementer or
                                    verifier on an active requirement
                                    becomes a failure.

The trace.md output is regenerated unconditionally; `--check` adds the
gating semantics on top.

@implements CONV-03
"""

import argparse
import re
import sys
from pathlib import Path

# ----------------------------------------------------------------------------
# Layout
# ----------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
REQUIREMENTS_MD = REPO_ROOT / "docs" / "reference" / "requirements.md"
TRACE_MD = REPO_ROOT / "docs" / "reference" / "trace.md"

OWNED_ROOTS = [
    REPO_ROOT / "firmware" / "src",
    REPO_ROOT / "firmware" / "include",
    REPO_ROOT / "tools",
]
VENDOR_ROOTS = [
    REPO_ROOT / "extern" / "vaios",
]

# Files this script must not parse for its own tag patterns.
SELF_PATH = Path(__file__).resolve()

# ----------------------------------------------------------------------------
# Regexes
# ----------------------------------------------------------------------------

# MOD-SUB-NNN where MOD is one of the nine documented module prefixes,
# SUB is alphanumeric (uppercase), NNN is 3 digits. CONV-NN (only two
# hyphenated segments) and other 2-segment tokens are intentionally
# excluded — those live in coding-guidelines.md §7.13 with their own
# ✅ tracking convention.
MODULE_PREFIXES = ("SYS", "HAL", "VOS", "SNS", "EST", "CTRL", "ACT", "COMM", "LOG")
ID_RE = re.compile(
    r"\b((?:" + "|".join(MODULE_PREFIXES) + r")-[A-Z][A-Z0-9]*-\d{3})\b"
)

# Tag spans — capture text after @implements / @verifies until newline
# or `*` (which terminates a doxygen block). IDs are then extracted
# from the captured span via ID_RE.
IMPL_RE = re.compile(r"@implements\s+([^\n*]+)")
VERIFY_RE = re.compile(r"@verifies\s+([^\n*]+)")

# Markdown table row beginning with `| <ID> |`.
REQ_ROW_RE = re.compile(r"^\|\s*((?:" + "|".join(MODULE_PREFIXES) +
                        r")-[A-Z][A-Z0-9]*-\d{3})\s*\|(.*)$")

# ----------------------------------------------------------------------------
# Status classification
# ----------------------------------------------------------------------------

ACTIVE = "active"
DROPPED = "dropped"
DEFERRED = "deferred"
GAP = "gap"  # active but flagged with 🟡 inline


def classify_row(rest: str) -> str:
    """Classify a requirement-row remainder (everything after the ID cell)."""
    if "❌ (dropped:" in rest or "dropped:" in rest.lower() and "❌" in rest:
        return DROPPED
    if "❌ deferred" in rest:
        return DEFERRED
    return ACTIVE


def has_gap_marker(rest: str) -> bool:
    return "🟡" in rest


# ----------------------------------------------------------------------------
# Vendor classification
# ----------------------------------------------------------------------------

def is_vendor(path: Path) -> bool:
    for v in VENDOR_ROOTS:
        try:
            path.relative_to(v)
            return True
        except ValueError:
            continue
    return False


# ----------------------------------------------------------------------------
# Parsers
# ----------------------------------------------------------------------------

def parse_requirements(req_path: Path):
    """Parse requirements.md. Returns dict[id -> {title, status, has_gap}]."""
    reqs = {}
    text = req_path.read_text(encoding="utf-8")
    for line in text.splitlines():
        m = REQ_ROW_RE.match(line)
        if not m:
            continue
        rid = m.group(1)
        rest = m.group(2)
        cells = [c.strip() for c in rest.split("|")]
        title = cells[0] if cells else ""
        reqs[rid] = {
            "title": title,
            "status": classify_row(rest),
            "has_gap": has_gap_marker(rest),
        }
    return reqs


def walk_tags(roots):
    """Walk source roots, return (impls, verifs, all_refs).

    impls / verifs : dict[id -> list[Path]]
    all_refs       : list[(id, Path, kind)] — used for unknown-ID detection.
    """
    impls = {}
    verifs = {}
    all_refs = []
    seen_files = set()
    for root in roots:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            if path.suffix not in (".c", ".h", ".cpp", ".hpp"):
                continue
            if path.resolve() == SELF_PATH:
                continue
            if path in seen_files:
                continue
            seen_files.add(path)
            try:
                src = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            for m in IMPL_RE.finditer(src):
                for rid in ID_RE.findall(m.group(1)):
                    impls.setdefault(rid, []).append(path)
                    all_refs.append((rid, path, "implements"))
            for m in VERIFY_RE.finditer(src):
                for rid in ID_RE.findall(m.group(1)):
                    # §5.3: vendor verifier refs are dropped on the floor.
                    if not is_vendor(path):
                        verifs.setdefault(rid, []).append(path)
                    all_refs.append((rid, path, "verifies"))
    return impls, verifs, all_refs


# ----------------------------------------------------------------------------
# Output
# ----------------------------------------------------------------------------

def rel(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def render_files(paths) -> str:
    if not paths:
        return "—"
    uniq = sorted({rel(p) for p in paths})
    return "<br>".join(f"`{p}`" for p in uniq)


def emit_trace_md(reqs, impls, verifs, out_path: Path) -> None:
    total = len(reqs)
    active_ids = [r for r, m in reqs.items() if m["status"] == ACTIVE]
    dropped_ids = [r for r, m in reqs.items() if m["status"] == DROPPED]
    deferred_ids = [r for r, m in reqs.items() if m["status"] == DEFERRED]
    gap_ids = [r for r, m in reqs.items() if m["has_gap"] and m["status"] == ACTIVE]
    impl_count = sum(1 for r in active_ids if r in impls)
    verif_count = sum(
        1 for r in active_ids
        if r in verifs or (r in impls and all(is_vendor(p) for p in impls[r]))
    )

    lines = []
    lines.append("# Vayu firmware — traceability matrix")
    lines.append("")
    lines.append("> Generated by `tools/dev/trace.py`. Do not edit by hand.")
    lines.append("> Source of truth: `docs/reference/requirements.md`.")
    lines.append("")
    lines.append("## Summary")
    lines.append("")
    lines.append(f"- Total requirements: **{total}**")
    lines.append(f"- Active: **{len(active_ids)}** "
                 f"(of which {len(gap_ids)} carry an inline 🟡 gap marker)")
    lines.append(f"- Dropped: **{len(dropped_ids)}**")
    lines.append(f"- Deferred: **{len(deferred_ids)}**")
    lines.append(f"- Active with implementer: **{impl_count} / {len(active_ids)}**")
    lines.append(f"- Active with verifier (or verified-upstream): "
                 f"**{verif_count} / {len(active_ids)}**")
    lines.append("")
    lines.append("## Trace")
    lines.append("")
    lines.append("| ID | Status | Title | Implementers | Verifiers |")
    lines.append("|----|--------|-------|--------------|-----------|")
    for rid in sorted(reqs.keys()):
        meta = reqs[rid]
        st = meta["status"]
        if meta["has_gap"] and st == ACTIVE:
            st_cell = "🟡 active (gap)"
        elif st == DROPPED:
            st_cell = "dropped"
        elif st == DEFERRED:
            st_cell = "deferred"
        else:
            st_cell = "active"

        if st == DROPPED:
            impl_cell = "—"
            verif_cell = "—"
        else:
            i = impls.get(rid, [])
            v = verifs.get(rid, [])
            impl_cell = render_files(i)
            if not v and i and all(is_vendor(p) for p in i):
                verif_cell = "verified-upstream"
            else:
                verif_cell = render_files(v)

        title = meta["title"].replace("|", "\\|")
        # Truncate title for the trace table.
        if len(title) > 60:
            title = title[:57] + "…"
        lines.append(f"| `{rid}` | {st_cell} | {title} | {impl_cell} | {verif_cell} |")

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ----------------------------------------------------------------------------
# CI gate
# ----------------------------------------------------------------------------

def run_check(reqs, impls, verifs, all_refs, strict: bool) -> int:
    """Return exit code. Phase 1: unknown ID fails; impl/verif warn-only.
    Phase 5 (--strict): impl/verif also fail."""
    failures = 0
    warnings = 0

    def fail(msg):
        nonlocal failures
        print(f"FAIL: {msg}", file=sys.stderr)
        failures += 1

    def warn(msg):
        nonlocal warnings
        print(f"WARN: {msg}", file=sys.stderr)
        warnings += 1

    # 1. Unknown ID — referenced in code but not in requirements.md.
    unknown_seen = set()
    for rid, path, kind in all_refs:
        if rid in reqs:
            continue
        key = (rid, rel(path), kind)
        if key in unknown_seen:
            continue
        unknown_seen.add(key)
        fail(f"unknown ID {rid} referenced at {rel(path)} (@{kind})")

    # 2. Missing implementer / verifier for active requirements.
    for rid in sorted(reqs.keys()):
        meta = reqs[rid]
        if meta["status"] != ACTIVE:
            continue
        impl_paths = impls.get(rid, [])
        verif_paths = verifs.get(rid, [])

        if not impl_paths:
            msg = f"missing implementer: {rid}"
            (fail if strict else warn)(msg)

        if not verif_paths:
            # §5.3 vendor exemption: vendor-only implementer ⇒ verified-upstream
            if impl_paths and all(is_vendor(p) for p in impl_paths):
                continue
            msg = f"missing verifier: {rid}"
            (fail if strict else warn)(msg)

    print(
        f"trace summary: {failures} failure(s), {warnings} warning(s); "
        f"{len(reqs)} requirements total.",
        file=sys.stderr,
    )
    return 1 if failures else 0


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        prog="trace.py",
        description="Traceability gate (CONV-03).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument(
        "--check",
        action="store_true",
        help="CI mode. Exits non-zero on unknown ID. "
             "Missing implementer/verifier are warnings unless --strict.",
    )
    ap.add_argument(
        "--strict",
        action="store_true",
        help="In --check mode, missing implementer or verifier for an "
             "active requirement is treated as failure (Phase 5 mode).",
    )
    args = ap.parse_args()

    reqs = parse_requirements(REQUIREMENTS_MD)
    impls, verifs, all_refs = walk_tags(OWNED_ROOTS + VENDOR_ROOTS)

    # Always regenerate the trace matrix.
    emit_trace_md(reqs, impls, verifs, TRACE_MD)

    if not args.check:
        print(
            f"wrote {rel(TRACE_MD)} "
            f"({len(reqs)} requirements, "
            f"{len(impls)} implemented IDs, "
            f"{len(verifs)} verified IDs).",
        )
        return 0

    return run_check(reqs, impls, verifs, all_refs, strict=args.strict)


if __name__ == "__main__":
    sys.exit(main())
