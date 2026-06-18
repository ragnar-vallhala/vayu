# NavLink Reference

Architecture explainers and the formal, normative wire spec for NavLink. This is
a living contract: edited in place as the protocol evolves, never deleted.

- [`navlink-v2-spec.md`](navlink-v2-spec.md) — the **authoritative** normative
  NavLink v2 wire spec (framing, 24-bit msgids, CRC_EXTRA, field encoding).
- [`messages/`](messages/) — per-message human reference catalog (one page per
  message family); defer to the dialect for exact field layouts.

The machine-readable dialect and generated codec live at the repository root:
[`../../dialect.json`](../../dialect.json), [`../../ABI.md`](../../ABI.md), and the
[`../../generate.py`](../../generate.py) codec generator.
