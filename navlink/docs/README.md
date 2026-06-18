# NavLink Documentation

NavLink is the Vayu **wire protocol and generated codec** — the framing, message
catalog, and serialization shared between the firmware and the ground control
station. The machine-readable contract and the generated codec live **outside**
this docs tree at the repository root:

- [`../dialect.json`](../dialect.json) — the canonical message/field/enum dialect.
- [`../ABI.md`](../ABI.md) — the frozen on-wire ABI (msgids, CRC_EXTRA, layouts).
- [`../generate.py`](../generate.py) — emits the codec (`generated/`) from the dialect.

The docs here are organized into four lifecycle layers, matching the global
documentation taxonomy in [`../../docs/README.md`](../../docs/README.md):

- **[reference/](reference/)** — architecture explainers and the formal,
  normative wire spec. A living contract: edited in place, never deleted.
- **[plans/](plans/)** — active, in-flight plans. Deleted once shipped.
- **[journal/](journal/)** — persistent, time-ordered record (design records,
  migration history) kept to gauge trajectory. Never deleted.
- **[scratch/](scratch/)** — pre-planning thought and studies. Deleted once
  shipped.
