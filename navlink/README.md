# NavLink v2 dialect & code generator

The single source of truth for the Vayu link protocol (NavLink v2). The dialect
JSON defines every message; the generator emits matching codecs for firmware (C),
the GCS (C/C++), and tools (Python), so the three trees can never drift.

See `../docs/analysis/navlink-v2-spec.md` for the normative protocol spec.

## Files

| File | What |
|------|------|
| `dialect.json` | the message catalog — **edit this** to add/change messages (spec §7) |
| `dialect.schema.json` | JSON Schema the dialect must validate against (spec §7.4) |
| `generate.py` | the generator (stdlib only, no deps) |
| `generated/c/navlink_msgs.{h,c}` | generated C codec (firmware + GCS) |
| `generated/python/navlink_msgs.py` | generated Python codec (tools / autotuner) |

## Generate

```sh
python3 generate.py                 # both languages -> ./generated
python3 generate.py --lang c        # C only
python3 generate.py --out ../some/dir
```

On run the generator validates the dialect (msgid range/uniqueness, contiguous
field indices, ≤255-byte payloads) and self-checks the CRC (`crc16("123456789")
== 0x6F91`, spec §16.1) before writing anything.

## What's generated, per message

- `enum`s (C `enum` / Python `IntEnum`)
- a **packed wire struct** matching the payload byte-for-byte, and a
  **naturally-aligned struct** for application code (spec §8.2)
- `pack` / `unpack` (one `memcpy`, with zero-fill truncation, spec §5.6)
- `to_aligned` / `from_aligned` converters
- the `CRC_EXTRA` table and a `msgid → {name, size, crc_extra}` lookup
  (C: binary search; Python: dicts)

## Test

```sh
python3 tests/run_tests.py      # regenerate → Python tests → C tests → parity
python3 tests/test_codec.py     # Python unit tests only (no C compiler needed)
```

The runner (1) regenerates from `dialect.json`, (2) runs the Python unit tests,
(3) compiles + runs the C codec (`-Werror`), and (4) asserts C and Python agree
on `CRC_EXTRA` + wire size **and packed bytes for every message**. The byte
parity uses one deterministic per-field value scheme (`generate.canonical_values`)
shared by the Python tests and the generated C parity emitter
(`navlink_parity.c`), so both sides exercise the same non-trivial values —
signed negatives, `u64`, arrays, `u24`, `char[]`, floats.

Coverage maps onto the spec §16 conformance vectors: CRC-16 self-check (§16.1),
the `ATTITUDE_EULER` CRC_EXTRA golden value recomputed independently from the
§4.3 input string (§16.2), unsecured frame round-trip + trailing-zero truncation
(§16.3 / §5.6), version demux (§16.5), plus per-message value **and** frame
round-trips, type extremes, oversize/short/empty unpacks, extension-field CRC
stability, registry consistency, and negative dialect-validation cases
(dup msgid/name, non-contiguous indices, unknown type/enum, vendor range, >255 B).
36 Python tests + C asserts + all-message cross-language parity. Needs `cc`/`gcc`
(override with `CC=...`).

## Simulate a real link

`sim/` runs two host processes (an FC model and a GCS model) that talk NavLink v2
over UDP through a configurable impairment layer (latency, jitter, loss,
duplication, reorder, bit-corruption, bandwidth cap), and reports real-world link
stats (rates, throughput, CRC errors, loss estimate, RTT, command-ACK latency).

```sh
python3 sim/sim.py --scenario telemetry_radio
python3 sim/sim.py --latency-ms 50 --jitter-ms 20 --loss 0.05 --corrupt 0.01
```

See `sim/README.md` for scenarios and knobs.

## Adding a message

1. Append a message object to `dialect.json` with a `msgid` in the core half
   (`0x000000`–`0x7FFFFF`) and contiguous field `index` values from 0.
2. Run `generate.py`. Field order **is** wire order (spec §5.2) — don't reorder.
3. Append new fields with `"extension": true` so old/new peers still validate
   (the seed is computed over non-extension fields only, spec §4.3 / §5.5).

## Notes / known gaps

- NavLink has **no `f16` scalar type** (spec §5.1). `IMU_COMPRESSED` carries its
  10 half-float deltas as `u16` raw bit patterns; the application decodes them.
- `u24` (used by `COMMAND_ACK.command`) has no C/`struct` primitive: it is 3 wire
  bytes in the packed struct and a `uint32_t` in the aligned struct / Python int.
- The dialect currently covers every **wired** v1 message migrated to v2 plus the
  firmware observability (`PERF_*`, `EST_PERF`), calibration, command, and
  parameter services. Mission and file-transfer services are specified in the
  doc but not yet authored here.
