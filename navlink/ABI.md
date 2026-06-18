# NavLink v2 — ABI & integration guide

This is the contract between the three trees that speak NavLink v2: the **FC
firmware** (C, STM32), the **Navigator / GCS** (C++), and **tools** (Python,
e.g. the autotuner). All three generate their codecs from one source —
`dialect.json` — so they cannot drift. This document is how you *use* that
codec: what is ABI-stable, the generated API, the frame layer you supply, and
worked integration for each side.

Normative wire spec: [`docs/reference/navlink-v2-spec.md`](docs/reference/navlink-v2-spec.md).
Generator & dialect: [`README.md`](README.md). Link emulator: [`sim/README.md`](sim/README.md).

---

## 1. The three layers

```
        dialect.json                         ← single source of truth (you edit this)
            │  generate.py
            ▼
  ┌───────────────────────┐
  │  generated codec       │  msgid + CRC_EXTRA, wire/aligned structs,
  │  + framing             │  pack/unpack, converters, info table, per-message
  │  (DO NOT EDIT)         │  encoder, incremental parser + handler table
  └───────────────────────┘
            ▲                                ← ABI boundary (this doc)
  ┌───────────────────────┐
  │  your glue             │  UART/UDP I/O, seq policy, handler bodies,
  │  (you write, per tree) │  time-sync, security
  └───────────────────────┘
```

**What is generated** (and ABI-stable): per message — the `msgid`, the
`CRC_EXTRA` layout signature, a packed **wire struct**, a naturally-aligned
**application struct**, `pack`/`unpack`/`to_aligned`/`from_aligned`, the
`msgid → {size, crc_extra, name}` lookup, a per-message **encoder**
(`navlink_<msg>_encode`) that builds a full frame, and an incremental **parser**
that resyncs, validates the CRC, and dispatches to a user-supplied **handler
table** (`navlink_handlers_t`).

**What you still write** (once per tree): the byte **transport** (UART / UDP
read+write), the `seq` counter policy, the **handler bodies** themselves, and the
optional time-sync / security layers (spec §10–§11). The wire format, CRC,
framing and dispatch are no longer hand-rolled per tree — that's the whole point.

---

## 2. What "ABI" means here

The stable contract is the **wire image** plus the **generated C struct layout**:

- A message's `msgid` and `CRC_EXTRA` together identify its exact field layout.
  If two peers disagree on layout, their `CRC_EXTRA` differ and the frame CRC
  fails — a mismatch is *detected*, never silently mis-decoded (spec §4.3).
- The **packed wire struct** matches the payload byte-for-byte (little-endian,
  IEEE-754), so `pack`/`unpack` are a single `memcpy`.
- **Extension fields** (`"extension": true`) are appended after the core fields
  and excluded from `CRC_EXTRA`, so adding one does **not** break old peers
  (spec §5.5). Old→new: absent fields read as 0. New→old: extra bytes ignored.

Rules that keep the ABI intact:

1. **Never edit generated files** — change `dialect.json` and regenerate.
2. **Never reorder or retype existing fields**, and never insert among them.
   Only *append*, and append growth fields as `extension: true`.
3. **msgids are forever.** Core ids live in `0x000000–0x7FFFFF`; vendor/experimental
   in `0x800000–0xFFFFFF` (spec §9).
4. Regenerate all three trees from the same `dialect.json` in the same change.

---

## 3. Generated API surface

### C (`generated/c/navlink_msgs.{h,c}`)

For a message `FOO` (struct/function names use the lower-cased name):

| symbol | meaning |
|--------|---------|
| `NAVLINK_MSGID_FOO` | the 24-bit message id (`uint`) |
| `NAVLINK_CRC_EXTRA_FOO` | 1-byte layout signature |
| `NAVLINK_WIRE_SIZE_FOO` | full packed payload size in bytes |
| `navlink_foo_wire_t` | packed wire struct (exact bytes; read members by value only) |
| `navlink_foo_t` | naturally-aligned struct for application code |
| `size_t navlink_foo_pack(uint8_t *buf, const navlink_foo_wire_t *w)` | wire→bytes (one memcpy, with trailing-zero truncation) → length |
| `void navlink_foo_unpack(navlink_foo_wire_t *w, const uint8_t *buf, size_t len)` | bytes→wire (zero-fills short buffers, clamps long ones) |
| `void navlink_foo_to_aligned(navlink_foo_t *a, const navlink_foo_wire_t *w)` | wire→aligned (field-wise) |
| `void navlink_foo_from_aligned(navlink_foo_wire_t *w, const navlink_foo_t *a)` | aligned→wire |

Enums are `navlink_<enum>_e` with members `NAVLINK_<ENUM>_<ENTRY>`.
Cross-message helpers:

```c
const navlink_msg_info_t *navlink_msg_info(uint32_t msgid);  /* {msgid,wire_size,crc_extra,name} or NULL */
extern const navlink_msg_info_t navlink_msg_table[NAVLINK_MSG_COUNT]; /* sorted by msgid */
static inline void navlink_crc_accumulate(uint8_t b, uint16_t *crc);  /* CRC-16/MCRF4XX, spec §4.1 */
```

Framing (one set, all messages):

```c
size_t navlink_foo_encode(uint8_t *out, const navlink_foo_t *msg,        /* aligned struct → full frame */
                          uint8_t seq, uint8_t sysid, uint8_t compid);
size_t navlink_frame(uint8_t *out, uint32_t msgid, const uint8_t *payload, size_t len,
                     uint8_t seq, uint8_t sysid, uint8_t compid);        /* low-level: pre-packed payload */

typedef struct { uint32_t msgid; uint8_t seq, sysid, compid, incompat_flags; } navlink_frame_hdr_t;
typedef struct navlink_handlers {                /* populate the slots you want */
    void *ctx;
    void (*on_foo)(void *ctx, const navlink_frame_hdr_t *hdr, const navlink_foo_t *msg);
    /* ...one per message... */
    void (*on_unknown)(void *ctx, uint32_t msgid, const uint8_t *payload, size_t len);
    void (*on_crc_error)(void *ctx, uint32_t msgid);
} navlink_handlers_t;

typedef struct { /* opaque */ } navlink_parser_t;
void navlink_parser_init(navlink_parser_t *p);
void navlink_parser_push(navlink_parser_t *p, const navlink_handlers_t *h,
                         const uint8_t *data, size_t n);   /* feed RX bytes; fires handlers */
```

`NAVLINK_MAX_FRAME` (267) bounds an encode buffer. The header is `extern "C"`-guarded,
so the GCS compiles the same `.c` as C++.

### Python (`generated/python/navlink_msgs.py`)

```python
import navlink_msgs as nl
m  = nl.AttitudeEuler(roll=0.1, pitch=0.0, yaw=1.2)
b  = m.pack()                       # bytes (truncation-free full payload)
m2 = nl.AttitudeEuler.unpack(b)     # zero-fills short input
nl.AttitudeEuler.MSGID, nl.AttitudeEuler.CRC_EXTRA, nl.AttitudeEuler.WIRE_SIZE
nl.MSGID_TO_CLASS[1026]             # -> AttitudeEuler
nl.CRC_EXTRA[1026]                  # -> crc_extra byte
nl.crc16(b)                         # CRC-16/MCRF4XX

frame = nl.encode(m, seq=0, sysid=1, compid=1)            # message -> full frame bytes
parser = nl.Parser(nl.Handlers(on_attitude_euler=lambda f, msg: print(f.seq, msg.roll)))
parser.push(frame)                                        # feed RX bytes; handlers fire
```

---

## 4. The frame (generated — spec §3–§4)

The frame the generated encoder/parser produce and consume:

```
 off size field
  0   1   sync = 0x56
  1   1   version = 0x02
  2   1   payload_len (after trailing-zero truncation)
  3   1   incompat_flags (0 for plain frames)
  4   1   seq (per-sender, rolls 0..255)
  5   1   sysid
  6   1   compid
  7   3   msgid (u24 little-endian)
 10   N   payload
10+N  2   CRC-16/MCRF4XX over header[1..9] ‖ payload, then CRC_EXTRA accumulated last
```

You no longer hand-write this: `navlink_<msg>_encode()` builds it, and
`navlink_parser_push()` resyncs on `0x56`, checks the version, validates the CRC
against the per-message `CRC_EXTRA`, and dispatches (unknown msgid ⇒ `on_unknown`,
bad CRC ⇒ `on_crc_error`, framing preserved via `payload_len`). The test harness
encodes in C and Python and asserts the frames are byte-identical, and pins the
spec §16 conformance vectors.

> v1/v2 coexistence: a peer that still speaks legacy v1 must demux on byte 1 —
> `(b1 & 0x0F)==1` ⇒ v1, `b1==0x02` ⇒ v2 (spec §3.4); the generated v2 parser
> ignores non-v2 frames. `IFLAG_SIGNED/ENCRYPTED/CRC32/FRAGMENTED` and time-sync
> are spec features layered on top; not provided by the codec.

---

## 5. FC firmware (C) integration

**Wire it into the build.** Point `generate.py --out` at the firmware tree (the
spec touch-points are `include/comm/` and `src/comm/`), or add a CMake step:

```cmake
add_custom_command(
  OUTPUT  ${GEN}/c/navlink_msgs.c ${GEN}/c/navlink_msgs.h
  COMMAND python3 ${CMAKE_SOURCE_DIR}/navlink/generate.py --lang c --out ${GEN}
  DEPENDS ${CMAKE_SOURCE_DIR}/navlink/dialect.json)
```

**RX path** — write a handler per command you accept, register them once, and
feed UART bytes to the generated parser:

```c
static void on_cmd_set_pid(void *ctx, const navlink_frame_hdr_t *hdr,
                           const navlink_cmd_set_pid_t *c) {
    apply_pid(c->axis, c->kp, c->ki, c->kd, c->kff);     /* aligned struct: real math here */
    send_command_ack(NAVLINK_MSGID_CMD_SET_PID, c->req_seq, NAVLINK_COMMAND_RESULT_ACCEPTED);
}

static const navlink_handlers_t HANDLERS = {
    .on_cmd_set_pid = on_cmd_set_pid,
    /* .on_cmd_arm = ..., .on_time_sync = ..., etc.  Unset slots are simply ignored. */
};

static navlink_parser_t g_parser;
void comm_init(void) { navlink_parser_init(&g_parser); }

void comm_rx_bytes(const uint8_t *buf, size_t n) {       /* call from your UART RX ISR/task */
    navlink_parser_push(&g_parser, &HANDLERS, buf, n);   /* resync + CRC check + fire on_* */
}
```

**TX path** (emit telemetry from a task) — one call builds the whole frame:

```c
void emit_attitude(float roll, float pitch, float yaw) {
    navlink_attitude_euler_t a = { .roll = roll, .pitch = pitch, .yaw = yaw };
    uint8_t frame[NAVLINK_MAX_FRAME];
    size_t n = navlink_attitude_euler_encode(frame, &a, tx_seq++, FC_SYSID, FC_COMPID);
    uart_write(frame, n);
}
```

**Firmware constraints (important):**

- **No dynamic allocation.** Structs are fixed-size; use static or stack buffers.
  A single-frame payload is ≤ 255 B; the frame ≤ 267 B.
- **Packed-struct access rule.** Read wire-struct members *by value*
  (`w.kp`) — the compiler emits unaligned-safe loads. **Never** take the address
  of a packed member (`&w.kp`); that is UB on Cortex-M. Use the `to_aligned`
  struct for anything that needs a pointer or arithmetic.
- **One clock domain.** High-rate messages carry `sample_time_us` (µs since
  boot) for intra-vehicle alignment; wall-clock timestamps come only from the
  GCS-driven sync (spec §6, §10) — the FC never invents wall-clock time.
- **Safety gate.** Until time-sync succeeds the FC is "unsynchronised" and must
  reject secured/command frames (spec §10.5) — that's policy in *your* dispatch,
  not the codec.

The parser + handler table replace the hand-written `if/else` on packet type and
the nested `if/else` on command id. `navlink_msg_table` /
`navlink_msg_info()` are still there if you need generic, msgid-driven tooling
(logging, relays) outside the handler model.

---

## 6. Navigator / GCS (C++) integration

The GCS compiles the **same generated `.c`** (the header is `extern "C"`), or
uses the Python codec for tooling. Decode → dispatch → update model; build →
frame → send for commands.

```cpp
extern "C" {
#include "navlink_msgs.h"
}

// Handler thunks: ctx is the Link*, so members are reachable.
static void onAttitude(void *ctx, const navlink_frame_hdr_t *, const navlink_attitude_euler_t *a) {
    static_cast<Link *>(ctx)->vehicle.setAttitude(a->roll, a->pitch, a->yaw);
}
static void onAck(void *ctx, const navlink_frame_hdr_t *, const navlink_command_ack_t *k) {
    static_cast<Link *>(ctx)->ackPending(k->command, k->req_seq, k->result);
}

Link::Link() {
    navlink_parser_init(&parser_);
    handlers_ = {};                 // zero all slots, then set the ones we handle
    handlers_.ctx = this;           // (member assignment avoids C++ designated-init ordering)
    handlers_.on_attitude_euler = onAttitude;
    handlers_.on_command_ack = onAck;
}

void Link::onBytes(const uint8_t *buf, size_t n) {            // from serial/UDP read
    navlink_parser_push(&parser_, &handlers_, buf, n);
}

void Link::sendSetPid(uint8_t axis, float kp, float ki, float kd) {
    navlink_cmd_set_pid_t c = { .target_sys = FC_SYSID, .target_comp = FC_COMPID,
                                .req_seq = nextReqSeq(), .controller = 0, .axis = axis,
                                .kp = kp, .ki = ki, .kd = kd, .kff = 0.f };
    uint8_t frame[NAVLINK_MAX_FRAME];
    size_t n = navlink_cmd_set_pid_encode(frame, &c, txSeq_++, GCS_SYSID, GCS_COMPID);
    transport_.write(frame, n);                              // serial or UDP
}
```

Services follow the same shape: commands are correlated to `COMMAND_ACK` by
`(command msgid, req_seq)`; the parameter dump uses `generation` for consistency;
PINGs measure RTT. The GCS owns link bookkeeping (seq-gap loss, RTT, throughput)
— see `sim/endpoint.py` for a complete reference implementation of all of it.

---

## 7. Tools (Python)

```python
import navlink_msgs as nl

frame = nl.encode(nl.CmdArm(target_sys=1, target_comp=1, req_seq=7, force=0))   # -> bytes
sock.send(frame)

parser = nl.Parser(nl.Handlers(on_command_ack=lambda f, m: print("ack", m.command, m.result)))
parser.push(sock.recv(4096))                                # fires handlers per frame
```

The autotuner and analysis tools import the same module; for ad-hoc decoding
`MSGID_TO_CLASS` lets a generic logger turn any payload into a typed object.

---

## 8. Compatibility & versioning checklist

- Change `dialect.json` → run `generate.py` → run `tests/run_tests.py` (it
  cross-checks C and Python byte-for-byte and pins the spec §16 vectors).
- Add fields only at the end; new growth fields are `extension: true`.
- A frame whose `msgid` is unknown, or whose `CRC_EXTRA` doesn't match, is
  dropped — handle that as "ignore", not "error".
- Advertise capabilities once at connect (`HEARTBEAT.capabilities` /
  `CAPABILITIES`); don't rely on an `incompat` feature a peer hasn't advertised.
- Commit the regenerated trees together; CI re-runs `generate.py` and fails if
  the checked-in output differs (i.e. someone hand-edited it).

## 9. Don't

- Don't edit anything under `generated/`.
- Don't take the address of a packed wire-struct member.
- Don't assume `payload_len` equals `WIRE_SIZE` — senders truncate trailing
  zeros; `unpack` restores them.
- Don't reuse a `(sysid, compid, seq)` for two secured frames (spec §3.5, §11.4).
