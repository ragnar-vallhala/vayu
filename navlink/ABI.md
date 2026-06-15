# NavLink v2 — ABI & integration guide

This is the contract between the three trees that speak NavLink v2: the **FC
firmware** (C, STM32), the **Navigator / GCS** (C++), and **tools** (Python,
e.g. the autotuner). All three generate their codecs from one source —
`dialect.json` — so they cannot drift. This document is how you *use* that
codec: what is ABI-stable, the generated API, the frame layer you supply, and
worked integration for each side.

Normative wire spec: [`../docs/analysis/navlink-v2-spec.md`](../docs/analysis/navlink-v2-spec.md).
Generator & dialect: [`README.md`](README.md). Link emulator: [`sim/README.md`](sim/README.md).

---

## 1. The three layers

```
        dialect.json                         ← single source of truth (you edit this)
            │  generate.py
            ▼
  ┌───────────────────────┐
  │  generated codec       │  msgid + CRC_EXTRA, wire/aligned structs,
  │  (DO NOT EDIT)         │  pack/unpack, to_aligned/from_aligned, info table
  └───────────────────────┘
            ▲                                ← ABI boundary (this doc)
  ┌───────────────────────┐
  │  your transport layer  │  framing (sync+header+CRC), seq, dispatch, UART/UDP,
  │  (you write, per tree) │  time-sync, security — NONE of this is generated
  └───────────────────────┘
```

**What is generated** (and ABI-stable): per message — the `msgid`, the
`CRC_EXTRA` layout signature, a packed **wire struct**, a naturally-aligned
**application struct**, `pack`/`unpack`/`to_aligned`/`from_aligned`, and a
`msgid → {size, crc_extra, name}` lookup table.

**What is *not* generated** (you provide it once per tree): the **frame** around
a payload (sync byte, 10-byte header, CRC-16 trailer, truncation), the byte-stream
**resync/parser**, the **dispatch** from `msgid` to your handler, and the
optional time-sync / security layers (spec §10–§11). `sim/frame.py` is a working
Python reference for the frame layer; the spec §3–§4 is normative.

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

The header is `extern "C"`-guarded, so the GCS compiles the same `.c` as C++.

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
```

---

## 4. The frame you must build (spec §3–§4)

The codec moves *payloads*. A frame wraps a payload:

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

Minimal C encoder/parser (illustrative — adapt to your UART/UDP):

```c
#include "navlink_msgs.h"

static uint16_t frame_crc(const uint8_t *hdr9, const uint8_t *pay, uint8_t n, uint8_t crc_extra) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < 9;  i++) navlink_crc_accumulate(hdr9[i], &crc);   /* header bytes 1..9 */
    for (int i = 0; i < n;  i++) navlink_crc_accumulate(pay[i],  &crc);
    navlink_crc_accumulate(crc_extra, &crc);                              /* seed last (§4.2) */
    return crc;
}

size_t navlink_frame(uint8_t *out, uint32_t msgid, const uint8_t *pay, size_t paylen,
                     uint8_t seq, uint8_t sysid, uint8_t compid) {
    while (paylen && pay[paylen - 1] == 0) paylen--;          /* trailing-zero truncation §5.6 */
    const navlink_msg_info_t *mi = navlink_msg_info(msgid);
    out[0]=0x56; out[1]=0x02; out[2]=(uint8_t)paylen; out[3]=0; out[4]=seq;
    out[5]=sysid; out[6]=compid; out[7]=msgid; out[8]=msgid>>8; out[9]=msgid>>16;
    memcpy(out + 10, pay, paylen);
    uint16_t crc = frame_crc(out + 1, pay, (uint8_t)paylen, mi->crc_extra);
    out[10 + paylen] = crc & 0xFF; out[11 + paylen] = crc >> 8;
    return 12 + paylen;
}
```

A receiver: scan for `0x56`, check `version==0x02`, read `payload_len`, look up
`navlink_msg_info(msgid)` (unknown ⇒ drop, framing preserved via `payload_len`),
recompute the CRC with `mi->crc_extra`, compare, then `unpack`. `sim/frame.py`
shows the same in Python and the test harness pins the conformance vectors
(spec §16).

> v1/v2 coexistence: demux on byte 1 — `(b1 & 0x0F)==1` ⇒ legacy v1, `b1==0x02`
> ⇒ v2 (spec §3.4). `IFLAG_SIGNED/ENCRYPTED/CRC32/FRAGMENTED` and time-sync are
> spec features layered on top of this frame; not provided by the codec.

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

**RX path** (parser → dispatch → handler):

```c
void on_frame(uint32_t msgid, const uint8_t *pay, size_t len) {
    switch (msgid) {
    case NAVLINK_MSGID_CMD_SET_PID: {
        navlink_cmd_set_pid_wire_t w; navlink_cmd_set_pid_unpack(&w, pay, len);
        navlink_cmd_set_pid_t c;      navlink_cmd_set_pid_to_aligned(&c, &w);
        apply_pid(c.axis, c.kp, c.ki, c.kd, c.kff);     /* aligned struct: do real math here */
        send_command_ack(NAVLINK_MSGID_CMD_SET_PID, c.req_seq, NAVLINK_COMMAND_RESULT_ACCEPTED);
        break;
    }
    /* ... other commands ... */
    default: break;                                     /* unknown id already CRC-validated away */
    }
}
```

**TX path** (emit telemetry from a task):

```c
void emit_attitude(float roll, float pitch, float yaw) {
    navlink_attitude_euler_t a = { .roll = roll, .pitch = pitch, .yaw = yaw };
    navlink_attitude_euler_wire_t w; navlink_attitude_euler_from_aligned(&w, &a);
    static uint8_t buf[NAVLINK_WIRE_SIZE_ATTITUDE_EULER]; /* or a shared TX scratch buffer */
    size_t n = navlink_attitude_euler_pack(buf, &w);
    uint8_t frame[12 + sizeof buf];
    size_t fn = navlink_frame(frame, NAVLINK_MSGID_ATTITUDE_EULER, buf, n, tx_seq++, FC_SYSID, FC_COMPID);
    uart_write(frame, fn);
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

The generated `navlink_msg_table` (sorted, binary-searchable via
`navlink_msg_info`) gives you `wire_size`/`crc_extra` for a generic parser
without a giant switch; the `switch` above is just the handler fan-out.

---

## 6. Navigator / GCS (C++) integration

The GCS compiles the **same generated `.c`** (the header is `extern "C"`), or
uses the Python codec for tooling. Decode → dispatch → update model; build →
frame → send for commands.

```cpp
extern "C" {
#include "navlink_msgs.h"
}

void Link::onFrame(uint32_t msgid, const uint8_t *pay, size_t len) {
    switch (msgid) {
    case NAVLINK_MSGID_ATTITUDE_EULER: {
        navlink_attitude_euler_wire_t w; navlink_attitude_euler_unpack(&w, pay, len);
        navlink_attitude_euler_t a;      navlink_attitude_euler_to_aligned(&a, &w);
        vehicle.setAttitude(a.roll, a.pitch, a.yaw);          // feed the UI model
        break;
    }
    case NAVLINK_MSGID_COMMAND_ACK: {
        navlink_command_ack_wire_t w; navlink_command_ack_unpack(&w, pay, len);
        navlink_command_ack_t k;      navlink_command_ack_to_aligned(&k, &w);
        ackPending(k.command, k.req_seq, k.result);           // correlate to the sent command
        break;
    }
    default: break;
    }
}

void Link::sendSetPid(uint8_t axis, float kp, float ki, float kd) {
    navlink_cmd_set_pid_t c = { .target_sys = FC_SYSID, .target_comp = FC_COMPID,
                                .req_seq = nextReqSeq(), .controller = 0, .axis = axis,
                                .kp = kp, .ki = ki, .kd = kd, .kff = 0.f };
    navlink_cmd_set_pid_wire_t w; navlink_cmd_set_pid_from_aligned(&w, &c);
    uint8_t pay[NAVLINK_WIRE_SIZE_CMD_SET_PID], frame[12 + sizeof pay];
    size_t n  = navlink_cmd_set_pid_pack(pay, &w);
    size_t fn = navlink_frame(frame, NAVLINK_MSGID_CMD_SET_PID, pay, n,
                              txSeq_++, GCS_SYSID, GCS_COMPID);
    transport_.write(frame, fn);                              // serial or UDP
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
cmd   = nl.CmdArm(target_sys=1, target_comp=1, req_seq=7, force=0)
frame = build_frame(nl.CmdArm.MSGID, cmd.pack(), nl.CmdArm.CRC_EXTRA)   # see sim/frame.py
```

The autotuner and analysis tools import the same module; `MSGID_TO_CLASS` lets a
generic logger decode any captured frame.

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
