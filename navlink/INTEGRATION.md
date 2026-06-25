# NavLink v2 — integrating across the FC firmware and the GCS

This is the migration study: how the generated NavLink v2 codec lands in the two
trees that ship today — the **FC firmware** (`src/comm/`, C11) and the
**Navigator / GCS** (`navigator/src/protocol/`, C++17 / Qt6). [`ABI.md`](ABI.md)
is the *contract* (generated API + rules); this document is the *plan* (what to
replace, in what order, and the decisions it forces).

> TL;DR — the firmware and GCS speak a **v1 frame** that differs from the
> generated **v2 frame** in header, msgid width, and CRC. So this is a
> wire-format migration, not a drop-in. It can be done incrementally because
> both framings start with `0x56` and disambiguate cleanly on byte 1.

---

## 1. The two frames are not the same wire

| | shipping today (v1) | generated codec (v2) |
|---|---|---|
| Header | 8 B: `sync, ver/type, len, device_id, timestamp(u32)` | 10 B: `sync, ver, len, incompat_flags, seq, sysid, compid, msgid(u24)` |
| Message id | **4 bits** (upper nibble of byte 1 → 16 types max) | **24 bits** (`0x000000–0x7FFFFF` core) |
| Version | low nibble of byte 1 = `0x1` | byte 1 == `0x02` |
| CRC | **CRC-32**, STM32 poly `0x04C11DB7`, MSB-first, over header+payload | **CRC-16/MCRF4XX** seeded with per-message `CRC_EXTRA` |
| Loss/seq | none | `seq` per sender |
| Per-frame timestamp | `u32` in every header | dropped; lives in payloads (`sample_time_us`) |
| Trailing-zero trunc | no | yes (spec §5.6) |
| Max frame | 8 + 256 + 4 = 268 B | ≤ 267 B (`NAVLINK_MAX_FRAME`) |

Porting therefore swaps the framing, the CRC, and the dispatch — **and**
reshapes a few payloads (§5).

### Source touch-points today

**Firmware** (`src/comm/`, `include/comm/comm_types.h`, `include/comm/perf_packet.h`):
`PROTOCOL_VERSION 0x1`, `SYNC_BYTE 0x56`; `packet_t` = 8-B header + `payload[256]`
+ `crc32`; `serializer.c::send_packet()` encodes; `deserializer.c::deserializer_feed()`
is a 4-state machine; `comm_processor.c::comm_processor_dispatch()` is the
`switch (packet_type)` with a nested `switch (cmd_id)`; `channel.c` is the
UART ping-pong/DMA transport; `telemetry_task.c` / `perf_telemetry.c` are the TX
producers.

**GCS** (`navigator/src/protocol/`): `DroneProtocol::parseBuffer()` does sync +
CRC32 + dispatch and emits Qt signals; `PacketDecoder` unpacks payloads into a
`std::variant`; `CommandCodec` builds outbound `0x3` frames; `PacketDissector`
is the analyzer GUI's hand-coded field map; `TelemetryEngine` owns the parser +
`SerialManager`/`UdpManager` transports and writes `VehicleState`.

---

## 2. Why coexistence works (the lever for incremental migration)

Both frames begin `0x56`, and **byte 1 disambiguates with no collision**:

- `(b1 & 0x0F) == 1` ⇒ v1 (the version is the low nibble; type is the high nibble)
- `b1 == 0x02`       ⇒ v2

The GCS already drops `version != 0x1` (`DroneProtocol.cpp:45`), and the v2 parser
ignores anything whose byte 1 isn't `0x02`. So **both stacks can run on the same
UART/UDP byte stream at once**: demux on byte 1, feed each frame to its parser.
That makes a message-by-message migration possible — no flag day.

---

## 3. What the v2 codec replaces

### Firmware (C)

| today | replaced by |
|-------|-------------|
| `deserializer.c` 4-state machine | `navlink_parser_push()` (resync + CRC + dispatch) |
| `serializer.c::send_packet()` | `navlink_<msg>_encode()` (builds the whole frame) |
| `comm_processor_dispatch()` `switch` + nested `cmd_id` switch | `navlink_handlers_t` table — one callback per msgid; commands become distinct msgids so the nested switch disappears |
| `utils_try_compute_crc32` | generated `navlink_crc_accumulate` (CRC-16) |
| `channel.c` UART ping-pong/DMA | **kept** — this is the transport you supply |

RX wiring (the ABI §5 shape):

```c
static navlink_parser_t g_parser;
static const navlink_handlers_t HANDLERS = {
    .on_cmd_set_pid = on_cmd_set_pid, .on_cmd_arm = on_cmd_arm,
    .on_time_sync = on_time_sync, /* unset slots are ignored */ };

void comm_init(void) { navlink_parser_init(&g_parser); }
void comm_rx_bytes(const uint8_t *b, size_t n) {     /* from the UART RX path */
    navlink_parser_push(&g_parser, &HANDLERS, b, n);
}
```

TX wiring (replaces a `send_packet` call):

```c
navlink_attitude_euler_t a = { .roll = r, .pitch = p, .yaw = y };
uint8_t f[NAVLINK_MAX_FRAME];
size_t n = navlink_attitude_euler_encode(f, &a, tx_seq++, FC_SYSID, FC_COMPID);
write_channel(g_telemetry_channel, f, n);            /* keep the existing transport */
```

Firmware constraints from the ABI still apply: no dynamic allocation; **never
take `&` of a packed wire-struct member** (read by value or via `to_aligned`);
the FC never invents wall-clock time — high-rate messages carry
`sample_time_us`, wall-clock comes only from `TIME_SYNC`.

### GCS (C++)

The GCS compiles the **same generated `.c`** through the `extern "C"` header.

| today | replaced by |
|-------|-------------|
| `DroneProtocol::parseBuffer()` sync/CRC/dispatch | `navlink_parser_push()` with handler thunks |
| handler thunks | emit the **existing Qt signals** so `TelemetryEngine` / `VehicleState` / UI are untouched |
| `PacketDecoder` variant unpack | generated `unpack` + `to_aligned` |
| `CommandCodec` builders | `navlink_<cmd>_encode()` |
| `PacketDissector` (analyzer field map) | regenerate from `navlink_msg_table` later — **lowest priority**, leave on v1 until last |
| `SerialManager` / `UdpManager` | **kept** — transport |

```cpp
extern "C" { #include "navlink_msgs.h" }

static void onAttitude(void *ctx, const navlink_frame_hdr_t *, const navlink_attitude_euler_t *a) {
    emit static_cast<Link*>(ctx)->attitudeReceived({a->roll, a->pitch, a->yaw}); // same signal as today
}

Link::Link() {
    navlink_parser_init(&parser_);
    handlers_ = {}; handlers_.ctx = this;          // member assignment: C++ has no out-of-order designated init
    handlers_.on_attitude_euler = onAttitude;
    handlers_.on_command_ack    = onAck;
}
void Link::onBytes(const uint8_t *b, size_t n) { navlink_parser_push(&parser_, &handlers_, b, n); }
```

---

## 4. Build wiring (Phase 0)

Generate into each tree as a build step so the checked-in output can never drift
from `dialect.json`:

```cmake
# both trees: regenerate before compiling
add_custom_command(
  OUTPUT  ${GEN}/c/navlink_msgs.c ${GEN}/c/navlink_msgs.h
  COMMAND python3 ${NAVLINK}/generate.py --lang c --out ${GEN}
  DEPENDS ${NAVLINK}/dialect.json ${NAVLINK}/generate.py)
```

- Firmware: add the two generated files to the `src/comm` source list; add `${GEN}/c`
  to includes. C11, `-Werror` clean (the generator output compiles under the
  firmware's `-Wall -Wextra`).
- GCS: add `navlink_msgs.c` to the `Navigator` target; it builds as C++ via the
  `extern "C"` guard. C++17/Qt6 unaffected.
- CI re-runs `generate.py` + `tests/run_tests.py` and fails if the tree differs
  (someone hand-edited generated code) or if C↔Python parity breaks.

---

## 5. Payload reshapings that need a decision (not mechanical)

1. **`SYSTEM_STATUS` (0x6) demultiplexes.** Today one type carries 8 origins
   (calibration / health / sys_state / flight_mode / est_perf / pid …) behind an
   `origin` byte. v2 gives each its own msgid. The origin `switch` on both ends
   collapses into distinct handlers / signals.
2. **`COMMAND` (0x3) demultiplexes** the same way: `cmd_id`-in-payload →
   distinct msgids (`CMD_ARM`, `CMD_SET_PID`, `CMD_SET_GYRO_LPF`,
   `CMD_SET_MOTOR_GEOMETRY`, `CMD_SET_FLIGHT_MODE`, `CMD_CALIBRATE_IMU`) — all
   already in the dialect.
3. **PERF fragmentation changes shape.** Firmware fragments rows via
   `perf_frag_hdr_t` (`section/index/count/seq`). v2 has **no array-of-struct
   type**, so the dialect models perf as **one message per row** correlated by
   `seq` (`PERF_GLOBAL`, `PERF_TASK`, `PERF_FIFO`, …). The
   fragment/reassembly code in `perf_telemetry.c` and in the GCS `PacketDecoder`
   is rewritten around per-row messages.
4. **Header timestamp goes away.** v1 stamped `u32` time on *every* frame; v2
   does not. Anything depending on per-frame FC time moves to a payload
   `sample_time_us` (the dialect's high-rate messages already carry one).

---

## 6. Time-sync — reconciled (done)

The just-shipped v1 time-sync (commit `a3d501a`: FC `time_sync_payload_t`, GCS
`TimeSyncEstimator`, packet `0xB`) and the dialect's old `TIME_REFERENCE` /
`TIME_REFERENCE_ACK` pair were two different designs. **The dialect now matches
the shipped design**: the pair is replaced by a single `TIME_SYNC` (msgid 10),
byte-identical to `time_sync_payload_t`:

```
role(u8, enum time_sync_role) seq(u8) _pad[2]
t1_gcs_tx(u64 ms) t2_fc_rx(u64 ms) t3_fc_tx(u64 ms) commanded_offset_ms(i32)   = 32 B
```

`role` is `REQUEST` (GCS→FC, stamps t1) or `RESPONSE` (FC→GCS, echoes t1, adds
t2/t3); the GCS captures t4 locally. So when this service migrates to v2, the
existing `TimeSyncEstimator` math and the FC's offset discipline carry over
unchanged — only the framing swaps. **Follow-up:** `docs/reference/navlink-v2-spec.md`
§10/§14 still describes the old two-message form and should be updated to the
single `TIME_SYNC` message to keep the normative spec consistent with the dialect.

---

## 7. Migration phases (each independently shippable + testable)

- **Phase 0 — build wiring (§4).** Generate into both trees; CI parity gate. No
  behavior change.
- **Phase 1 — dual parser (§2).** Add the byte-1 demux in front of both RX paths;
  v1 frames to the old code, v2 to `navlink_parser_push`. Still no behavior change.
- **Phase 2 — FC→GCS telemetry**, one msgid at a time (start `HEARTBEAT`,
  `ATTITUDE_EULER`): FC emits v2; GCS handler emits the same Qt signal it does
  today; drop the v1 emitter for that type once the GCS handler is live.
- **Phase 3 — GCS→FC commands** (`CommandCodec` → `*_encode`; FC handlers per cmd
  msgid).
- **Phase 4 — reshapings (§5) + time-sync (§6)**, then retire v1 framing: delete
  `deserializer.c` / `serializer.c` / `PacketDecoder`, regenerate `PacketDissector`
  from `navlink_msg_table`.

Each phase is validated against `tests/run_tests.py` (C↔Python byte + frame
parity, spec §16 vectors) and the link emulator in `sim/`.

---

## 8. Open decisions

- **Sysid/compid scheme.** v1 had a single `device_id`; v2 splits sender identity
  into `sysid`/`compid`. Pick the FC's and GCS's ids (and any companion-computer
  ids) before Phase 2.
- **Seq policy.** One `seq` counter per sender (FC, GCS) rolling 0..255; the GCS's
  loss/RTT bookkeeping keys off it (reference impl in `sim/endpoint.py`).
- **PERF reshape ownership.** Whether the per-row v2 perf messages keep the
  `report-id = seq` correlation the firmware already produces, or move to the
  frame `seq` — decide with whoever owns `perf_telemetry.c`.
- **Spec doc** §10/§14 update for `TIME_SYNC` (§6 above).
- **Security/time-sync gate.** Until sync succeeds the FC should reject
  secured/command frames (spec §10.5) — policy in *your* dispatch, not the codec.
