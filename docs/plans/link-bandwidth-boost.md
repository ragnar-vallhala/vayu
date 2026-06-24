# Link Bandwidth Boost (ESP8266 WiFi bridge) — pre-FTP

## Context

Before building the FTP/xfer substrate (`docs/plans/navlink-xfer-substrate.md`) we need a
**high-bandwidth link**. Today the GCS link goes FC USART6 → ESP8266 → WiFi/UDP → Navigator,
and it **starts dropping past ~150 packets/s ≈ 5–6 KB/s** — far below the UART's ~23 KB/s and
nowhere near WiFi capability. FTP on a 5–6 KB/s pipe is pointless, so we lift the link first.

**Decisions (with the user):**
- **Firmware-only on the existing ESP8266** (no hardware swap).
- **Target ~20–30 KB/s** sustained (≈ saturate the UART) — enough to pull blackbox logs in
  minutes and run FTP comfortably.
- **Keep UDP + app-layer acks** (the xfer cumulative-ack we already designed); no TCP path.

## Diagnosis

The numbers localise the bottleneck precisely:

- **The FC and UART are not the limit.** STM32 USART6 (APB2 @ 84 MHz) clocks to ~5 Mbaud; the
  channel path (2×2048 B ping-pong, `flush_task` @ 1 kHz, DMA2-S7) has ~2 MB/s of headroom
  (`src/comm/channel.c`). Baud is a single `#define` (`UART_BAUDRATE`, `include/vaios_app_config.h:20`).
- **The ESP8266 is the wall, two ways.** (1) **Small-packet datagram-rate ceiling** — at ~37 B
  per telemetry frame with the 8 ms flush, the bridge sends ~1 frame/datagram → ~150
  datagrams/s of tiny packets = the 5–6 KB/s you see. The cost is WiFi airtime + unicast
  MAC-retries *per datagram*, not bytes. (2) **Single-core stall** — the ESP8266's cooperative
  `loop()` blocks inside `udp.endPacket()` during WiFi TX/retries; while it blocks, UART RX is
  unserviced, so at high baud a burst overflows the 2048 B RX ring and desyncs the frame parser.
- **History confirms it.** Commit `97b2ae3 "lower telemetry UART baud to 230400 for the WiFi
  bridge"` dropped baud from 921600 → 230400 *for the ESP8266* — a buffering band-aid for
  failure mode (2), not a real fix. `../lora-link` still documents the wired link as 921600.

**Crucial nuance for FTP:** the 5–6 KB/s ceiling is a *small-packet* artifact. **FTP uses big
254 B frames**, which the bridge already coalesces ~5/datagram into MTU-sized datagrams — so
bulk transfer is **UART-byte-bound (~23 KB/s), not datagram-bound.** Raising the UART baud
therefore directly multiplies bulk throughput, and the datagram ceiling never binds for bulk.
(LoRa — the `../lora-link` sibling — is the opposite tool: long range, *low* bandwidth; not
relevant here.)

## The boost (firmware-only)

### 1. FC: raise the UART baud — `include/vaios_app_config.h:20`
`UART_BAUDRATE 230400 → 460800` (target). 460800 = ~46 KB/s raw, comfortable headroom over the
20–30 KB/s goal after framing overhead. 921600 is a stretch goal to validate (see §verification).
- Single-point change; `UART_BAUDRATE` feeds USART6 via `src/main.c:72` → `channel.c`.
- FC side needs **no other change**: at 46 KB/s the 2 KB ping-pong flushed @ 1 kHz drains ~46 B/ms
  against 2 KB/ms capacity — trivially within headroom. DMA2-S7 handles the higher bit clock.
- **Caveat:** any *direct USB-serial* GCS connection must match the new baud (Navigator serial
  transport). The **UDP path is baud-agnostic** (the ESP re-clocks), so the WiFi relay only needs
  the FC + ESP to agree.

### 2. ESP bridge: survive the higher baud — `tools/arduino/telemetry_bridge/telemetry_bridge.ino`
This is where the real work is — absorb the WiFi-stall-vs-UART-fill race so higher baud doesn't
overflow the RX ring:
- `#define FC_BAUD` → match the FC (460800).
- `Serial.setRxBufferSize(2048)` → **4096** (`setup()`). At 460800 (46 B/ms) a 4096 B ring absorbs
  ~89 ms of WiFi stall before loss (vs ~44 ms at 2048); at 921600 it still gives ~44 ms. This is
  the core fix for the historical instability.
- `uint8_t acc[2048]` → **4096** (the framing accumulator must hold the same burst).
- Keep the existing MTU coalescing + fill-path flush + 8 ms timer + `WIFI_NONE_SLEEP` + unicast —
  all already loss-optimised. **No protocol change** (stays UDP).
- Drain order is already correct (read UART into `acc` early in `loop()`); just make sure the
  bigger `acc` isn't capped by `MAX_UDP` logic (it isn't — `out[]` is the MTU buffer, `acc[]` is
  separate).
- **No bulk-mode flush needed:** the fill-path already flushes at MTU the instant a datagram fills,
  so sustained bulk (big frames) coalesces optimally without touching `FLUSH_MS`. The 8 ms timer
  only bounds latency for *sparse* telemetry — leave it.

### What this deliberately does NOT change
The **small-packet telemetry frame rate** stays datagram-bound (~150/s) — raising baud doesn't
lift it (it's airtime, not bytes). That's fine: the goal is the **bulk byte rate** for FTP, which
*is* baud-bound. If the live-telemetry packet rate ever needs lifting, that's an ESP32/airtime
problem, out of scope here.

## Why this hits 20–30 KB/s
Bulk frames (254 B) coalesce ~5/MTU-datagram → at 460800 the ESP forwards ~36 big datagrams/s
carrying ~46 KB/s — well inside the ESP8266's *big-datagram* WiFi throughput (hundreds of KB/s)
and far under its datagram-rate ceiling. The binding constraint becomes the UART at 46 KB/s, so
effective FTP throughput lands ~30–40 KB/s before loss, comfortably above target. The bigger RX
ring removes the burst-overflow that forced 230400.

## Verification (on real hardware — the deciding step)

1. **Throughput + loss harness.** Extend `tools/udp_telem_sniff.py` to report **effective bytes/s,
   frames/s, and frame-loss %** (gap detection by frame seq) over a rolling window — it already
   decodes the NavLink stream and prints per-message Hz, so add an aggregate bytes/s + a seq-gap
   counter.
2. **Synthetic bulk source on the FC.** A debug "blast" mode that streams max-size frames as fast
   as the channel accepts (or reuse the future xfer DOWNLOAD once it lands) to actually load the
   link — the fixed telemetry set can't reveal the bulk ceiling.
3. **Baud sweep.** Flash FC+ESP at **230400 / 460800 / 921600**; for each, run the bulk source and
   record effective KB/s and loss %. Pick the **highest baud with loss < ~1%** and ≥ 20–30 KB/s.
   Expectation: 460800 clean, 921600 marginal-but-maybe-OK with the 4 KB ring.
4. **Stall/burst stress.** Induce WiFi retries (range/interference) and confirm the 4 KB RX ring
   prevents frame desync (no RAW/UNK bursts at the GCS) — i.e. the historical 921600 failure is gone.
5. **Regression.** Confirm normal telemetry + the command uplink (ARM/PID/time-sync) still work at
   the new baud; the Navigator UDP transport needs no change, but a direct-serial GCS must be set to
   the new baud.

## Risks / notes
- **ESP8266 is still single-core** — 460800 is the safe sweet spot for the 20–30 KB/s target;
  921600 may work with the 4 KB ring but is the validation's job to confirm, not an assumption.
- **Wiring** — higher baud wants short FC↔ESP leads and a solid common ground (already direct 3.3 V,
  no level shifter); flaky wiring shows up as CRC errors at the GCS.
- **Direct-serial GCS** must match baud; the WiFi/UDP path does not.
- If 460800 still won't clear ~20 KB/s cleanly, the firmware-only ceiling is reached and the next
  lever is hardware (ESP32: dual-core kills the stall race, MB/s WiFi) — explicitly out of this
  scope per the decision, but the measured numbers from §verification tell us whether we need it.

## Files
- `include/vaios_app_config.h` — `UART_BAUDRATE` 230400 → 460800
- `tools/arduino/telemetry_bridge/telemetry_bridge.ino` — `FC_BAUD`, RX ring 4096, `acc[]` 4096
- `tools/arduino/telemetry_bridge/README.md` — document the new baud + buffer rationale
- `tools/udp_telem_sniff.py` — add bytes/s + frame-loss reporting
- (debug) a FC bulk-blast mode for load testing, or defer to the xfer DOWNLOAD path
