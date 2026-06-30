# Vayu FC — Feasibility: Max Telemetry Rate & VFS Logging Rate

**What this is:** a feasibility analysis of (A) the maximum *valid* telemetry rate
over the GCS link, and (B) the maximum SD/VFS logging rate — with the binding
constraint identified for each. Numbers from `src/comm/`, `src/logger/`,
`navlink/generated/`, the SDIO driver, and the target build config, 2026-06-23.
Builds on `resource-ownership-map.md` (C1/C3 contention) and
`memory-footprint-analysis.md`.

## TL;DR

| | Raw ceiling | **Binding constraint** | Current use | Verdict |
|---|---|---|---|---|
| **Telemetry** | 23.0 KB/s (UART6 230400 8N1) | **~150 packets/s** (ESP8266 WiFi relay), *not* bytes | ~6.9 KB/s, **~167 pkt/s** | **packet-saturated already**; bytes only 30% used |
| **VFS logging** | 10.5 MB/s (SDIO 21 MHz ×4-bit) | **sync-per-record latency** (~1–4 ms, ≤100 ms tail), *not* bandwidth | text/test only | **~250–1000 rec/s best case, caller-blocking**; unusable from hot loops as-is |

Both ceilings are **far below** their raw transport bandwidth — each is limited by
a *protocol/architecture* bottleneck, not the wire.

---

## A. Telemetry rate

### Transport
- **UART6, 230400 baud, 8N1** → 10 bits/byte → **23,040 B/s (22.5 KiB/s)**
  (`vaios_app_config.h:20`).
- But the link is a **WiFi relay (ESP8266)**, which sustains only **~150
  packets/s shared across all telemetry** (`telemetry_task.c:78,149-150`). This is
  the real ceiling — the byte budget is rarely the limit.
- TX path: writers → `channel.c` 2×2048 B ping-pong (`ENTER_CRITICAL`) →
  `flush_task` (spins at ~1 kHz, `channel.c:342` `v_delay(1)`) → DMA2 Stream7.
  Overflow → frame dropped + `_tx_overflow_count++`.
- Frame overhead = **12 B/frame** (10 B header + 2 B CRC, `navlink/sim/frame.py`).

### Steady-state load (telemetry_task, 6 ms tick = 166.7 Hz; rate = 166.7/N)

| Message | flag (`%N`) | rate | payload | frame (+12) | B/s |
|---|---|---:|---:|---:|---:|
| IMU_COMPRESSED | `%6` | 27.8 | 22 | 34 | 945 |
| CONTROL_TRACE (pid_err) | `%8` | 20.8 | 72 | 84 | 1,750 |
| MOTOR_TELEMETRY | `%8` | 20.8 | 32 | 44 | 917 |
| RC_CHANNELS | `%15` | 11.1 | 38 | 50 | 556 |
| ATTITUDE_EULER | `%15` | 11.1 | 24 | 36 | 400 |
| VERTICAL_STATE | `%15` | 11.1 | 21 | 33 | 367 |
| BARO | `%15` | 11.1 | 16 | 28 | 311 |
| SYSTEM_HEALTH+HEARTBEAT+FLIGHT_MODE | `%50` | 3.3×3 | 13/13/2 | 25/25/14 | 213 |
| IMU_RAW (full) | `%100` | 1.7 | 44 | 56 | 93 |
| EST_PERF | per-loop* | ~1 | 16 | 28 | ~28 |
| **periodic subtotal** | | **~152 pkt/s** | | | **~5.6 KB/s** |
| PERF burst (PERF_GLOBAL + ≤24 TASK + ≤16 FIFO) | 1 Hz | 41 frames | — | — | ~1.3 KB |
| **TOTAL** | | **~167 pkt/s peak** | | | **~6.9 KB/s** |

\*EST_PERF gated on queue availability; assumed low.

### Findings
1. **Bytes are not the limit:** ~6.9 KB/s is **~30%** of the 23 KB/s UART. Even a
   direct UART has ~16 KB/s spare.
2. **Packets ARE the limit:** ~167 pkt/s peak (periodic ~152 + the 41-frame perf
   burst) is **already at/over the ESP's ~150 pkt/s budget.** This is exactly why
   the sysid dump *suppresses* periodic telemetry to "hand the bridge's ~150
   pkt/s budget to the dump" (`telemetry_task.c:78`). The perf burst is the spike
   that pushes it over.
3. **Frame overhead punishes small frames:** 12 B of framing on a 16–32 B payload
   is 27–43% overhead. Over the *packet*-limited ESP link, the lever is
   **coalescing** (fewer, larger frames), not raising rates.

### Root cause of the ~150 pkt/s ESP ceiling (it is NOT UDP bandwidth)

The relay is `tools/arduino/telemetry_bridge/telemetry_bridge.ino` (ESP8266):
FC USART6 → ESP → WiFi/UDP → GCS. The ~150 pkt/s is an **empirical limit on
sustainable UDP *datagram* sends**, caused by:
1. **Per-datagram WiFi airtime + unicast MAC retries** — once the GCS is learned,
   sends are unicast with `WIFI_NONE_SLEEP`, so each datagram is a full TX→ACK
   exchange + backoff + link-layer retransmits (`*.ino:34-35,66-71`). Small
   packets are airtime-inefficient; fixed 802.11 per-frame overhead dominates.
2. **Single 80 MHz core, cooperative `loop()`** — WiFiManager, UDP RX, UART drain,
   framing, and UDP send all share one core; lwIP send cost caps sends/s.
3. **UDP has no backpressure** — overflow is *silently dropped*, so the FC must
   self-limit (hence the budget lives in firmware, not the link).

It is **not** UDP-the-protocol's byte overhead (datagrams ≤512 B; 28 B IP/UDP
header negligible) and **not** the UART (30% used). The bridge already mitigates
by **batching whole frames into one ≤512 B datagram**, flushed every **2 ms**
(timer1) or when full (`*.ino:152-173`) — capping datagrams at 500/s and
coalescing bursts (the 1336 B perf burst → ~3 datagrams, not 41). But at steady
trickle the UART delivers ~46 B per 2 ms tick (~1 frame), so **datagram-rate ≈
frame-rate ≈ 150–167/s**, hitting the ESP ceiling directly. Levers: coalesce
multiple messages per NavLink frame; lengthen the bridge flush interval (latency
for efficiency); or bypass the relay (direct UART/USB → 23 KB/s byte ceiling, or
an ESP32).

### Max *valid* telemetry rate
- **Over the ESP relay (shipped config): ~150 pkt/s — already saturated.** You
  cannot add periodic messages without dropping; you can only trade (coalesce
  small frames, drop the perf burst cadence, or rate-limit). Byte headroom is
  irrelevant here.
- **Over a direct UART (no relay): ~23 KB/s.** Practical per-message ceilings:
  IMU_COMPRESSED (34 B) ≈ **677 Hz**, CONTROL_TRACE (84 B) ≈ **274 Hz**, average
  50 B frame ≈ **460 frames/s**. So even direct UART **cannot** stream 1–2 kHz
  hot-loop IMU/control telemetry (84 B × 1 kHz = 84 KB/s ≫ 23 KB/s). **Hot-loop
  capture must go to SD, not the GCS link.**
- Raising the UART baud (e.g. 921600) lifts the byte ceiling 4×, but the ESP
  relay's packet budget caps it regardless unless the relay is bypassed.

### Raising the ceiling: ESP-side coalescing design

The right fix is to pack **more frames per UDP datagram** so fewer datagrams/s are
sent — attacking the binding constraint directly, entirely in the bridge (no FC or
protocol change). The constraint on how far this goes is **MTU and flush latency,
NOT the ESP's RAM** (RAM is already ample; frames arrive only as fast as the UART
trickles them, so coalescing trades *latency* for throughput).

**Two knobs in `telemetry_bridge.ino`:**

1. **`MAX_UDP`: 512 → ~1472 B** (1500 MTU − 20 IP − 8 UDP). ~2.9× more frames per
   datagram with **zero IP fragmentation**.
   - Do **not** go above MTU using spare RAM: lwIP would fragment into multiple IP
     fragments, each still its own over-the-air WiFi TX (**no airtime saved** — the
     actual constraint), and losing any one fragment drops the **whole** coalesced
     datagram (UDP has no retransmit). Keep one datagram = one ≤MTU WiFi frame so
     loss stays atomic.

2. **`FLUSH_TICKS`: 2 ms → a tunable latency budget.** Frames accumulate only at
   the UART rate (~46 B ≈ 1 frame per 2 ms @ 230400), so packing N frames means
   waiting N ticks. Pick by latency budget:

   | Flush interval | Bytes/datagram (@23 KB/s) | Datagram rate | vs ~150 ceiling | Added latency |
   |---|---:|---:|---:|---|
   | 2 ms (today) | ~46 (≈1 frame) | ~150/s | **at limit** | baseline |
   | 5 ms | ~115 (≈2–3 frames) | ~100/s | under | +3 ms |
   | 10 ms | ~230 (≈4–5 frames) | ~100/s | comfortable | +8 ms |

3. **Adaptive flush (recommended):** send immediately when `out[]` reaches the
   MTU cap, else on the timer. Bursts (e.g. the 1336 B perf burst) go out fast;
   idle periodic telemetry batches. The existing `outLen >= MAX_UDP` early-flush
   already does half of this — just raise the cap and the timer.

**Where the bottleneck moves:** at ~10 ms flush + 1472 B cap the datagram rate
drops well under the ESP's ~150/s ceiling while carrying the same-or-more frames,
so the limit shifts **off the ESP and back onto the UART** (23 KB/s ≈ ~460 frames/s
at 50 B/frame). To exceed that, raise the FC `UART_BAUDRATE` (e.g. 921600 →
~92 KB/s); then the ESP's WiFi *byte* throughput and the flush latency become the
new limits. Net: **~3× more packets/s for a few ms of latency, no fragmentation,
no FC change.**

**RAM's real role:** the large `acc[]` / RX ring is a **burst/jitter buffer** — it
lets the ESP ride out transient WiFi TX stalls or MAC-retry storms and batch
bursts without overflowing, improving *delivered reliability*, not sustained rate.
That is the legitimate use of the ESP's spare RAM here.

---

## B. VFS logging rate

### Transport
- **SDIO 21 MHz, 4-bit** (`sdio.c:275-279`, SDIOCLK 84 MHz / 4) = 84 Mbit/s =
  **~10.5 MB/s raw**.
- **DMA backend is OFF** in the flash build (`_SDIO_BACKEND_DMA` undefined) →
  **polled** transfers: the CPU/calling task spins during the block move.
- FatFS over SDIO; one global `vfs_mutex` serializes *all* file ops across the 3
  loggers + PID + calib (`vfs.c:5`).

### The actual bottleneck: sync-per-record
`logger_write_internal` does, **for every record**, under the mutex
(`logger.c:60-73`):
```
v_mutex_lock → vfs_lseek → vfs_write → vfs_sync → v_mutex_unlock
```
`vfs_sync` = FatFS `f_sync` = flush the dirty data sector **+ rewrite the FAT +
rewrite the directory entry** → multiple 512 B block writes plus the card's
**internal program latency**:
- Decent card: **~1–4 ms/sync** → **~250–1000 records/s** ceiling.
- Worst case (wear-levelling / internal GC on a sector): **tens to >100 ms**
  single-record stalls.
- It runs **inline in the caller's task context** (no dedicated logger task), so
  the producer **blocks** for that whole latency on every record.

The data volume is trivial by comparison: 1 kHz × 64 B = 64 KB/s = **0.6%** of
the 10.5 MB/s SDIO bandwidth. **Bandwidth is never the limit; sync latency is.**

### Findings
1. **Practical ceiling ~250–1000 rec/s best case**, with multi-ms (occasionally
   ≥100 ms) tail latency that stalls the calling task — **unusable from the
   control/estimator hot loops** (a 1 kHz task cannot afford a 4 ms blocking sync).
2. **It compounds contention C1/C3** from the resource map: logging at rate holds
   `vfs_mutex` during slow syncs, starving `comm_processor`'s RX drain and any
   PID/calib save — i.e. high-rate logging can drop inbound GCS commands.
3. **Currently underused:** the only real callers found are the text/`vayu_log`
   path and a test (`grep logger_write`); the 3×10 MB circular files
   (`v_nav/v_sys/v_gen.bin`) are provisioned but the high-rate blackbox path
   isn't wired/exercised yet.

### Max *valid* logging rate
- **As architected: a few hundred records/s, caller-blocking.** Fine for
  low-rate events/text; not for flight blackbox.
- **To reach a real blackbox (e.g. 1 kHz IMU+control ≈ 64–128 KB/s)** — well
  within SDIO bandwidth — requires removing the sync-per-record design:
  1. **Batch** records into 512 B-block-aligned buffers; `f_sync` **once per
     block** (or once/second), not per record. ~512 B/block at 64 KB/s ≈ 125
     syncs/s — achievable.
  2. **Enable `_SDIO_BACKEND_DMA`** so transfers don't burn CPU.
  3. **Dedicated logger task** fed by an SPSC ring (decouple producers from SD
     latency; the buffers already exist in `.bss`).
  4. Keep blackbox on its **own file/mutex** so it doesn't serialize against the
     comm RX path (mitigate C1/C3).
  With those, **1–2 MB/s sustained is feasible** (limited by FAT overhead and
  card program throughput, not the 10.5 MB/s link).

---

## Combined picture

- **Telemetry** is for *monitoring*: packet-rate-limited to ~150 pkt/s by the ESP
  relay, ~7 KB/s today, cannot carry hot-loop data.
- **SD logging** is the right home for hot-loop data and has the raw bandwidth
  (10.5 MB/s) — but the current sync-every-record + polled + inline + global-mutex
  design caps it at a few hundred caller-blocking records/s. The fix is
  architectural (batch + DMA + dedicated task + per-stream lock), not a faster bus.

## References
- `src/comm/telemetry_task.c` (schedule, ESP 150 pkt/s budget), `src/comm/channel.c`
  (ping-pong, `flush_task` 1 kHz, DMA2 S7), `navlink/sim/frame.py` (12 B framing),
  `navlink/generated/c/navlink_msgs.h` (WIRE_SIZE_*)
- `src/logger/logger.c` (sync-per-record), `extern/vaios/kernel/vfs.c` (global mutex),
  `extern/.../stm32/sdio/sdio.c` (21 MHz ×4-bit, polled — `_SDIO_BACKEND_DMA` off)
- `vaios_app_config.h:20` (UART_BAUDRATE 230400)
- Siblings: `resource-ownership-map.md` (C1/C3), `memory-footprint-analysis.md`
