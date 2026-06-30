# telemetry_bridge — ESP8266 WiFi/UDP telemetry bridge

Arduino sketch that bridges the flight controller's telemetry UART to the
Navigator GCS over **WiFi/UDP**, in both directions — so the GCS no longer needs
a USB cable to the FC.

```
FC USART6  ──UART──▶  ESP8266  ──WiFi/UDP :14555──▶  Navigator GCS   (telemetry)
FC USART6  ◀──UART──  ESP8266  ◀──WiFi/UDP :14555──  Navigator GCS   (commands)
```

On the GCS, pick **UDP** in the toolbar transport selector (port **14555**) and
it parses the stream identically to serial.

> **Why this is more than a dumb byte-pump.** The FC→ESP UART link is clean, so
> the sketch parses the `0x56` framing *there* and puts only **whole frames** in
> each UDP packet. A dropped WiFi packet then loses whole frames cleanly instead
> of splitting one mid-stream and desyncing the GCS parser into a burst of
> garbage. This is the single biggest factor in a clean link.

## Hardware

- **ESP8266 NodeMCU 1.0** (ESP-12E). Other ESP8266 boards work if the strapping
  pins below are respected.
- Flight controller streaming vayu telemetry on **USART6 @ 460800 8N1** (matches the
  FC's `UART_BAUDRATE`; was 230400 before the bandwidth boost).

### Wiring — FC ⇄ ESP (3.3 V, direct, no level shifter)

| FC (USART6) | ESP pin | GPIO | Role |
|-------------|---------|------|------|
| PC6 (TX)    | **D7**  | GPIO13 | telemetry in — UART0 RX (after `Serial.swap()`) |
| PC7 (RX)    | **D4**  | GPIO2  | commands out — UART1 TX |
| GND         | GND     | —      | common ground |
| *(nothing)* | ~~D8~~  | GPIO15 | **leave UNCONNECTED** — boot strap, must be low at boot |

> **Flash with the FC still wired.** Telemetry RX sits on UART0's *alternate*
> pins (GPIO13/15 via `Serial.swap()`) and commands go out UART1 (GPIO2), so the
> default UART0 pins (GPIO1/3) stay free for USB flashing. Nothing the FC drives
> is a strapping pin — **do not** put a wire on D8/GPIO15 or the ESP won't boot
> *or* enter flash mode.
>
> The baud **must match** the FC's `UART_BAUDRATE` (`include/vaios_app_config.h`).

## First-time WiFi setup

On first boot (or if it can't join a known network) the ESP raises a captive
portal AP named **`vayu-config`**. Join it from a phone/laptop, browse to
**http://192.168.4.1**, and enter your WiFi credentials — they're saved to
flash. The GCS and the ESP must be on the same network (e.g. a laptop hotspot).

## Build & flash

Requires the **esp8266** core and the **WiFiManager** (tzapu) library.

```
arduino-cli core install esp8266:esp8266
arduino-cli lib install WiFiManager

arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 tools/arduino/telemetry_bridge
arduino-cli upload  -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 tools/arduino/telemetry_bridge
```

## How it works

- **Frame-aware relay + coalescing** — UART bytes accumulate in `acc[]`; whole
  frames (`sync 0x56 · typever · length · … · crc32`, total = `8 + length + 4`)
  are sliced out and packed into `out[]`, sent as one datagram when it fills the
  **MTU cap (~1472 B)** or on the **`FLUSH_MS` (8 ms) timer** (a hardware timer1
  ISR sets the flush flag). Never splits a frame across packets. Packing many
  frames per datagram is the throughput lever: the ESP's limit is sustainable UDP
  **datagrams/s** (per-packet WiFi airtime + MAC retries), not bytes/s — so fewer,
  fuller datagrams carry more frames under the same ceiling. `MAX_UDP` is capped
  at the MTU so each datagram stays a single WiFi frame (above MTU, IP
  fragmentation saves no airtime and makes any lost fragment drop the whole
  datagram). `FLUSH_MS` trades latency for coalescing.
- **Reliable delivery** — `WIFI_NONE_SLEEP` kills modem-sleep latency/loss, and
  the bridge unicasts to the GCS once it has heard from it (the GCS announces
  itself periodically), so WiFi MAC retries cover lost frames instead of an
  unacked broadcast.
- **Bidirectional** — inbound UDP frames starting with `0x56` are forwarded out
  UART1 to the FC (ARM, PID, calibration, time-sync).
- **Idle heartbeat** — when no FC frames arrive for 2 s it emits `BRIDGE-HB` so
  the link is testable with no FC attached.

## Config (constants at the top of `telemetry_bridge.ino`)

| Constant | Default | When to change |
|----------|---------|----------------|
| `FC_BAUD` | `460800` | Must equal the FC's `UART_BAUDRATE`. Raised from 230400 with the 4 KiB RX ring + `acc[]` (below) to absorb WiFi-TX stalls at the higher byte rate. See `firmware/docs/plans/link-bandwidth-boost.md`. |
| `UDP_PORT` | `14555` | Must match the GCS UDP port. |
| `MAX_UDP` | `1472` | Datagram size cap = MTU − IP/UDP headers. **Do not raise past ~1472** — above MTU lwIP fragments (no airtime saved; one lost fragment drops the whole datagram). |
| `FLUSH_MS` | `8` | Flush interval (ms). Latency-for-coalescing knob: longer = more frames/datagram = fewer datagrams/s (further under the ESP ceiling). Drop to `2` for low-latency stick-feel data. `FLUSH_TICKS` derives from this (5000 ticks/ms @ TIM_DIV16). |
| `CFG_PORTAL` | `"vayu-config"` | Captive-portal AP name. |
