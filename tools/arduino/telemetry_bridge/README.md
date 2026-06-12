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
- Flight controller streaming vayu telemetry on **USART6 @ 230400 8N1**.

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

- **Frame-aware relay** — UART bytes accumulate in `acc[]`; whole frames
  (`sync 0x56 · typever · length · … · crc32`, total = `8 + length + 4`) are
  sliced out and packed into `out[]`, which is sent as one datagram per **512 B**
  or per **2 ms** (a hardware timer1 ISR sets the flush flag). Never splits a
  frame across packets.
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
| `FC_BAUD` | `230400` | Must equal the FC's `UART_BAUDRATE`. |
| `UDP_PORT` | `14555` | Must match the GCS UDP port. |
| `MAX_UDP` | `512` | Datagram size cap (keep ≤ MTU to avoid IP fragmentation). |
| `FLUSH_TICKS` | `10000` | Flush timeout in timer1 ticks (5 MHz → 0.2 µs/tick; 10000 = 2 ms). |
| `CFG_PORTAL` | `"vayu-config"` | Captive-portal AP name. |
