# RC Data

> Updated for NavLink v2. Authoritative wire spec: navlink/dialect.json + docs/analysis/navlink-v2-spec.md.

RC Data reports the raw Remote Control channel values received via iBus.

In NavLink v2 this is **`RC_CHANNELS` (msgid 1028)**. The payload is:

| Field     | Type      | Description                                   |
| --------- | --------- | --------------------------------------------- |
| `chan`    | u16 × 18  | Per-channel value in microseconds (µs)        |
| `rssi`    | u8        | Receiver signal strength                      |
| `count`   | u8        | Number of valid channels                      |

> Note: v1 carried 14 × u16 channels. v2 carries 18 plus `rssi` and `count`.

## Failsafe Detection

> Heuristic / unverified: a value of 0 on the throttle/first channel has historically been treated as a failsafe or receiver-disconnect indication. Prefer `count`/`rssi` and the HEARTBEAT `nav_state` (FAILSAFE) as the authoritative signal; treat the "channel 1 == 0" rule as a fallback hint only.

Byte layout and CRC: `../../navlink/dialect.json`, `../analysis/navlink-v2-spec.md`.

## Changelog

| Date       | Author      | Description                                  |
| ---------- | ----------- | -------------------------------------------- |
| 11/03/2026 | Antigravity | Initial version                              |
| 06/2026    | —           | NavLink v2: msgid 1028, chan[18]+rssi+count  |
