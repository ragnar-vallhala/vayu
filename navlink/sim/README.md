# NavLink v2 link simulator

Two host processes — a flight-controller model and a ground-station model — talk
NavLink v2 over UDP, with a configurable impairment layer between them. Use it to
see how the protocol and a GCS behave on a real-world link: lossy radios, jittery
Wi-Fi, high-latency satellite, bit corruption, bandwidth caps.

```sh
python3 sim.py                                 # clean link, 10 s
python3 sim.py --scenario telemetry_radio
python3 sim.py --scenario lossy --duration 20
python3 sim.py --latency-ms 50 --jitter-ms 20 --loss 0.05 --corrupt 0.01
python3 sim.py --scenario satellite --json     # machine-readable
```

It reuses the generated Python codec, so it always matches `dialect.json`
(regenerated automatically if missing).

## Interactive console (`live.py`)

For a hands-on session — tune the link and inject commands while stats update —
use the interactive console:

```sh
python3 sim/live.py                 # start clean, then type `help`
python3 sim/live.py --scenario lossy
```

It runs the FC and GCS as threads sharing the live link config, so changes take
effect immediately. A **live status bar** pinned to the bottom row updates on its
own (~2.5 Hz) showing RX rate, throughput, loss %, reorders, CRC errors, RTT,
command-ACK count, and the current up/down link config. At the `nlctl>` prompt:

| command | effect |
|---------|--------|
| `status` | print a full stats snapshot inline |
| `set [up\|down] K V` | tune a knob live: `latency jitter loss dup reorder corrupt rate` |
| `scenario NAME` | apply a preset |
| `reset` | clear all impairments |
| `arm` / `disarm` / `ping` | inject a command / probe now |
| `pid KP KI KD` | send a `CMD_SET_PID` |
| `quit` | stop and print a final summary |

Example: type `scenario lossy` and watch the bottom bar's loss %, reorders, CRC
errors and RTT climb; `reset` brings them back down. The bar is shown only on a
real terminal (disabled when output is piped).

## How it works

`sim.py` spawns two `endpoint.py` processes wired to each other over localhost
UDP. Each owns a `Link` (in `link.py`) that impairs its **egress**, so the
downlink (FC→GCS) and uplink (GCS→FC) are impaired independently.

- **FC** emits the real telemetry set at firmware-like rates (HEARTBEAT 1 Hz,
  ATTITUDE 50 Hz, IMU full/compressed, RC, motors, control trace, health, …),
  echoes PINGs, and answers commands with `COMMAND_ACK`.
- **GCS** receives and validates every frame, periodically sends a `PING` and a
  `CMD_SET_PID`, and reports link statistics.

## Impairments (CLI flags / scenario knobs)

| knob | meaning |
|------|---------|
| `--latency-ms` | one-way base latency |
| `--jitter-ms` | uniform `[0, jitter]` added per datagram (also causes reordering) |
| `--loss` | P(drop), 0..1 |
| `--dup` | P(send a duplicate copy) |
| `--reorder` | P(add a large extra delay → reorder) |
| `--corrupt` | P(flip one random bit; the CRC catches it at RX) |
| `--rate-bytes` | egress bandwidth cap (B/s); 0 = unlimited |
| `--seed` | RNG seed (broadly reproducible; exact timing still varies) |

Presets (`--scenario`): `clean`, `wifi`, `telemetry_radio`, `lossy`, `satellite`.
Explicit flags override the preset.

## Stats reported by the GCS

- frames + bytes received, RX rate (Hz), throughput (B/s)
- CRC errors, undecodable (unknown msgid), non-frame bytes
- **loss estimate** — reorder-tolerant: the 8-bit seq is unwrapped to a monotonic
  counter, so loss is `span(seqs seen) − received` rather than a naive per-gap
  count (which over-reports under reordering). The combined report also prints
  the link's *actual* egress drops as ground truth to compare against.
- reorders (out-of-order arrivals)
- PING round-trip time (min/avg/max, lost)
- command ACK rate and ACK latency

## Files

| file | role |
|------|------|
| `sim.py` | orchestrator: presets, spawns the two processes, prints the combined report |
| `endpoint.py` | one endpoint (`--role fc` / `--role gcs`): models + stats |
| `link.py` | egress impairment model (delay/loss/dup/reorder/corrupt/rate) |
| `frame.py` | NavLink v2 frame encode/decode (sync+header+CRC), reuses the generated codec |
| `common.py` | locates / generates and imports the codec |
