# sim_bridge — FS-i6 PPM → PC bridge

Arduino sketch that reads PPM from an FS-iA6B receiver (FS-i6 in PPM output mode) and forwards channel values to a PC over USB-serial for use as flight-simulator input.

> **Why PPM and not iBus?** The Arduino Nano (ATmega328P) has a single hardware UART, permanently wired to its onboard USB-serial bridge. Pulling iBus (115200 baud) onto D0 contends with the bridge and breaks the bootloader; SoftwareSerial at 115200 drops bytes. PPM uses any digital pin via a pin-change interrupt and dodges the conflict entirely. The Vayu firmware itself stays on iBus; this is a sim-only path.

## Hardware

- **Arduino Nano** (ATmega328P, 16 MHz) — any 5V AVR with an external interrupt on D2 also works (Uno, Pro Mini).
- **FS-iA6B** receiver bound to an FS-i6 transmitter.
- **FS-i6 in PPM mode**: long-press **OK** → **System** → **Output Mode** → **PPM** → **Yes** → long-press **OK** to save. The receiver now outputs combined PPM on the **CH1** servo header.

### Wiring — FS-iA6B `CH1` → Nano

| FS-iA6B `CH1` pin | Wire          | Nano |
|-------------------|---------------|------|
| Signal (top)      | white/yellow  | **D2** |
| +V (middle)       | red           | **5V** |
| GND (bottom)      | black/brown   | **GND** |

Nano USB → PC.

## Upload

Arduino IDE → **Board: Arduino Nano** → **Processor: ATmega328P** → select the COM/`/dev/ttyUSB*` port → Upload. If upload fails with `not in sync: resp=0x??`, switch to **Processor: ATmega328P (Old Bootloader)** — the choice depends on which bootloader your Nano clone shipped with (115200 vs 57600 baud).

No external libraries required.

### Command-line flash (Linux)

```
arduino-builder -compile \
  -hardware /usr/share/arduino/hardware -hardware /usr/share/arduino-builder \
  -tools /usr/share/arduino/hardware/tools -tools /usr/share/arduino-builder \
  -fqbn arduino:avr:nano:cpu=atmega328 \
  -build-path /tmp/sim_bridge_build \
  tools/arduino/sim_bridge/sim_bridge.ino

avrdude -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D \
  -Uflash:w:/tmp/sim_bridge_build/sim_bridge.ino.hex:i
```

Use `cpu=atmega328old` in the FQBN and `-b57600` for the old bootloader.

## Output format

USB-serial, **115200 8N1**, ~50 Hz.

```
# sim_bridge ready
1500,1500,1000,1500,2000,1000
1499,1501,1003,1502,2000,1000
...
NO_SIGNAL
```

- One line per PPM frame, comma-separated channel pulse widths in microseconds (typ. 1000–2000).
- `NO_SIGNAL` is printed when no valid frame arrives for **>100 ms** (receiver unpowered, transmitter off, miswiring).
- `# sim_bridge ready` is a one-shot startup banner — a PC-side script can wait for it before reading data.

## Default FS-i6 channel map (Mode 2)

| Channel | Source |
|---------|--------|
| CH1 | Aileron — roll |
| CH2 | Elevator — pitch |
| CH3 | Throttle |
| CH4 | Rudder — yaw |
| CH5 | SwA |
| CH6 | VrA / SwB / SwD |

## Reading from Python

```python
import serial

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
for line in s:
    line = line.decode(errors='ignore').strip()
    if not line or line.startswith('#') or line == 'NO_SIGNAL':
        continue
    ch = [int(x) for x in line.split(',')]
    # ch[0]=roll, ch[1]=pitch, ch[2]=throttle, ch[3]=yaw, ...
    print(ch)
```

## Tuning

All thresholds are constants at the top of `sim_bridge.ino`:

| Constant | Default | When to change |
|----------|---------|----------------|
| `PPM_PIN` | `2` | Move signal to a different external-interrupt pin (D2 or D3 on Nano). |
| `SYNC_GAP_US` | `3000` | If frames split incorrectly: raise toward 4000 µs. |
| `MIN_VALID_US` / `MAX_VALID_US` | `800` / `2200` | Widen if your transmitter's endpoints exceed the typical 1000–2000 µs range. |
| `SIGNAL_TIMEOUT_US` | `100000` | Sensitivity of the `NO_SIGNAL` watchdog. |
| `SEND_INTERVAL_MS` | `20` | Drop to 10 ms for 100 Hz output (still well within 115200 baud). |
