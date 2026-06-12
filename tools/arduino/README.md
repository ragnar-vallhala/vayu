# tools/arduino — companion MCU sketches

Arduino-framework sketches that run on **companion microcontrollers** (not the
flight controller) as bench and development tools for the Vayu stack — RC
bridging for the simulator, wireless telemetry, and the like.

Each sketch lives in **its own folder whose name matches the `.ino`**, which is
what `arduino-cli` (and the Arduino IDE) require:

```
tools/arduino/
├── sim_bridge/        FS-i6 PPM → PC RC bridge (sim input)
│   ├── sim_bridge.ino
│   └── README.md
└── telemetry_bridge/  ESP8266 WiFi/UDP telemetry+command bridge (FC ⇄ GCS)
    ├── telemetry_bridge.ino
    └── README.md
```

## Sketches

| Sketch | Board | Purpose |
|--------|-------|---------|
| [`sim_bridge`](sim_bridge/) | Arduino Nano (ATmega328P) | Reads PPM from an FS-iA6B receiver and streams channel values to the PC over USB-serial, as flight-simulator RC input. |
| [`telemetry_bridge`](telemetry_bridge/) | ESP8266 NodeMCU | Bridges the FC's telemetry UART to the Navigator GCS over WiFi/UDP, both directions — no USB cable to the FC. |

See each folder's `README.md` for wiring, build/flash, and configuration.

## Building

Sketches use the `arduino-cli` layout, so the folder path *is* the sketch:

```
arduino-cli compile --fqbn <fqbn> tools/arduino/<sketch>
arduino-cli upload  -p <port> --fqbn <fqbn> tools/arduino/<sketch>
```

Cores used here:

| Board | Core | FQBN |
|-------|------|------|
| Arduino Nano | `arduino:avr` | `arduino:avr:nano:cpu=atmega328` |
| ESP8266 NodeMCU | `esp8266:esp8266` | `esp8266:esp8266:nodemcuv2` |

Install a core with `arduino-cli core install <core>` (the ESP8266 core needs
the board-manager URL `http://arduino.esp8266.com/stable/package_esp8266com_index.json`
in `arduino-cli config`). These run on external boards and are independent of
the firmware CMake build.
