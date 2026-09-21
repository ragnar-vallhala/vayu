# Vayu

A flight controller for a small multirotor, written from the ground up: the
firmware that flies the aircraft, the simulator that is the same firmware
compiled for a host, and the tooling around both.

This repository is also the **entry point to the stack** — the other pieces
live in their own repositories and are linked below.

## What is here

| | |
| --- | --- |
| `firmware/` | the flight controller itself — estimation, control, mixing, failsafes, drivers. Runs on an STM32F401RE. |
| `sim/host/` | SITL: the firmware compiled for a host, running the real scheduler and the real control code against simulated physics |
| `sim/vsim/` | the rigid-body physics and sensor models the SITL flies against |
| `tools/` | telemetry decoding, calibration harnesses, log analysis, the build/test wrappers |
| `navlink/` | submodule — the wire protocol |
| `extern/vaios/` | submodule — the RTOS the firmware runs on |

## The rest of the stack

- [**vayu-navigator**](https://github.com/ragnar-vallhala/vayu-navigator) — the
  Qt6 ground station, the headless Python SDK, and the autotuner. It builds
  against a *release* of this repo, not its source.
- [**navlink**](https://github.com/ragnar-vallhala/navlink) — the wire protocol:
  a dialect plus a code generator, from which every consumer generates its own
  codec.
- [**vaios**](https://github.com/ragnar-vallhala/vaios) — the real-time kernel
  and its NavHAL drivers.
- [**vtest**](https://github.com/ragnar-vallhala/vtest) — the test runner these
  repos share.

## Running the simulator without building firmware

Every release ships a **SITL SDK**: the engine as a loadable module, the same
engine as a headless binary, and the headers a host compiles against.

```sh
curl -sSLf -LO https://github.com/ragnar-vallhala/vayu/releases/latest/download/vayu-sitl-sdk-v0.1.0-linux-x86_64.tar.gz
tar xzf vayu-sitl-sdk-v0.1.0-linux-x86_64.tar.gz
```

That is the whole dependency a ground station has on this repo.

## Building

See [build.md](build.md) for per-component commands, options and the CI map.
The short version:

```sh
git submodule update --init --recursive
tools/scripts/vayu.sh build firmware     # ARM firmware
tools/scripts/vayu.sh build sitl         # host SITL + its tests
tools/scripts/vayu.sh test               # the vtest TUI
```

## Documentation

- [System architecture](ARCHITECTURE.md) — how the pieces fit together
- [Documentation index](firmware/docs/README.md) — reference, plans, journals
- [Wire protocol](https://github.com/ragnar-vallhala/navlink/blob/main/docs/reference/messages/data_frame.md)

## Status

The aircraft arms, hovers and holds attitude. The estimator, the FFT dynamic
gyro notch, the high-speed IMU-to-SD recorder and the NavLink v2 command path
are all flight-proven.

It is `v0.1.0` and not `v1.0.0` because hover throttle sits near 0.65 where the
airframe was characterised at 0.38 and the cause is not yet found; because
every flight since the 2026-09-19 reflash has ended in a ground impact whose
triggering failsafe the logs do not record; and because the ESCs have never
been calibrated.

## License

Apache License 2.0 — see [LICENSE.md](LICENSE.md). Copyright (C) 2026
NAVRobotec Pvt Ltd.
