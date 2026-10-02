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

## Branches

- **`main`** — integration. Every change arrives as a pull request and must
  pass the suite. This is what you build from day to day.
- **`stable`** — releases, and what a downstream consumer or a packaged SITL
  SDK pins. Release tags live here. A release is `main` **merged** into
  `stable` and tagged there.

Both are protected: a pull request is required, the branch must be up to date
before merging, the checks below must pass, and that applies to admins too.
Force pushes and branch deletion are refused on both.

Two required contexts rather than a list of the twelve jobs: **`CI required`**
and **`Gates required`** are fan-in jobs that pass only when everything they
depend on did. Naming the twelve individually means a renamed or newly added
job silently stops being enforced, which is worse than not gating at all — it
still shows up green on the pull request. `tools/dev/check_ci_required.sh`
asserts every job is reachable from its workflow's fan-in, and runs as the
fan-in's own first step.

**Merge a release into `stable`, never squash it.** A squash gives `stable` a
commit that is not on `main`, so `main` stops being an ancestor and the next
release has to reconcile two histories that hold the same content. This is not
hypothetical — it happened twice on `navlink` before the rule was written down.

**Do not stack pull requests.** GitHub only retargets a stacked pull request
when its base is deleted *before* the merge, so merging a stack in quick
succession lands each one on the branch below it instead of on `main`. That
happened here with #9, #10 and #11: all three reported merged while `main` had
only the first, and it took a fourth pull request to actually land the work.

### `stable` means released, not flown

`stable` tracks what has been cut as a release. It does **not** assert that the
tree has been in the air. Airworthiness is carried by the tags instead:

- `flight-YYYY-MM-DD` — this tree flew
- `bench-YYYY-MM-DD` — bench-verified only, never flown

As of this writing `flight-2026-09-18` is the newest flown tree, and everything
after it is bench-only. Anchor a "known good in the air" claim on a `flight-*`
tag, not on `stable`.

## Status

The aircraft arms, hovers and holds attitude. The estimator, the FFT dynamic
gyro notch, the high-speed IMU-to-SD recorder and the NavLink v2 command path
are all flight-proven.

It is `v0.1.0` and not `v1.0.0` because every flight since the 2026-09-19
reflash has ended in a ground impact whose triggering failsafe the logs do not
record, and because nothing has flown since 2026-09-18 — the ESC endpoint
calibration, the battery instrument and the rangefinder diagnostics are all
bench-verified only.

The hover-throttle discrepancy that stood here — 0.65 to 0.75 measured against
an airframe characterised at 0.38 — was the propellers. Voltage, weight, KV,
prop size and rotation, the ESC endpoints and the firmware output path were all
eliminated by measurement first. Hover has not yet been re-measured on the
repaired airframe, so treat every hover and tuning figure recorded before the
props were dealt with as taken from an aircraft making roughly half its thrust.

The ESCs now have a calibration procedure and it has been run: a two-boot
sequence driving every endpoint, confirmed from the card with all four motors
at the idle floor afterwards.

## License

Apache License 2.0 — see [LICENSE.md](LICENSE.md). Copyright (C) 2026
NAVRobotec Pvt Ltd.
