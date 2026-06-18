# Vayu — system architecture

Vayu is a full drone stack: real-time **flight-controller firmware**, a desktop
**ground station**, a **physics simulator** that runs the real firmware in the loop
(SITL), and a **headless scripting SDK** to fly it from Python. This page is the
map of how those pieces fit together and where to read deeper.

Four components, two shared contracts:

| Component | Lives in | What it is | Deep dive |
|-----------|----------|------------|-----------|
| **FC** (firmware) | `src/`, `include/` | STM32F401 @ 84 MHz, vaios RTOS: sensors → EKF → cascade control → motors | [docs/reference/software-flow.md](docs/reference/software-flow.md) |
| **NavLink** (wire) | `navlink/` | The message dialect; one spec generates C / C++ / Python codecs | [navlink/docs/reference/navlink-v2-spec.md](navlink/docs/reference/navlink-v2-spec.md) |
| **GCS** (Navigator) | `software/src/` | Qt6 ground station: telemetry, plots, calibration, replay, autotune | [software/docs/reference/gcs-architecture.md](software/docs/reference/gcs-architecture.md) |
| **Sim + Pilot** | `tools/vsim/`, `tools/sim_host/`, `software/headless-sdk/` | `vsim_d` physics daemon, the SITL host, and the `vayu_headless` Pilot scripting API | [tools/vsim/docs/reference/sim-architecture.md](tools/vsim/docs/reference/sim-architecture.md) |

The two contracts are the seams that keep components decoupled: **NavLink** (the
FC↔GCS↔Pilot wire) and **vsim_proto** (the FC↔sim transport).

---

## System context

```mermaid
flowchart TB
    classDef comp fill:#e8f0fe,stroke:#3367d6,color:#111;
    classDef contract fill:#fff4ce,stroke:#9a6700,color:#111;
    classDef ext fill:#e6f4ea,stroke:#137333,color:#111;

    OP[("operator<br/>human at the GCS, or a Python script")]:::ext

    GCS["GCS — Navigator (software/src/)<br/>telemetry · plots · calibration · replay · autotune"]:::comp
    FC["FC — firmware (src/, include/)<br/>STM32F401 · vaios · sensors→EKF→control→motors"]:::comp
    VSIM["vsim_d — physics daemon (tools/vsim/)<br/>rigid-body dynamics · wind · sensor models"]:::comp
    SITL["vayu_sitl — the real firmware on host (tools/sim_host/)<br/>same code as FC, sensors-in / PWM-out"]:::comp
    PILOT["Pilot — vayu_headless (software/headless-sdk/)<br/>arm · goto · truth() · fidelity scoring"]:::comp

    NAV[["NavLink dialect (navlink/dialect.json)<br/>→ C · C++ · Python codecs"]]:::contract
    VP[["vsim_proto (tools/vsim/include)<br/>pwm · imu · pose · ctl"]]:::contract

    OP --> GCS
    OP --> PILOT
    GCS <-->|"NavLink v2 · serial / UDP"| FC
    FC <-->|"vsim FIFOs (SITL only)"| VSIM
    SITL <-->|"vsim FIFOs"| VSIM
    GCS -.->|"hosts in-process + spawns vsim_d"| SITL
    PILOT -.->|"spawns + scripts"| SITL
    PILOT -.->|"ctl / pose"| VSIM

    NAV -.->|"defines the wire"| FC
    NAV -.->|"defines the wire"| GCS
    NAV -.->|"defines the wire"| PILOT
    SITL -.->|"is the FC code, host-compiled"| FC
    VP -.->|"defines the transport"| VSIM
```

The same firmware control logic runs in all paths: on the board as **FC**, and on the
host as **vayu_sitl**. NavLink is generated from a single dialect, so the FC, the GCS,
and the Python SDK can never drift on the wire format.

---

## Runtime topologies

The pieces compose into three ways to fly, plus the tuner:

| Topology | Who runs | Link | Use |
|----------|----------|------|-----|
| **Hardware flight** | FC board + GCS | NavLink v2 over serial/UDP | real flying; GCS shows telemetry, sends commands |
| **In-app SITL** | GCS hosts the firmware **in-process** + spawns `vsim_d` | vsim FIFOs (PWM/IMU/pose) + in-process UART2 telemetry | fly the real firmware from the UI / a joystick, no hardware |
| **Headless SITL** | `vayu_headless` spawns `vayu_sitl` + `vsim_d` | vsim FIFOs + RC/UART PTYs | scripted missions, CI, fidelity tests — `Pilot.arm_takeoff` / `goto` |
| **Autotune** | an isolated `vayu_sitl` + `vsim_d` pair | vsim `ctl` test-rig + NavLink telemetry | search PID gains against step responses |

The **SITL seam** is a hard contract: the firmware only ever consumes sensors (IMU FIFO)
and RC, and emits PWM + telemetry — it never reads ground truth. The operator/Pilot *may*
read ground truth (the `pose` FIFO). This is what makes a SITL result trustworthy; it is
enforced at runtime (see the sim doc).

---

## Data-flow seams

- **FC ↔ GCS (NavLink v2):** telemetry down (attitude, IMU, RC, motors, health, control
  trace…) and commands up (arm, calibrate, set-PID, set-mode, time-sync). Sync `0x56`,
  version `0x02`, 24-bit msgids, CRC-16. Decoded identically for live, sim, and replay.
- **FC ↔ Sim (vsim_proto):** four `/tmp` FIFOs — `pwm` (FC→sim duty), `imu` (sim→FC
  sample), `pose` (sim→operator ground truth), `ctl` (operator→sim: reset/world/wind/
  rates/rig/faults). Per-session `VSIM_FIFO_SUFFIX` isolation.
- **Pilot ↔ SITL (scripting):** Python reads telemetry (`lab.telem`) and ground truth
  (`lab.truth()`), and actuates **only via RC sticks** — the same seam a human pilot uses.

---

## Repository map

| Path | Role |
|------|------|
| `src/`, `include/` | FC firmware (control, estimation, sensors, comm, RTOS glue) |
| `navlink/` | NavLink dialect (`dialect.json`), code generator, ABI, integration notes |
| `software/src/` | GCS Navigator (Qt6) |
| `software/headless-sdk/` | `vayu_headless` — the Pilot scripting SDK + fidelity scoring |
| `tools/vsim/` | `vsim_d` physics daemon + `vsim_proto.h` |
| `tools/sim_host/` | the SITL host seam (`vayu_sitl`, IMU/RC feeders, PWM/UART shims) |
| `tools/autotune/` | PID autotuner (drives `vsim_d` directly) |
| `docs/` | documentation, organized per component into reference / plans / journal / scratch |

Documentation is organized **per component**, each with the same four lifecycle layers
(reference / plans / journal / scratch). Start at the
[documentation index](docs/README.md) for the taxonomy and the component map.

## Where to go next

- New to the firmware? → [docs/reference/software-flow.md](docs/reference/software-flow.md)
- Wire format / adding a message? → [navlink/docs/reference/navlink-v2-spec.md](navlink/docs/reference/navlink-v2-spec.md)
- Hacking the ground station? → [software/docs/reference/gcs-architecture.md](software/docs/reference/gcs-architecture.md)
- Scripting a SITL flight? → [tools/vsim/docs/reference/sim-architecture.md](tools/vsim/docs/reference/sim-architecture.md) (§5, the Pilot API)
