# Headless SDK — standardisation plan

> Status: **PLAN ONLY** (no implementation yet). This document is the agreed
> blueprint for turning the ad-hoc `tools/sim_host/sitl_lab.py` harness into a
> first-class, reusable SDK for driving the **real** flight-controller logic
> headlessly. Companion to `docs/sim-fidelity/00-phasing.md`.

---

## 1. Why

We can already fly the *actual* firmware (estimator → angle/rate cascade →
mixer → arming → telemetry) with no hardware and no human on the sticks, and
render it live in the GCS. But the whole capability lives in one ~1100-line
script (`tools/sim_host/sitl_lab.py`) that grew organically while debugging:
process orchestration, three wire protocols, a guidance autopilot, a socket
server, a CLI, and the GCS bridges are all interleaved in one file.

That is fine for a spike, wrong for something we now want to *depend on* —
for CI regression flights, controls/algorithm experiments, tuning sweeps, and
GCS-attached demos. The goal of this effort is to **standardise** it: a clean,
documented, importable SDK with a stable contract, so a test, a notebook, a
teammate, or CI can drive a flight in a few lines without reverse-engineering a
script.

## 2. What exists today (the raw material)

| Piece | Location | Role |
|---|---|---|
| `SitlLab` | `tools/sim_host/sitl_lab.py` | process lifecycle (vsim_d + vayu_sitl), FIFO/pty wiring, RC feed, NavLink telemetry decode, UART2→GCS bridge, pose read + GCS fan-out, world-mesh push, ground truth |
| `Pilot` | same file | continuous 50 Hz outer-loop guidance (takeoff/goto/land/station-keep) |
| `serve()` / `client()` | same file | persistent session over a Unix socket + thin CLI |
| `demo()` / `flight()` | same file | one-shot run modes |
| vsim wire helpers | same file | `_ctl/_reset/_world/_geometry_frame/_wind/_world_mesh` framing |
| world-mesh builder | `tools/sim_host/worldmesh/` (C++) | reuses GCS `vsim::loadMesh` + `buildWorldBvh` to build the collision BVH |
| host shims | `tools/sim_host/src/*.c` | the real-firmware host (RC/IMU feeders, UART2 pty, PWM fifo) |
| physics daemon | `tools/vsim/` | `vsim_d` rigid-body + collision |
| NavLink codec | `navlink/generated/python/`, `navlink/sim/` | telemetry decode |

**Pain points to fix:** one-file monolith; no installable package or import
path; no API/versioned contract; socket command set is ad-hoc strings; config
discovery (GCS `.conf`) is hard-coded; no automated tests pinning behaviour;
binary paths via loose env vars; guidance is hard-wired (not pluggable);
duplicated wire constants that must track `vsim_proto.h`/the NavLink dialect by
hand.

## 3. Goals / non-goals

**Goals**
- A Python package `vayu_headless` under `software/headless-sdk/` that is
  `pip install -e`-able and importable.
- A **stable public API** (`SitlSession`, `Pilot`/autopilot, telemetry &
  ground-truth accessors) and a versioned **session protocol**.
- A single CLI (`vayu-headless`) covering serve/do/run, replacing the
  `sitl_lab.py --serve/--do/--attach` surface.
- Behaviour-preserving migration: every capability we have today keeps working
  (attach-once-fly-many, GCS pose + telemetry bridges, world collision).
- Shared, not duplicated, wire definitions (single source of truth with
  `vsim_proto.h` and the NavLink dialect).
- A pytest-based headless flight suite usable in CI.

**Non-goals (for this effort)**
- No new physics/firmware features (collision, wind, etc. are already done).
- Not replacing the GCS sim — the SDK *complements* it (and reuses its codec
  and mesh builder).
- Not a general robotics framework; scope is Vayu SITL.
- No rewrite of the C host shims or `vsim_d`.

## 4. Proposed layout

```
software/headless-sdk/
  PLAN.md                     # this doc
  README.md                   # quickstart + API tour (Phase 5)
  pyproject.toml              # installable: package `vayu_headless`, CLI `vayu-headless`
  vayu_headless/
    __init__.py               # curated public API surface + __version__
    config.py                 # GCS .conf / env / explicit-arg resolution
    paths.py                  # binary + FIFO/pty path resolution, suffix isolation
    session.py                # SitlSession: process lifecycle + orchestration
    transport/
      vsim.py                 # vsim ctl/pose framing (generated from vsim_proto.h)
      navlink.py              # telemetry decode (thin wrap of generated codec)
      rc.py                   # RC pty feeder
    bridges.py                # GCS pose fan-out FIFO + UART2 telemetry bridge
    world.py                  # world-mesh build + push (wraps worldmesh CLI)
    autopilot.py              # Pilot base + default cascade guidance (pluggable)
    server.py                 # session daemon (socket, command dispatch)
    client.py                 # client lib used by the CLI
    cli.py                    # `vayu-headless` argparse entrypoint
  cpp/worldmesh/              # (moved) BVH builder, or kept in tools/ and referenced
  tests/                      # pytest headless-flight suite
  examples/                   # roll-step sysID, ring course, tuning sweep
```

Lives in `software/` (per request) because it shares code with the GCS
(NavLink codec, `vsim_proto`, the mesh builder). The firmware **binaries** it
drives (`vsim_d`, `vayu_sitl`) stay in `tools/` and are resolved by path/env;
the SDK never rebuilds them.

## 5. Public API sketch (the contract to lock)

```python
from vayu_headless import SitlSession, Pilot

# Boot physics + real FC; match the GCS's selected vehicle/world; bridge to GCS.
with SitlSession.from_gcs_conf(gcs=True, world_collision=True) as sess:
    pilot = Pilot(sess, alt=-5.0)
    pilot.takeoff()
    pilot.fly(course=[(20, 0), (20, 20), (0, 20), (0, 0)])   # blocks until reached
    s = sess.status()                  # nav_state, armed, pose, telem counts
    truth = sess.truth()               # ground-truth pose dict
    att = sess.telemetry("AttitudeEuler")
    pilot.land()
```

Lower level, for sysID / custom controllers:
```python
sess.rc(roll=0.0, pitch=0.2, thr=0.5, yaw=0.0)   # raw stick, bypass autopilot
for row in sess.stream(hz=50): ...                # truth + telemetry samples
```

**Locked decisions to bake in:**
- Ground truth and telemetry are **read-only cached** accessors fed by internal
  threads (already the case post-pose-refactor); callers never touch fds.
- The autopilot only ever writes **RC sticks** (never position) except the
  explicit `takeoff`/`land` respawn — keep fidelity property: position is pure
  physics. (This is the principle established in this session.)
- One `SitlSession` == one isolated stack (private `VSIM_FIFO_SUFFIX`); many can
  coexist for parallel CI.

## 6. Session protocol (replace ad-hoc strings)

Today: newline command strings over `/tmp/sitl_lab.sock` (`takeoff`, `fly C a s`,
`status`, …). Standardise to a small **versioned JSON protocol**:
`{"v":1,"cmd":"fly","course":[[20,0]],"alt":-5,"timeout":40}` →
`{"ok":true,"status":{...}}`. Keep a human one-line text mode for the CLI.
Document every command + response shape once, in `README.md`.

## 7. Phased migration (behaviour-preserving)

- **Phase 0 — Skeleton + pin behaviour.** Create the package and `pyproject.toml`;
  add a pytest that boots a session, takes off, flies a box, lands, asserts
  `nav==ARMED`/altitude held (a golden of *current* behaviour). No logic moved
  yet. *DoD:* `pip install -e` works; the golden test passes against today's
  code path (imported from the script).
- **Phase 1 — Carve out modules.** Move `SitlLab` → `session.py` + `transport/*`
  + `bridges.py` + `world.py`, `Pilot` → `autopilot.py`, verbatim (no behaviour
  change). Re-run the Phase 0 golden each step. *DoD:* `sitl_lab.py` becomes a
  thin shim importing the package; golden still green.
- **Phase 2 — Config + paths + CLI.** `config.py` (GCS conf/env/args precedence),
  `paths.py` (binary + suffix isolation), `cli.py` (`vayu-headless serve|do|run`).
  *DoD:* no hard-coded `/home/...`/`/tmp/...` literals outside `paths.py`.
- **Phase 3 — Versioned session protocol.** JSON command schema in `server.py`/
  `client.py`; document it. Keep text CLI ergonomics. *DoD:* protocol doc +
  round-trip test.
- **Phase 4 — Pluggable autopilot + missions.** `Autopilot` interface; ship the
  current cascade guidance as the default; allow a custom controller / scripted
  mission (Python). *DoD:* a `examples/` mission runs end-to-end.
- **Phase 5 — Single-source wire defs, docs, CI.** Generate `transport/vsim.py`
  constants/structs from `vsim_proto.h` (or assert-match in a test) so they
  can't drift; finalise `README.md`; wire the flight suite into CI. *DoD:* CI
  job runs a headless flight and asserts; drift test guards the wire layer.

## 8. Shared-code strategy (avoid duplication / drift)

- **NavLink:** depend on `navlink/generated/python` (already the source of
  truth) — no copy.
- **vsim wire:** the harness hand-mirrors `vsim_proto.h` constants/struct
  formats. Phase 5: generate them or add a build-time/test-time assertion that
  Python `struct` sizes equal the C `static_assert` sizes, so a proto change
  fails loudly.
- **World mesh:** keep the C++ builder as the one implementation (it already
  reuses the GCS's `loadMesh`/`buildWorldBvh`); the SDK shells out to it.
  Decide in Phase 1 whether it moves to `software/headless-sdk/cpp/worldmesh/`
  or stays in `tools/` and is referenced.

## 9. Risks / watch-items

- **Wire drift** between Python and `vsim_proto.h`/NavLink — mitigated by the
  Phase 5 single-source/assert step.
- **Behaviour regressions** during the carve-out — mitigated by the Phase 0
  golden test landing *first*.
- **Hardware-specific quirks** already learned (stale UART2 advert; pose
  fan-out must not stall the physics drain; disarm+settle before respawn;
  thrust-axis sign; ground-clamp) must be preserved as **regression tests**, not
  re-discovered. See `~/.claude/.../memory/sitl-test-harness.md`.
- **Two consumers of one FIFO** (pose) — the publisher pattern is load-bearing;
  encode it in `bridges.py` with a comment + test.

## 10. Open decisions (flag before Phase 1)

1. **Package name / CLI name** — `vayu_headless` + `vayu-headless` (proposed) vs
   keep `sitl_lab`.
2. **Worldmesh C++ location** — move under `headless-sdk/cpp/` vs stay in
   `tools/` and reference.
3. **Back-comp** — keep `tools/sim_host/sitl_lab.py` as a thin shim
   indefinitely, or hard-cut once the CLI lands.
4. **CI scope** — smoke flight only, or the full sysID/course/tuning suite.

## 11. Definition of done (whole effort)

- `pip install -e software/headless-sdk` → `import vayu_headless` and
  `vayu-headless serve|do|run` both work.
- Everything today's script does is reachable through the package API + CLI.
- A CI job boots a headless session, flies a scripted course, and asserts
  on telemetry/ground truth.
- Wire layer is single-sourced or drift-guarded; protocol + API documented in
  `README.md`.
- `sitl_lab.py` is either a thin shim or removed (per decision #3).
```
