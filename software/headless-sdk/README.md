# vayu_headless

SDK for driving the **real** Vayu flight-controller logic in SITL — the same
estimator → angle/rate cascade → mixer → arming → telemetry that runs on the
FC — with no hardware and no human on the sticks. It only stubs the sensors and
RC; vsim_d handles the physics and actuation response.

See `PLAN.md` for the design/roadmap. Background: the firmware-in-the-loop
topology and quirks live in the project memory `sitl-test-harness`.

## Install

PEP 668 systems need a venv:

```bash
cd software/headless-sdk
python3 -m venv .venv
./.venv/bin/pip install -e ".[test]"
```

Needs the SITL binaries built (resolved from the repo, or via `VSIM_BIN_PATH` /
`VAYU_SITL_BIN`): `tools/vsim/build/vsim_d` and
`tools/sim_host/build_sitl/vayu_sitl`. For world collision, build the mesh tool:
`cmake -B cpp/worldmesh/build -S cpp/worldmesh && cmake --build cpp/worldmesh/build`.

## Library API

```python
from vayu_headless import SitlSession, Pilot

with SitlSession(gcs=False, conf="~/.config/Vayu/Vayu GCS.conf") as sess:
    pilot = Pilot(sess, alt=-5.0)     # NED z, <0 = up
    pilot.arm_takeoff()
    pilot.goto([(8, 0), (8, 8), (0, 8), (0, 0)])   # fly a box
    print(sess.truth())               # ground-truth pose dict
    print(sess.telem["AttitudeEuler"])# decoded FC telemetry
    pilot.land()
```

- `gcs=True` re-broadcasts pose to `/tmp/vsim_pose` and bridges UART2 telemetry
  so the Navigator GCS renders the flight ("Attach Ext") and shows live
  telemetry ("SITL UART2") — attach **once**, fly many.
- The autopilot only ever writes RC **sticks** (never position, except the
  takeoff/land respawn), so the flight stays genuine firmware-in-the-loop physics.

## CLI

```bash
vayu-headless serve --alt -5 --gcs-wait 20   # boot once; attach the GCS during the wait
vayu-headless do takeoff -5                  # send commands to the running session
vayu-headless do "fly 8,0;8,8;0,8;0,0 -5 40"
vayu-headless do status                      # text reply
vayu-headless do '{"v":1,"cmd":"status"}'    # JSON reply (versioned protocol)
vayu-headless do quit

vayu-headless run --course "8,0;8,8;0,8;0,0" --alt -5 --secs 40   # one-shot
```

Commands: `takeoff [alt]`, `goto <course> [alt]` (non-blocking), `fly <course>
[alt] [timeout]` (blocks until reached), `alt <z>`, `wait <s>`, `rc r p t y`,
`land`, `status`, `quit`. A course is `"N,E;N,E;..."`.

## Tests

```bash
./.venv/bin/python -m pytest tests -q                 # unit + integration
./.venv/bin/python -m pytest tests/unit -q            # fast, no daemon
./.venv/bin/python -m pytest -m integration -q        # boots vsim_d + vayu_sitl
```

Integration tests skip cleanly if the binaries aren't built. They assert
tuning-independent properties (stays ARMED, altitude held, telemetry streaming,
advances through the course) so the still-untuned outer guidance loop doesn't
make them flaky.

## Layout

```
vayu_headless/
  session.py     SitlSession — process lifecycle + I/O (SitlLab = alias)
  autopilot.py   Pilot + guidance_outputs() (pure cascade math)
  server.py      control server + command protocol (text + JSON)
  client.py      send one command to a session
  cli.py         vayu-headless serve|do|run
  config.py      GCS .conf parse + vehicle-geometry framing
  world.py       world collision-mesh build/push
  paths.py       binary / FIFO / singleton path resolution
  transport/     vsim (ctl+pose framing), navlink (telemetry), rc (channels)
cpp/worldmesh/   BVH builder (reuses the GCS's loadMesh + buildWorldBvh)
tests/{unit,integration}/
examples/box_mission.py
```
