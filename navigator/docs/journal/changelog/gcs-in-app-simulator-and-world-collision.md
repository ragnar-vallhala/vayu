# Navigator GCS & In-App Simulator — Changelog

The ground-control station (`navigator/`, Qt6 — "Navigator") and the in-app
software-in-the-loop simulator it hosts (`sim/vsim/` `vsim_d` physics daemon +
the firmware SITL harness). Newest first.

Architecture in brief: Navigator embeds the firmware (`vayu_sitl_core`) and
spawns a standalone **`vsim_d`** physics daemon; the two talk over `/tmp` FIFOs
(pwm / imu / pose / ctl), isolated per Navigator instance by
`$VSIM_FIFO_SUFFIX`. All coordinates are NED (Z down).

Log-analysis tooling: `tools/analysis/sim_log_to_csv.py` converts a captured sim log to
CSV and `tools/analysis/sim_log_plot.py` renders the per-domain plots.

---

## 2026-06-03 — Huge-world import, rigid mesh collision & camera modes

### Added

- **Import huge worlds (`.glb`/`.gltf`/`.obj`/`.stl`/`.blend`) with rigid
  triangle-mesh collision**, delivered in phases (plan:
  `docs/roadmap/huge-world-import.md`):
  - **P0 — shared BVH module** `sim/vsim/include/trimesh_bvh.h`: a
    header-only, dependency-free triangle-mesh BVH (median split, flat node
    array) with a self-describing serialized blob format, `Bvh::fromBytes`,
    `querySphere`, and Ericson `closestPointOnTriangle`. Unit test 12/12.
  - **P1 — visual import:** `MeshLoader` bakes an up-axis→NED + scale transform
    per vertex; the renderer gained a lit world-mesh path; the World tab grew an
    import/clear UI; persisted to QSettings and the `.vworld` file.
  - **P3 — collision in `vsim_d`:** extracted the impulse contact (normal +
    rotational coupling + capped friction + push-out) into `applyContact()`, and
    added `resolveWorldMesh()` — the 4-point drone footprint vs the world BVH
    with **winding-independent** normals (works on either side of a face).
    Standalone test 5/5.
  - **P2 — transport:** `VSIM_CTL_SET_WORLD_MESH` / `CLEAR_WORLD_MESH` +
    `vsim_ctl_world_mesh_t`; the GCS serializes the BVH and the daemon mmaps it
    read-only (the 256 B ctl frame can't carry a mesh). `WorldMeshBuilder` is a
    Qt-free bridge that keeps `trimesh_bvh.h`'s `vsim::Vec3` struct from clashing
    with the GCS's `vsim::Vec3 == QVector3D` alias. Atomic publish
    (write-tmp-then-rename); re-pushed on daemon `online()`. End-to-end test
    (spawns real `vsim_d`) 6/6.
- **Configurable simulation rates + physics substepping:** a rate panel
  (IMU / physics / pose Hz); the daemon paces at the IMU/firmware-loop rate and
  runs `physics_hz / imu_hz` RK4 substeps per sample, so physics can integrate
  at **8–10 kHz** without sleep-jitter.
- **Per-mode 3D view + camera modes:**
  - Vehicle mode shows just the airframe; World mode shows the whole world
    (imported mesh + obstacles).
  - **Free-roam camera** (WASD / Q-E / left-drag / scroll) in World mode while
    stopped — not locked to the drone — so you can roam an imported world while
    still editing obstacles.
  - On **Start** the camera locks a 3rd-person orbit on the drone and the **FPV**
    toggle is enabled; **Stop** restores free-roam.
- **World placement offset** (NED N/E/D metres) in the World tab — move an
  imported world off the drone's spawn (e.g. lift a building so the drone isn't
  trapped under it). The same baked mesh drives render **and** collision, and the
  offset re-bakes/re-ships the BVH live while running.

### Fixed

- **Motor layout now rigidly attached to the airframe body transform:** the
  body Translate/Rotate (e.g. Rotate=180° to fix an imported up-axis) is applied
  to the motors as well as the mesh, then made CoM-relative — so rotating/moving
  the airframe no longer strands the rotor markers, and 3D-gizmo edits round-trip
  through the inverse transform.

---

## 2026-06-01 — World editing, obstacles, FPV & audio

### Added

- **Static world obstacles:** add / edit / render boxes, spheres, and cylinders
  in the World tab (phase 1, visual).
- **Blender-style gizmos** for obstacles in the 3D view (click-select, G move /
  R rotate / S scale, X/Y/Z constrain).
- **World Save/Load** to a portable `.vworld` file.
- **Obstacle collision in `vsim_d`** (phase 2): the drone collides with the
  primitive obstacle set.
- **Contact torque** — the drone tips off obstacle edges (rotational coupling on
  contact), and a **tipped airframe topples back to level** on the ground; the
  **ground-righting gain/damp** are exposed in the World editor.
- **Onboard FPV camera** in the 3D view (rides the drone, looks forward; body
  mesh hidden so it doesn't fill the lens).
- **Propeller audio** synthesised from motor rpm.
- **HUD** overlay: flight state + accel/gyro mini-plots.
- **RC input source** selector: USB joystick or UART CSV.
- Status bar shows **"Connected: SIM"** while the in-app sim is live, and
  serial-connection controls are **locked** so you can't double-drive the link.

### Fixed

- `SimWorker::killDaemon()` made thread-safe (pid-kill race between the GUI and
  worker threads).
- Stop killing the freshly-spawned `vsim_d` on sim restart.
- Quiet telemetry + grey out Start/Stop on sim Stop; persist the selected
  vehicle (not only on Apply); fixed HEALTH counters rendering as a tofu box in
  the status pill.

---

## 2026-05-30 → 2026-05-31 — SITL daemon split & per-instance isolation

### Added

- **Mesh-derived inertia + Gazebo-style motor-mapping editor:** load an airframe
  mesh, compute its CoM + full inertia tensor at a target mass, and edit the
  4-motor layout (position, thrust axis, spin, coefficients).
- **Firmware-side SITL host harness + GCS bring-up** — the firmware runs
  in-process inside Navigator and talks to the simulator over the FIFO transport.
- **Per-instance FIFO paths via `$VSIM_FIFO_SUFFIX`**, coordinated across
  `vsim_d`, the firmware host, and the GCS so two Navigators don't cross streams.
- `docs/roadmap/` to track cross-cutting work.

### Changed

- **Split the in-app physics into a standalone `vsim_d` daemon** — `SimWorker`
  became a thin process supervisor + pose decoder; all physics/sensor sim moved
  to the separate binary.
- Compute dynamics **about the CoM**, and keep the geometry editor laid out
  in-column.

---

## 2026-05-26 — In-app simulator replaces Gazebo

### Added

- **In-app C++/OpenGL simulator** replacing the external Gazebo SITL — a single
  RK4 rigid-body integrator + OpenGL renderer inside Navigator.
- **Navigator UX foundation** (Phase 0 + Phase 1a/1c/1d).

---

## 2026-05-24 — Early SITL plumbing (Gazebo era, since retired)

Historical; superseded by the in-app simulator above. Kept for context.

### Added / Changed

- Wired **UART2 telemetry into a pty + raw log** for post-sim analysis, and let
  the GCS port combo accept a custom path (e.g. the SITL UART2 pty).
- A pty-backed **virtual RC panel** was added and then reverted.
- Iterated the Gazebo physics bridge (per-rotor wrench application, single
  wrench publisher, `ApplyLinkWrench`, rotor-pos/spin alignment with the mixer,
  raised max rotor velocity for TWR ≈ 3.3) — all later replaced by the in-app
  simulator.

---

## Earlier (2026-03-06)

- **Initial Navigator application:** UI widgets, drone protocol decode, and
  serial communication — the GCS baseline. The first major feature pass (iBus RC
  monitor, packet-analyzer filtering, frequency ribbon) is written up in
  [implement-rc-telemetry-and-gcs-ui.md](implement-rc-telemetry-and-gcs-ui.md).
