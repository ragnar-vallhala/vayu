# Sim: mesh geometry, computed MoI, and a motor-mapping editor

Status: ✅ shipped as FR-SIM-11 (Phase-1 item 1e mostly closed; sensor-noise
editing FR-SIM-04 still pending). See [`../requirements.md`](../requirements.md).
Headless coverage: `tools/vsim/tests/massprops_test.cpp` (analytic inertia +
`Mat3` inverse + STL import) and `simworker_smoke.cpp` (geometry push over the
ctl FIFO without desync).

## Context

The `vsim_d` simulator currently flies a hardcoded X3-class quad: a
**diagonal-only** inertia (`DroneParams.inertia_diag`, a `Vec3`) and a
**fixed 4-motor layout** baked into `tools/vsim/include/vsim_types.h`,
rendered as a procedural box + disks. There is no way to use a real
airframe's mass distribution or to reposition the motors without
recompiling.

This initiative lets a user load their drone's **mesh** (STL/glTF/OBJ),
have the GCS compute the **full 3×3 moment-of-inertia tensor** from that
geometry, edit the **4 motors'** body positions / thrust axes / spin /
coefficients in a Gazebo-style editor, and push it all live to the
running daemon. Outcome: the sim flies the actual airframe's dynamics
and the 3D view shows the real shape with motor markers.

## Decisions

- **Geometry input:** tessellated **mesh via assimp** (already
  installed). No OpenCASCADE. Inertia is computed from the closed mesh
  assuming **uniform density normalized to a user-entered target mass**.
- **Motors:** stay at **4** (matches firmware mixing + the 4-wide
  pwm/pose wire frames); make each motor's position, thrust axis, spin,
  and thrust/torque coefficients editable.
- **Editor UX (v1):** **numeric parameter grid + motor markers** drawn
  on the rendered mesh (no interactive drag-gizmos).

## Architecture principle

Keep the daemon **dependency-free**. All mesh loading, inertia
computation, and editing happen GCS-side (Qt/OpenGL). The GCS pushes a
compact POD payload (mass + 3×3 inertia + 4-motor layout) to the daemon
over the existing **`/tmp/vsim_ctl`** FIFO via a new opcode. This
preserves the two-process split. Scope is entirely `software/` +
`tools/vsim/` — **no firmware edits**.

## Work items

1. **Daemon math** — `tools/vsim/include/vsim_math.h`: header-only
   `Mat3` (row-major) with symmetric/diagonal builders, `Mat3*Vec3`,
   `Mat3*Mat3`, and `inverse()` (closed-form 3×3, det≈0 guard).
2. **Daemon params** — `tools/vsim/include/vsim_types.h`:
   `DroneParams.inertia_diag (Vec3)` → `inertia (Mat3)`; `MotorParams`
   gains `axis_b[4]` (default `{0,0,-1}`) and per-motor
   `k_thrust`/`k_moment`/`max_omega` arrays.
3. **Daemon physics** — `physics_core.{h,cpp}`: cache `I_inv`,
   `d_omega = I_inv·(τ − ω×(I·ω) − drag·ω)`. `motor_model.cpp`: thrust
   along per-motor `axis_b[i]`, per-motor coefficients, reaction torque
   about that axis.
4. **Daemon protocol** — `tools/vsim/include/vsim_proto.h`: add
   `VSIM_CTL_SET_GEOMETRY = 5`, POD `vsim_ctl_geometry_t` (mass + 9-float
   inertia + 4×{pos[3],axis[3],spin,k_thrust,k_moment,max_omega} ≈ 200 B),
   enlarge ctl body `64 → 256`, update static_assert. **No
   `VSIM_PROTO_VERSION` bump / no firmware impact** — the ctl frame is
   Navigator↔daemon only; pwm/imu/pose frames are byte-identical.
   `main.cpp`: dispatch case → `setDroneParams` + `setMotorParams`.
5. **GCS mesh + mass props** — new `software/src/vsim/MeshLoader.{h,cpp}`
   (assimp wrapper → verts/normals/indices + triangle soup) and
   `MassProperties.{h,cpp}` (closed-polyhedron integral → volume, CoM,
   inertia at density 1, scaled to target mass). Pure math, unit-tested.
6. **GCS renderer** — `SimRendererWidget`: `setDroneMesh(...)` upload to
   the existing VBO/VAO, a normal attribute + directional Lambert term in
   the shader, motor markers along `axis_b` colored by spin, CoM
   crosshair. Procedural body stays as the no-mesh fallback.
7. **GCS editor** — new `software/src/ui/widgets/GeometryEditorWidget.
   {h,cpp}`: mesh picker + target-mass, **Load & Compute** (shows tensor
   + CoM offset diagnostic), 4-row motor grid, **Apply** →
   `geometryApplied(GeometryConfig)`. Mirrors `SettingsWidget` /
   `CalibrationWidget` form idioms + `core/Theme.h` + `core/ui/Buttons.h`.
8. **GCS wiring** — `SimWorker::sendGeometry(...)` (modeled on
   `sendReset()`); `GeometryConfig` in `VsimTypes.h`; `SimulatorWidget`
   hosts the editor and routes `geometryApplied` to renderer + daemon;
   QSettings persistence under `sim/geometry/*`.
9. **Build** — `software/CMakeLists.txt`: `find_package(assimp)`, link
   `assimp::assimp` inside the SITL gate, add the new sources.
10. **Tests + docs** — cube analytic-inertia + `Mat3` inverse unit tests,
    extend `simworker_smoke` with `sendGeometry`, ship a sample STL,
    update `requirements.md` (FR-SIM-03/04, new FR-SIM-11, Phase-1 1e).

## Dynamics are about the center of mass

The CoM, not the model origin, is the reference for all dynamics. After
the mass-properties integral yields the CoM, the GCS:
- recenters the imported mesh on the CoM (`loadAndCompute` shifts every
  vertex by −CoM), and
- exposes `GeometryEditorWidget::physicsConfig()`, which shifts the motor
  arms to be CoM-relative and zeroes the offset.

Both the renderer and the daemon consume that CoM-frame config, and the
inertia tensor is already computed about the CoM, so the tracked point,
the torque arms, and the inertia all share one reference. The daemon
needs no change — it already treats its tracked point as the CoM. The
editor still *displays* the CoM offset from the model origin as
information. `cfg_` (and persistence) stay in the user's model-origin
frame for intuitive editing.

## v1 limitations (documented, not bugs)

- Uniform density only (no per-part materials).

## Interactive motor gizmos (shipped)

Blender-style direct manipulation of motors in the 3D view, active only
when the sim is stopped (`SimRendererWidget::setMotorsEditable`). Click a
motor marker to select it (screen-space pixel pick), then **G** to move /
**R** to rotate the thrust axis, with **X/Y/Z** to lock to a body axis;
click confirms, **Esc** cancels. Move drags on the camera-facing plane
(or the locked axis); rotate spins the thrust axis about the locked body
axis. On confirm the renderer emits `motorEdited(index, posComFrame,
axis)`; `GeometryEditorWidget::setMotorFromGizmo` converts back to the
model-origin frame, updates the numeric grid + config, and refreshes the
preview. All gizmo work is in the CoM frame the renderer/daemon share.

## Out of scope (future)

Arbitrary motor count (hexa/octo — needs wider wire frames + firmware),
true STEP via OpenCASCADE, per-part densities, time-varying inertia,
free (view-axis) rotate and translate-snap increments.

## Coordination note

`vsim_proto.h`'s ctl-body enlargement is GCS↔daemon-only and needs no
firmware rebuild for correctness, but the firmware instance `#include`s
this header — flag the struct-size change so a coordinated rebuild picks
it up.
