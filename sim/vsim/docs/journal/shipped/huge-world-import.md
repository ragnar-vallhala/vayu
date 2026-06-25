# Plan: Import huge worlds (.blend/.glb/.obj) with rigid triangle-mesh collision

> **DELIVERED.** Phases P0–P3 landed and tested and most of P4; shipped on
> **2026-06-03** — see `docs/changelog/gcs-in-app-simulator-and-world-collision.md`
> ("Huge-world import, rigid mesh collision & camera modes"). The per-phase plan
> below is retained as design rationale, collapsed. Only the deferred P4 items
> remain open.

Status: 🟢 implemented (P0–P3 landed + tested; P4 mostly satisfied, rest deferred)

**Still deferred** (no real >2 M-tri asset yet to justify the cost): swept-sphere
CCD, vertex welding, decimate-on-import, frustum cull, "show collision proxy"
debug overlay.

<details>
<summary>Delivered phase plan + design rationale (historical)</summary>

Progress:
- ✅ **P0** `trimesh_bvh.h` + `trimesh_bvh_test` (12/12) — commit `444dbc0`.
- ✅ **P1** visual import (MeshLoader baked transform, renderer worldMesh_, World tab
  import UI, `.vworld`/QSettings persistence) — commit `786f342`.
- ✅ **P3** collision: `applyContact()` refactor + `resolveWorldMesh()`, footprint-vs-mesh
  impulse, winding-independent normals; `world_collision_test` (5/5) — commit `f6f8fd3`.
- ✅ **P2** transport: `VSIM_CTL_SET/CLEAR_WORLD_MESH` + `vsim_ctl_world_mesh_t`, daemon
  mmap+validate, `WorldMeshBuilder` (Qt-free BVH bridge — quarantines the `vsim::Vec3`
  struct/QVector3D-alias clash), `SimWorker::sendWorldMesh`, write-tmp-then-rename publish,
  re-push on `online()`; `world_mesh_transport_test` end-to-end vs real `vsim_d` (6/6) —
  commit `c1373cd`.
- 🟡 **P4** partially done: re-import/clear/exit all `munmap` (daemon `drop_world_mesh`,
  fd closed post-mmap); render VBO + BVH bytes share one baked `LoadedMesh` so they can't
  disagree.

## Context
The simulator's "world" is today a handful of analytic primitives (box/sphere/cylinder) with
collision in `vsim_d` (`sim/vsim/src/physics_core.cpp::resolveObstacles`). We want to import a
whole **huge** world mesh and have the drone physically collide with it as a **static rigid
surface** — the drone's contact points vs the world's triangles (closest-point-on-triangle →
impulse). assimp 5.3 (already linked) reads `.blend/.glb/.obj`, and the renderer already has a
lit-mesh path, so *rendering* a world is cheap; the real work is (a) getting a large mesh across to
the separate `vsim_d` process and (b) colliding against arbitrary triangles at 1 kHz without testing
every triangle.

**Approach:** a shared, dependency-free **BVH** built GCS-side, handed to `vsim_d` via an
**mmap'd file** (the 256 B ctl FIFO can't carry a mesh), traversed each physics step and fed into
the **existing** footprint-impulse contact model. Phased so visual import lands first and the
dependency-free collision math is independently testable.

## Grounding (verified in code)
- Daemon loop `sim/vsim/src/main.cpp`: single thread, 1 kHz (`kPhysicsHz`), `ctl.tick(duty,dt)`;
  IMU every `kImuDiv`(=5) ticks (the substep-cache window). ctl switch is where new subtypes slot in.
- Collision `physics_core.cpp::resolveObstacles()`: 4-point footprint `fp[4]=(±0.13,±0.13,0)`,
  contact radius `rc=0.04`; the impulse block (≈ lines 167-203) is self-contained → extract to
  `applyContact()`.
- Math `sim/vsim/include/vsim_math.h` (Vec3/Quat/Mat3, dep-free) — shared between GCS and daemon
  today; the new BVH header belongs alongside it.
- Wire `sim/vsim/include/vsim_proto.h`: `vsim_ctl_frame_t` body is only **256 B** (subtypes to 8).
- Mesh load `navigator/src/vsim/MeshLoader.cpp`: assimp → flat triangle soup `LoadedMesh{positions,
  normals,bbox}`; `.blend` confirmed supported.
- Renderer `navigator/src/vsim/SimRendererWidget.{h,cpp}`: lit path `setDroneMesh→uploadLitMesh→drawLit`;
  world frame NED (up = −Z). No mmap mechanism exists yet.
- Tests `sim/vsim/tests/*` are standalone `main()` 0/1; a pure-`vsim_math` BVH test needs no Qt.

## Phase 0 — Shared trimesh + BVH module (foundation)
New header-only `sim/vsim/include/trimesh_bvh.h` (depends only on `vsim_math.h`), used by **both**
GCS (build+serialize) and `vsim_d` (mmap+traverse).
- **POD file format** (LE, 4-byte fields, explicit offsets, 16-byte-aligned regions):
  `TrimeshHeader{magic 'VWLD', version, vertex_count, triangle_count, node_count, flags(double_sided),
  aabb_min/max, *_off}` → vertices `float[N][3]` (NED world-space, transform baked) →
  triangles `uint32[M][3]` → `tri_perm uint32[M]` → `BvhNode{bmin[3],bmax[3],left,first_tri,tri_count,_pad}` (32 B).
- **Builder** (GCS, off realtime thread): `buildSerialized(verts,tris,doubleSided)->vector<uint8_t>`,
  median split on longest AABB axis, leaf ≤ ~8 tris, flat node array. (`// TODO: SAH`.)
- **Query** (daemon, alloc-free, fixed stack): `Bvh::fromBytes(base,len)` (validate magic/version/offsets);
  `querySphere(bvh,center,r,out[],cap)->count`; `closestPointOnTriangle(p,a,b,c)` (Ericson barycentric).
  Add free `dot/length/normalized` (Vec3 lacks them).
- **Test** `sim/vsim/tests/trimesh_bvh_test.cpp` (no Qt): floor+ramp → querySphere/closestPoint asserts,
  serialize→fromBytes round-trip.

## Phase 1 — Visual import (de-risk the pipeline first)
- `navigator/src/vsim/MeshLoader.{h,cpp}`: bake a **Blender→NED + scale + up-axis** transform per vertex
  (normals by rotation only); keep the airframe call site (identity) unchanged.
- `navigator/src/vsim/SimRendererWidget.{h,cpp}`: `worldMesh_` + `setWorldMesh(pos,nrm)` mirroring the
  `droneMesh_` deferred-upload machinery; draw `drawLit(worldMesh_, view, identity, color)`.
- `navigator/src/vsim/VsimTypes.h` `WorldConfig`: add `worldMeshPath, worldScale, worldUpAxis,
  worldRestitution, worldDoubleSided`. Import button + scale/up-axis controls in `WorldEditorWidget`;
  persist in `persistWorld/restoreWorld` (QSettings) + `.vworld` JSON.
- Risk: `.blend` is best-effort in assimp (modifiers/instances may drop) — validate tri-count/bbox,
  surface `GetErrorString`; recommend `.glb` as primary.

## Phase 2 — Transport GCS → vsim_d (mmap + small ctl frame)
- GCS after import: `buildSerialized()` → write to `fifoSuffixed("/tmp/vsim_world")+".bin"` via
  **write-`.tmp`-then-`rename()`** (atomic; daemon never maps a half-written file). v1 may keep the flat
  soup as indices `{3i,3i+1,3i+2}` (vertex weld is a later optimization).
- `vsim_proto.h`: `VSIM_CTL_SET_WORLD_MESH=9`, `VSIM_CTL_CLEAR_WORLD_MESH=10`, and
  `vsim_ctl_world_mesh_t{vertex_count,triangle_count,node_count,flags,world_restitution,path_len,char path[216]}`
  (static_assert ≤ 256; counts carried redundantly for cross-check).
- `navigator/src/vsim/SimWorker.{h,cpp}`: `sendWorldMesh(...)` + `clearWorldMesh()` (same ctl boilerplate
  as `sendObstacles`).
- `sim/vsim/src/main.cpp`: ctl cases → `open/fstat/mmap(PROT_READ)`, `Bvh::fromBytes` validate
  (counts match the frame), forward to `SimController::setWorldMesh(view,restitution)`; CLEAR → `munmap`.
  RAII holder unmaps on re-import/exit.
- Lifecycle: `SimulatorWidget` `online()` re-pushes the world mesh on daemon respawn; re-import rewrites
  `.bin` + resends; per-instance `$VSIM_FIFO_SUFFIX` isolates concurrent Navigators.

## Phase 3 — Collision in physics_core
- **Refactor**: extract the impulse block of `resolveObstacles()` into
  `applyContact(cpb_body, nW, pen, restitution, posCorr&, maxPen&)` — bit-identical (keep the
  rotational-coupling effective mass `keff = 1/m + dot(nB, cross(I_inv_*(cpb×nB), cpb))`, friction,
  push-out). Primitive path calls it; the Phase-3 test regression-guards the refactor.
- **`resolveWorldMesh()`** called from `step()` after `resolveObstacles()`. For each contact point `cpb`
  (configurable set; default the 4 footprint pts ± CoM/top): `Pw = pos + att·cpb`; `querySphere(bvh,Pw,rc)`;
  for each candidate `cp = closestPointOnTriangle(Pw,a,b,c)`, `d = Pw - cp`; if `|d| < rc`:
  **normal `nW = d/|d|` from the closest-point direction, NOT face winding** (double-sided), `pen = rc - |d|`,
  `applyContact(...)`. Degenerate `|d|≈0` → face normal disambiguated by contact-velocity sign.
- `physics_core.h`: `setWorldMesh(Bvh,restitution)` + members; `sim_controller.h` forwards like `setObstacles`.
- **Tunnelling** (≤1 kHz): 40 mm skin (`rc`) + **swept-sphere** query (center at midpoint of `Pw_prev→Pw`,
  radius `rc + 0.5|motion|`; cache `Pw_prev`). True CCD (ray-vs-BVH) deferred.
- **Perf/caching**: 1 kHz × ~5 pts × O(log N) is trivial at 1 M tris; cache `querySphere` candidates per
  point, re-query every `kImuDiv`(5) ticks or when the point moves > ~`0.5·rc`.
- **Test** `sim/vsim/tests/world_collision_test.cpp` (no Qt): drop on floor → rests at surface, no
  fall-through; off-center ramp → acquires roll/pitch + slides; thin wall → stops below a documented speed.

## Phase 4 — Robustness / huge-world specifics
Re-import/clear/unmap lifecycle (no fd/mem leak — `/proc/<pid>/maps`); a single shared baked transform used
by **both** render VBO and BVH bytes; optional "show collision proxy" debug overlay. Frustum cull /
decimate-on-import only if a real >2 M-tri asset is slow (else deferred).

## Critical files
- New `sim/vsim/include/trimesh_bvh.h`.
- `sim/vsim/src/physics_core.cpp` + `physics_core.h`; `sim/vsim/include/vsim_proto.h`;
  `sim/vsim/src/main.cpp`; `sim/vsim/include/sim_controller.h`.
- `navigator/src/vsim/SimWorker.{h,cpp}`, `SimRendererWidget.{h,cpp}`, `MeshLoader.{h,cpp}`, `VsimTypes.h`;
  `navigator/src/ui/widgets/WorldEditorWidget.{h,cpp}`, `SimulatorWidget.cpp`.

## Verification
1. **Unit (no deps, single g++ line, exit 0/1 like `massprops_test.cpp`):** `trimesh_bvh_test` (P0),
   `world_collision_test` (P3) — fastest, highest value, also guard the `applyContact` refactor.
2. **Daemon transport smoke** (extend `simworker_smoke`): spawn vsim_d, write a real BVH `.bin`, send
   `SET_WORLD_MESH`, PWM hover→descend, read pose, assert it rests on the imported floor.
3. **In-app:** import `.glb` and `.blend` in the World tab, fly into geometry (bounce/tip/slide),
   re-import, clear → falls only to `ground_z`. Build: `cmake --build build_vsim --target vsim_d`,
   `cmake --build navigator/build --target Navigator`.

## Top risks
Tunnelling through thin geometry at speed (skin + swept query; CCD later) · `.blend` partial import
(prefer `.glb`, validate) · transport torn/stale mmap (write-then-rename + header validation + path cap) ·
render/collision frame drift (one shared baked transform) · perf if caching wrong (substep cache + test).

## Effort
≈ 9 working days, front-loaded on the dependency-free, independently-testable BVH + collision math (P0+P3).

</details>
