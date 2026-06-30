# Procedural world generation for the SITL renderer

## Context

The SITL renderer (`navigator/src/vsim/SimRendererWidget.*`, Qt6 + OpenGL 3.3
core, GLSL `330 core`) can today show a world in exactly two ways:

1. A flat 10 m × 10 m line grid at `z = 0` (`buildGroundGrid()`), or
2. An **imported** mesh — user picks a `.glb`/`.gltf`/`.obj`/`.stl` in the World
   tab, `MeshLoader` (assimp) triangulates it, `setWorldMesh(pos,normal,color)`
   defers an interleaved `[px py pz nx ny nz r g b]` upload into a VBO, and it
   draws through the per-vertex-lit Lambert shader. Collision comes from
   `WorldMeshBuilder::buildWorldBvh()` → `VSIM_CTL_SET_WORLD_MESH` → the
   `vsim_d` daemon mmaps the BVH blob and queries it per physics tick.

There is **no procedural geometry of any kind** beyond hand-coded primitives
(box body, rotor disk, training-gate torus, box/sphere/cylinder obstacles). No
noise, no heightfield, no scatter.

The goal is to **generate worlds automatically**, with several interchangeable
generation methods, the first concrete target being a **photorealistic meadow**
(rolling green hills, mountains, a river valley, grass + wildflowers, hazy
atmosphere — the reference screenshot). Hard requirements from the brief:

- **Nothing hardcoded per-biome.** The reusable part is the *machinery*; a biome
  is a *recipe* on top of it. Adding a desert, jungle, or city must reuse the
  terrain/scatter/material/render machinery and supply only new parameters and a
  few biome-specific rules.
- **Procedural primitives now, imported assets later.** For the meadow we
  generate everything (terrain, grass, flowers) with no imported meshes. Later,
  imported tree/building/rock meshes plug into the *same scatter system* so a
  city or jungle reuses all the placement logic — the scatter system must not
  care whether an instance is procedural or imported.

This is a **design plan only**. No code is written here. File:line references
are pointers to the live tree at time of writing, not contracts.

### Hard constraints to design around

- **OpenGL 4.6 core, behind a capability seam.** We target GL 4.6 on the primary
  platforms (Linux / Windows desktop), which unlocks **compute shaders, tessellation,
  SSBOs, and indirect draw**. This is a one-line bump: `navigator/src/app/main.cpp:11`
  (`fmt.setVersion(3,3)` → `4,6`) plus switching the renderer base class from
  `QOpenGLFunctions` to `QOpenGLFunctions_4_6_Core` / `QOpenGLExtraFunctions`.
  The default-format change is global; the existing `AttitudeWidget`/`Drone3DWidget`
  use the common subset and run unchanged under a 4.6 core context.
  - macOS (GL 4.1 ceiling, **no compute ever**; Apple froze GL) is **not a target
    now** but must stay *fillable later*. So all version-specific GL lives behind a
    **render-backend seam** (see next section), SDL-style: high-level code talks to
    capabilities + feature interfaces, never to raw 4.6 calls. A future portable
    backend (CPU geomipmap + instanced flora, no compute/tess) slots in without
    touching the procgen layers or the scene logic.
  - "Volumetric" fog stays analytic (height/distance exponential, sun-tinted) — it
    looks right and keeps the portable backend trivial; raymarched volumetrics are
    out of scope.

### Render-backend seam (SDL-style)

Do **not** build a full multi-backend renderer now — that's over-engineering. Build
*one* 4.6 backend, but isolate it so a second backend is additive, not a rewrite:

- **`GpuCaps`** — queried once at `initializeGL`: GL version, `hasCompute`,
  `hasTessellation`, `hasSSBO`, max instance count. Everything branches on caps,
  never on a hardcoded version.
- **Feature interfaces, not feature calls.** The two subsystems that *want* 4.6 —
  `TerrainLod` (tessellation/displacement) and `FloraField` (compute cull + indirect
  draw) — are coded against interfaces with two intended implementations:
  - `Gl46TerrainLod` / `Gl46FloraField` — built now, use tess + compute.
  - `PortableTerrainLod` / `PortableFloraField` — CPU geomipmap + plain instanced
    arrays; **left as a documented stub/seam now**, filled if/when macOS matters.
  The renderer picks an implementation from `GpuCaps` at startup.
- **Rule:** no `gl*` call that requires > GL 3.3 may appear outside a `Gl46*` class.
  This is the whole seam — keep it and the macOS path is a contained add later.
- **Wire protocol.** `vsim_proto.h:63` — `payload_bytes` is `uint32`, so there
  is no practical mesh-size limit on the world-mesh transport. A 256² heightfield
  is ~130 k tris; the BVH blob is the thing to watch, not the header.
- **Determinism.** Everything seeds from one integer. Same seed + same params →
  byte-identical world, on render *and* physics side. No wall-clock, no
  unseeded RNG.
- **Two consumers, one source.** Every generated world must feed *both*
  `setWorldMesh()` (render) and `buildWorldBvh()` (collision) from the same
  triangle data, so what you see is what you crash into.

---

## Design overview — four layers

The single most important decision: **separate the generation machinery (layers
1–2, biome-agnostic, Qt-free) from biome recipes (layer 3, data) and from the
renderer (layer 4).** This is what makes "city/desert/jungle reuse the code"
true rather than aspirational.

```
 Layer 4  Renderer            terrain shader · instanced flora · atmosphere · sky
            ▲ mesh+splat+normals          ▲ instance buffers
 Layer 3  Biome recipe        Meadow = {TerrainGen, [MaterialLayer], [ScatterLayer]}
            ▲ composes                     (Desert, Jungle, City = other recipes)
 Layer 2  Asset providers     InstanceSource  (procedural quad/cross/rock  |  imported .glb)
            ▲ provides renderable + collision proxy per scattered item
 Layer 1  Generation core     Noise · Heightfield · Scatter · Splat/masks · MeshAssembly
            (Qt-free, std::vector + plain structs, unit-tested)
```

The flow at world-load time:

```
.vworld (v2, procedural block)
   → BiomeRegistry.create(name, seed, params)         // layer 3
   → TerrainGenerator → Heightfield                   // layer 1
   → MaterialLayers   → per-vertex splat/colors        // layer 1
   → MeshAssembly     → triangle mesh (+normals,+uv)   // layer 1
   → ScatterLayers    → instance lists per InstanceSource  // layer 1+2
   → emit:  setWorldMesh(mesh)        [render]
            buildWorldBvh(mesh)       [collision → VSIM_CTL_SET_WORLD_MESH]
            uploadInstances(scatter)  [render only — flora has no collision]
```

### Layer 1 — generation core (biome-agnostic, Qt-free)

New module `navigator/src/vsim/procgen/`. Follows the `WorldMeshBuilder`
precedent: **Qt-free**, plain `struct Vec3 {float x,y,z;}` + `std::vector`, so it
unit-tests without a GL context and could later be shared with `sim/vsim/`.

Components, each independently useful and reusable:

- **`noise/`** — fBm Perlin/Simplex, ridged multifractal (mountains), domain
  warping (natural-looking valleys/rivers), Worley/Voronoi (city blocks, cracked
  desert, cell patterns). All seeded. These are the "several generation methods":
  a `TerrainGenerator` is just a named composition of these.
- **`Heightfield`** — 2D scalar grid + bilinear sample, analytic gradient →
  normals, and optional **erosion** passes (thermal talus + simple hydraulic) for
  the realistic ridge/gully look. World-space size and resolution are params.
- **`Scatter`** — Poisson-disk (blue-noise) sampler and stratified-jitter grid,
  driven by a **density field** (a function of altitude/slope/biome-mask). Returns
  positions + per-instance yaw/scale/tint. Biome-agnostic: it scatters *handles*,
  not grass specifically.
- **`Splat / biome-mask`** — per-vertex or per-texel material weights from
  altitude, slope, moisture/temperature fields (Whittaker-style). Drives both
  vertex colors (Phase 0) and the splatmap terrain shader (Phase 2).
- **`MeshAssembly`** — heightfield → indexed triangle mesh with normals, UVs,
  vertex colors, and **chunking + LOD rings** (geomipmap) for the renderer.

### Layer 2 — asset providers (the procedural-now / imported-later seam)

```cpp
// Qt-free interface. The scatter system holds these; it never branches on kind.
struct InstanceSource {
    virtual MeshData    renderMesh(uint32_t lod) const = 0;  // for instanced draw
    virtual ColliderRef collider() const = 0;                // none | obstacle | bvh
    virtual ~InstanceSource() = default;
};
```

Concrete providers:

- **Procedural (now):** `GrassQuad`, `FlowerCrossQuad` (two perpendicular quads,
  alpha-cut texture), `ProcRock` (noise-displaced icosphere). Collider = *none*
  for grass/flowers.
- **Imported (later):** `MeshAssetSource` wrapping a `MeshLoader` result for a
  `tree.glb` / `building.glb`. Collider = obstacle primitive or its own small
  BVH. **Crucially, the scatter layer and the renderer are unchanged** when this
  arrives — that is the whole point of the seam. A jungle biome is then "meadow
  terrain recipe + dense `MeshAssetSource(tree.glb)` scatter"; a city is "Voronoi
  block terrain + `MeshAssetSource(building.glb)` on lots."

### Layer 3 — biome recipes (data, composable)

A biome is **data + a registry entry**, not a class hierarchy of hardcoded
content:

```cpp
struct BiomeRecipe {
    TerrainGenerator           terrain;       // which noise composition
    std::vector<MaterialLayer> materials;     // splat rules by altitude/slope
    std::vector<ScatterLayer>  scatter;       // each: InstanceSource + density rule
    Atmosphere                 atmosphere;    // sun dir, fog color/density, sky tint
};
```

- **Meadow (first target):** ridged+fBm rolling terrain with domain-warped valley
  → light erosion; materials = grass (low slope) / rock (high slope) / dirt
  (riverbank); scatter = grass (high density, low altitude) + flowers (Poisson,
  lower density, color variants); atmosphere = warm sun, strong green ambient,
  hazy blue-grey distance fog.
- **Desert / Jungle / City** are *future recipes* — listed only to prove the
  machinery generalizes; not built now. They reuse layers 1–2 entirely.

`BiomeRegistry` maps name → factory. New biome = register a recipe. This is the
"not hardcoded, add biomes easily" requirement made concrete.

### Layer 4 — renderer (the photorealism work)

This is where the reference screenshot lives, and it is the **largest** part.
Every 4.6-only path below lives in a `Gl46*` class behind the backend seam; what
each feature buys, honestly:

1. **Terrain** — splatmapped multi-material (grass/rock/dirt blended by the
   layer-1 splat weights), triplanar on steep slopes, normal maps, distance fog
   folded in. LOD via **`Gl46TerrainLod` (tessellation + height displacement)**;
   portable fallback = CPU geomipmap chunks + frustum cull.
2. **ACES tonemapping + correct gamma** — *cheap, do early, backend-agnostic.*
   Half the "filmic" feel of the screenshot is tonemapping, not geometry.
3. **Atmosphere** — biggest mood-per-cost win, backend-agnostic:
   - Sky: upgrade the gradient (`kSkyFragmentShader`, SimRendererWidget.cpp:91)
     to analytic Rayleigh/Hosek-style scattering for the hazy horizon.
   - **Height + distance exponential fog** with sun tint — this *is* the misty
     mountain look, and it's a few shader lines.
4. **GPU flora** — grass + flower cross-quads. `Gl46FloraField` uses a **compute
   pass to cull/place from the SSBO scatter buffer + indirect instanced draw**,
   feeding millions of blades; portable fallback = CPU-culled
   `glDrawElementsInstanced`. **Wind sway** in the vertex shader (sin of
   time+world-pos), distance fade + density LOD. This is what makes it *alive*.
5. **Directional shadow map (CSM)** — large effort, large payoff, but optional;
   the scene reads well with fog + AO before shadows exist. Backend-agnostic.
6. **Polish** — SSAO, subtle bloom on flowers. Lowest priority.

---

## Wire / physics considerations

- **Collision cost.** A 256² terrain → ~130 k tris → a large general trimesh
  BVH. Option A: feed it straight into the existing `buildWorldBvh()` path (works
  today, just heavier). Option B (recommended later): add a dedicated
  **heightfield collider** to `vsim_d` — O(1) cell lookup instead of BVH
  traversal, since terrain is a function `z = h(x,y)`. Flag as an optimization,
  not a Phase-0 blocker.
- **Flora never collides.** Grass/flowers are render-only; only terrain and
  (later) trees/buildings produce colliders. Keeps the BVH small.
- **No proto bump needed** for terrain — it rides the existing
  `VSIM_CTL_SET_WORLD_MESH` channel. A heightfield collider (option B) *would*
  add a message type and bump `VSIM_PROTO_VERSION`.

## `.vworld` schema v2

Extend the existing JSON (bump `version` to 2). Procedural and imported are
mutually exclusive at the top level; obstacles still allowed alongside.

```json
{
  "format": "vayu-world",
  "version": 2,
  "procedural": {
    "biome": "meadow",
    "seed": 12345,
    "size_m": 512,
    "resolution": 256,
    "params": { "...": "biome-specific overrides, all optional" }
  },
  "obstacles": []
}
```

Loader: if `procedural` present, run the generator at load; else fall back to the
existing `world_mesh_path` import path. World tab gains: biome dropdown, seed
field (+ randomize), size/resolution, **Regenerate** button.

---

## Phasing

Ordered so **Phase 0 ships flyable procedural terrain with zero renderer
changes**, and photorealism is layered on top without rework.

- **Phase 0 — generation core + flat-shaded terrain.** Layers 1 (+ minimal 2).
  Qt-free `procgen/` lib, unit-tested. Meadow heightfield with vertex-color
  splat, rendered through the *existing* lit shader, collided through the
  *existing* BVH. Deliverable: fly over procedural rolling hills and land on
  them. **No renderer rewrite, immediate value.**
- **Phase 1 — biome framework + UI + schema + GL 4.6 bump.** `InstanceSource`
  seam, `BiomeRegistry`, `.vworld` v2, World-tab controls. Bump `main.cpp` to GL
  4.6, switch the renderer base class, stand up `GpuCaps` + the backend-seam
  interfaces (with the `Gl46*` impls empty and `Portable*` stubbed). Procedural
  primitives only. Proves "add a biome = add a recipe."
- **Phase 2 — terrain shader + tonemap + atmosphere/fog + sky.** Splatmap
  terrain, `Gl46TerrainLod` (tessellation), ACES + fog + scattering sky. The
  single biggest visual jump for the cost. After this it already *reads* like the
  screenshot's lighting, minus the grass.
- **Phase 3 — GPU flora + wind.** `Gl46FloraField` (compute cull + indirect
  instanced draw) for grass + flowers, wind sway. Now it looks alive.
- **Phase 4 — shadows (CSM) + SSAO + polish.** Closes the gap to the reference.
- **Phase 5 — imported-asset `InstanceSource` + new biomes.** Trees/buildings
  plug into the Phase-1 scatter seam unchanged → jungle, city, desert reuse
  everything. Optional: heightfield collider in `vsim_d`.

Phase 0 is the proof the architecture is sound; Phases 2–4 are the photorealism;
Phase 5 is the payoff of the procedural-now/imported-later seam.

## Open questions

1. **Texture assets.** Photoreal terrain wants albedo/normal textures for
   grass/rock/dirt. Generate procedurally, ship a small bundled set, or import?
   (Affects Phase 2 scope.)
2. **Heightfield collider vs. trimesh BVH** — accept the heavy BVH for now, or
   build the dedicated collider in Phase 0? (Perf vs. scope.)
3. **Where does generation run** — GCS-side only (current assumption, since mesh
   upload + BVH build already live there), or expose a headless generator for the
   SITL lab / SDK so tests can spawn procedural worlds?
4. **Camera/vision use later?** If a simulated camera ever consumes this render
   for vision-in-the-loop, Phase 4 (shadows/AO) stops being optional and
   determinism of flora becomes a correctness requirement, not a nicety.
```
