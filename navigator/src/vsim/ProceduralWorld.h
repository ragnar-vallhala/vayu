// ProceduralWorld.h — Qt adapter from the Qt-free procgen core to LoadedMesh.
//
// The procgen/ module produces a plain-struct triangle soup. This thin bridge
// converts it into a vsim::LoadedMesh (QVector3D), so a procedural world flows
// through the *existing* render (setWorldMesh) and collision (buildWorldBvh)
// pipelines with no special-casing downstream. Keeping the conversion here is
// what lets the core stay Qt-free and unit-tested.
#pragma once

#include "MeshLoader.h"            // vsim::LoadedMesh
#include "VsimTypes.h"             // vsim::WorldConfig
#include "procgen/Heightfield.h"   // procgen::Heightfield
#include "procgen/TerrainGen.h"    // procgen::TerrainParams

namespace vsim {

// True if `biome` names a generator this build knows how to produce.
bool isKnownBiome(const QString& biome);

// Map the procedural fields of `w` to the finite meadow generator's params.
procgen::TerrainParams meadowParams(const WorldConfig& w);

// The finite meadow's elevation grid — lets the minimap sample terrain height
// without retaining the (much larger) render mesh.
procgen::Heightfield proceduralMeadowHeightfield(const WorldConfig& w);

// Generate a procedural world from the procedural fields of `w`, in NED world
// metres (terrain up is -Z, ground plane at z=0). Returns {valid=false} if the
// biome is empty/unknown. The result drops straight into setWorldMesh() and
// sendWorldMeshToSim() like an imported mesh.
LoadedMesh generateProceduralWorld(const WorldConfig& w);

}  // namespace vsim
