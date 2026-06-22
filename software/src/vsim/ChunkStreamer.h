// ChunkStreamer.h — pages endless procedural terrain around a moving centre.
//
// Holds an infinite TerrainField and a set of currently-loaded chunk cells.
// Each update() recomputes which chunks should be loaded for the current centre
// (the drone, or the free-fly camera) and returns a diff: meshes to upload,
// chunk keys to drop, and — when the centre crosses into a new cell — a freshly
// built local collision mesh for the daemon. The streamer is render/sim-agnostic
// (it returns data); SimulatorWidget applies the diff to the renderer(s) + sim.
#pragma once

#include "MeshLoader.h"  // vsim::LoadedMesh
#include "procgen/TerrainField.h"

#include <QVector3D>

#include <climits>
#include <cstdint>
#include <memory>
#include <set>
#include <vector>

namespace vsim {

// One chunk's renderable geometry (NED world frame), keyed like the renderer's
// chunk map.
struct ChunkMeshData {
  qint64 key = 0;
  std::vector<QVector3D> positions;
  std::vector<QVector3D> normals;
  std::vector<QVector3D> colors;
};

// What to apply after an update(). `add`/`remove` drive the renderer; when
// `collisionChanged` is set, `collision` is the new local terrain BVH source.
struct StreamDiff {
  std::vector<ChunkMeshData> add;
  std::vector<qint64> remove;
  bool collisionChanged = false;
  LoadedMesh collision;
};

class ChunkStreamer {
 public:
  struct Config {
    procgen::FieldParams field;
    float chunkM = 160.0f;     // world metres per chunk side
    int resolution = 64;       // grid cells per chunk side
    int renderRadius = 4;      // Chebyshev chunk radius kept loaded (visual)
    int collisionRadius = 1;   // chunk radius shipped to the daemon as a BVH
  };

  // (Re)configure: rebuilds the field and forgets the loaded set, so the next
  // update() re-streams from scratch. Marks the streamer active.
  void configure(const Config& c);
  // Turn streaming off and forget state (e.g. switching away from an endless
  // biome). active() is false afterwards.
  void deactivate();
  bool active() const { return active_; }

  const Config& config() const { return cfg_; }

  // Bring the loaded set in line with world-XY centre (wx, wy) and return the
  // diff. Cheap when the centre stays within its current cell (no new geometry).
  StreamDiff update(float wx, float wy);

  // Pack/unpack chunk cell <-> key (must match the renderer's opaque key use).
  static qint64 keyOf(int cx, int cy) {
    return (static_cast<qint64>(cx) << 32) |
           static_cast<qint64>(static_cast<uint32_t>(cy));
  }

 private:
  ChunkMeshData buildRenderChunk(int cx, int cy) const;
  LoadedMesh buildCollisionMesh(int cx, int cy) const;

  Config cfg_;
  std::unique_ptr<procgen::TerrainField> field_;
  std::set<qint64> loaded_;
  int curCx_ = INT_MIN, curCy_ = INT_MIN;  // centre's current chunk cell
  int colCx_ = INT_MIN, colCy_ = INT_MIN;  // cell the shipped collision covers
  bool active_ = false;
  bool first_ = true;
};

}  // namespace vsim
