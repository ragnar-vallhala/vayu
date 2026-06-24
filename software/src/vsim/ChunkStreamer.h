// ChunkStreamer.h — bookkeeping for endless procedural terrain streaming.
//
// Decides which terrain chunks should be loaded around a moving centre (the
// drone or the free-fly camera) and which to drop. It does NOT mesh chunks
// itself: meshing is the expensive part (~15 ms/chunk) and runs OFF the UI
// thread (SimulatorWidget farms meshFieldChunk out via QtConcurrent), then the
// results come back and are applied on the main thread. The streamer just
// tracks loaded / in-flight / desired sets and hands out a plan.
//
// Collision reuses the meshes already built for rendering (cached by the
// caller), so crossing a chunk boundary no longer re-generates terrain.
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

// A chunk that needs meshing, identified by both its key and cell coords.
struct ChunkReq {
  qint64 key = 0;
  int cx = 0;
  int cy = 0;
};

// What to do for the current centre: build these chunks (off-thread), drop
// those, and — when the centre crossed into a new cell — rebuild local
// collision around (colCx, colCy).
struct StreamPlan {
  std::vector<ChunkReq> toBuild;
  std::vector<qint64> toRemove;
  bool collisionDue = false;
  int colCx = 0;
  int colCy = 0;
};

class ChunkStreamer {
 public:
  struct Config {
    procgen::FieldParams field;
    float chunkM = 192.0f;       // world metres per chunk side
    int resolution = 100;        // visual tessellation (grid cells per side)
    int collisionResolution = 48;  // coarse mesh for the collision BVH (cheap)
    int renderRadius = 3;        // Chebyshev chunk radius kept loaded (visual)
    int collisionRadius = 1;     // chunk radius shipped to the daemon as a BVH
    int maxInFlight = 6;       // cap on concurrent off-thread chunk builds
  };

  void configure(const Config& c);
  void deactivate();
  bool active() const { return active_; }
  const Config& config() const { return cfg_; }

  // The live infinite field (null when inactive). Shared so off-thread builds
  // and the minimap can sample it concurrently (read-only -> safe), and an
  // in-flight build keeps the old field alive across a reconfigure.
  std::shared_ptr<const procgen::TerrainField> field() const { return field_; }

  // Plan the work for centre world-XY (wx, wy). Chunks returned in toBuild are
  // marked in-flight (won't be re-requested); call markBuilt or forget when
  // each resolves. toBuild is nearest-first and capped to the in-flight budget.
  StreamPlan plan(float wx, float wy);

  bool wanted(qint64 key) const { return desired_.count(key) > 0; }
  void markBuilt(qint64 key);   // a build resolved -> loaded
  void forget(qint64 key);      // dropped from the renderer -> untrack

  // The chunk cells the current collision neighbourhood needs (so the caller
  // can check its mesh cache has them all before building the BVH).
  std::vector<ChunkReq> collisionChunks(int cx, int cy) const;

  static qint64 keyOf(int cx, int cy) {
    return (static_cast<qint64>(cx) << 32) |
           static_cast<qint64>(static_cast<uint32_t>(cy));
  }
  static int cxOf(qint64 k) { return static_cast<int>(k >> 32); }
  static int cyOf(qint64 k) { return static_cast<int>(static_cast<uint32_t>(k)); }

 private:
  Config cfg_;
  std::shared_ptr<const procgen::TerrainField> field_;
  std::set<qint64> loaded_;    // uploaded to the renderer
  std::set<qint64> inflight_;  // meshing off-thread
  std::set<qint64> desired_;   // should be loaded for the current cell
  int curCx_ = INT_MIN, curCy_ = INT_MIN;
  int colCx_ = INT_MIN, colCy_ = INT_MIN;
  bool active_ = false;
  bool first_ = true;
};

// Convert a meshed chunk (procgen soup) into renderer form (QVector3D arrays).
ChunkMeshData toChunkMeshData(qint64 key, const procgen::ProcMesh& m);

// Concatenate a set of chunk meshes into one collision LoadedMesh (NED world
// frame, positions + normals). Used to feed buildWorldBvh from cached chunks.
LoadedMesh collisionMeshFromChunks(
    const std::vector<const procgen::ProcMesh*>& chunks);

}  // namespace vsim
