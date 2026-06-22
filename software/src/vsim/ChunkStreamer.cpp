#include "ChunkStreamer.h"

#include <cmath>

namespace vsim {

using procgen::ProcMesh;
using procgen::TerrainField;

void ChunkStreamer::configure(const Config& c) {
  cfg_ = c;
  field_ = std::make_unique<TerrainField>(c.field);
  loaded_.clear();
  curCx_ = curCy_ = INT_MIN;
  colCx_ = colCy_ = INT_MIN;
  first_ = true;
  active_ = true;
}

void ChunkStreamer::deactivate() {
  active_ = false;
  field_.reset();
  loaded_.clear();
  first_ = true;
}

ChunkMeshData ChunkStreamer::buildRenderChunk(int cx, int cy) const {
  ChunkMeshData out;
  out.key = keyOf(cx, cy);
  const ProcMesh m = meshFieldChunk(*field_, cx, cy, cfg_.chunkM, cfg_.resolution);
  out.positions.reserve(m.positions.size());
  out.normals.reserve(m.positions.size());
  out.colors.reserve(m.positions.size());
  for (std::size_t i = 0; i < m.positions.size(); ++i) {
    out.positions.emplace_back(m.positions[i].x, m.positions[i].y, m.positions[i].z);
    out.normals.emplace_back(m.normals[i].x, m.normals[i].y, m.normals[i].z);
    out.colors.emplace_back(m.colors[i].x, m.colors[i].y, m.colors[i].z);
  }
  return out;
}

LoadedMesh ChunkStreamer::buildCollisionMesh(int cx, int cy) const {
  // Concatenate the chunks within collisionRadius of (cx,cy) into one soup the
  // BVH builder can chew. The drone can only hit terrain near it, so this stays
  // small and is rebuilt only when the centre changes cell.
  LoadedMesh out;
  const int r = cfg_.collisionRadius;
  QVector3D lo(1e9f, 1e9f, 1e9f), hi(-1e9f, -1e9f, -1e9f);
  for (int j = cy - r; j <= cy + r; ++j) {
    for (int i = cx - r; i <= cx + r; ++i) {
      const ProcMesh m =
          meshFieldChunk(*field_, i, j, cfg_.chunkM, cfg_.resolution);
      for (std::size_t k = 0; k < m.positions.size(); ++k) {
        const procgen::PgVec3& p = m.positions[k];
        const procgen::PgVec3& n = m.normals[k];
        out.positions.emplace_back(p.x, p.y, p.z);
        out.normals.emplace_back(n.x, n.y, n.z);
        lo.setX(std::min(lo.x(), p.x)); lo.setY(std::min(lo.y(), p.y));
        lo.setZ(std::min(lo.z(), p.z));
        hi.setX(std::max(hi.x(), p.x)); hi.setY(std::max(hi.y(), p.y));
        hi.setZ(std::max(hi.z(), p.z));
      }
    }
  }
  out.bboxMin = lo;
  out.bboxMax = hi;
  out.valid = !out.positions.empty();
  return out;
}

StreamDiff ChunkStreamer::update(float wx, float wy) {
  StreamDiff diff;
  if (!active_ || !field_) return diff;

  const int cx = static_cast<int>(std::floor(wx / cfg_.chunkM));
  const int cy = static_cast<int>(std::floor(wy / cfg_.chunkM));

  // Nothing to do if we're still in the same cell and not the first call.
  if (!first_ && cx == curCx_ && cy == curCy_) return diff;

  // Desired render set: Chebyshev disc of renderRadius around (cx, cy).
  const int r = cfg_.renderRadius;
  std::set<qint64> desired;
  for (int j = cy - r; j <= cy + r; ++j)
    for (int i = cx - r; i <= cx + r; ++i)
      desired.insert(keyOf(i, j));

  // Unload chunks that drifted out of range.
  for (qint64 key : loaded_)
    if (desired.find(key) == desired.end()) diff.remove.push_back(key);
  for (qint64 key : diff.remove) loaded_.erase(key);

  // Load chunks that came into range.
  for (int j = cy - r; j <= cy + r; ++j) {
    for (int i = cx - r; i <= cx + r; ++i) {
      const qint64 key = keyOf(i, j);
      if (loaded_.insert(key).second) diff.add.push_back(buildRenderChunk(i, j));
    }
  }

  // Refresh local collision when the cell changed (or on the first update).
  if (first_ || cx != colCx_ || cy != colCy_) {
    diff.collision = buildCollisionMesh(cx, cy);
    diff.collisionChanged = diff.collision.valid;
    colCx_ = cx;
    colCy_ = cy;
  }

  curCx_ = cx;
  curCy_ = cy;
  first_ = false;
  return diff;
}

}  // namespace vsim
