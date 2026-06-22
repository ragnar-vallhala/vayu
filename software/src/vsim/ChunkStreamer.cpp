#include "ChunkStreamer.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace vsim {

using procgen::ProcMesh;
using procgen::TerrainField;

void ChunkStreamer::configure(const Config& c) {
  cfg_ = c;
  field_ = std::make_unique<TerrainField>(c.field);
  loaded_.clear();
  desired_.clear();
  curCx_ = curCy_ = INT_MIN;
  colCx_ = colCy_ = INT_MIN;
  first_ = true;
  active_ = true;
}

void ChunkStreamer::deactivate() {
  active_ = false;
  field_.reset();
  loaded_.clear();
  desired_.clear();
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
  const bool cellChanged = first_ || cx != curCx_ || cy != curCy_;

  if (cellChanged) {
    // Recompute the desired set (Chebyshev disc of renderRadius) and unload any
    // chunks that drifted out of range. Removal is cheap (no meshing).
    const int r = cfg_.renderRadius;
    desired_.clear();
    for (int j = cy - r; j <= cy + r; ++j)
      for (int i = cx - r; i <= cx + r; ++i)
        desired_.insert(keyOf(i, j));
    for (qint64 key : loaded_)
      if (desired_.find(key) == desired_.end()) diff.remove.push_back(key);
    for (qint64 key : diff.remove) loaded_.erase(key);

    // Local collision follows the cell. Built synchronously (small: a
    // collisionRadius neighbourhood), reshipped only on a cell change.
    if (cx != colCx_ || cy != colCy_) {
      diff.collision = buildCollisionMesh(cx, cy);
      diff.collisionChanged = diff.collision.valid;
      colCx_ = cx;
      colCy_ = cy;
    }
    curCx_ = cx;
    curCy_ = cy;
    first_ = false;
  }

  // Amortised load: mesh at most maxBuildsPerUpdate of the still-missing chunks
  // per call, nearest-first, so a full neighbourhood streams in over several
  // ticks instead of freezing the UI thread in one burst.
  std::vector<qint64> missing;
  for (qint64 key : desired_)
    if (loaded_.find(key) == loaded_.end()) missing.push_back(key);
  if (!missing.empty()) {
    auto cheb = [cx, cy](qint64 key) {
      const int ix = static_cast<int>(key >> 32);
      const int iy = static_cast<int>(static_cast<uint32_t>(key));
      return std::max(std::abs(ix - cx), std::abs(iy - cy));
    };
    std::sort(missing.begin(), missing.end(),
              [&](qint64 a, qint64 b) { return cheb(a) < cheb(b); });
    const int budget = cfg_.maxBuildsPerUpdate > 0 ? cfg_.maxBuildsPerUpdate : 1;
    const int n = std::min<int>(budget, static_cast<int>(missing.size()));
    for (int k = 0; k < n; ++k) {
      const qint64 key = missing[k];
      const int ix = static_cast<int>(key >> 32);
      const int iy = static_cast<int>(static_cast<uint32_t>(key));
      diff.add.push_back(buildRenderChunk(ix, iy));
      loaded_.insert(key);
    }
  }
  return diff;
}

}  // namespace vsim
