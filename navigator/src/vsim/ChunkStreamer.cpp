#include "ChunkStreamer.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace vsim {

using procgen::ProcMesh;
using procgen::TerrainField;

void ChunkStreamer::configure(const Config &c) {
  cfg_ = c;
  field_ = std::make_shared<const TerrainField>(c.field);
  loaded_.clear();
  inflight_.clear();
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
  inflight_.clear();
  desired_.clear();
  first_ = true;
}

void ChunkStreamer::markBuilt(qint64 key) {
  inflight_.erase(key);
  loaded_.insert(key);
}

void ChunkStreamer::forget(qint64 key) {
  inflight_.erase(key);
  loaded_.erase(key);
}

std::vector<ChunkReq> ChunkStreamer::collisionChunks(int cx, int cy) const {
  std::vector<ChunkReq> out;
  const int r = cfg_.collisionRadius;
  for (int j = cy - r; j <= cy + r; ++j)
    for (int i = cx - r; i <= cx + r; ++i)
      out.push_back(ChunkReq{keyOf(i, j), i, j});
  return out;
}

StreamPlan ChunkStreamer::plan(float wx, float wy) {
  StreamPlan p;
  if (!active_ || !field_)
    return p;

  const int cx = static_cast<int>(std::floor(wx / cfg_.chunkM));
  const int cy = static_cast<int>(std::floor(wy / cfg_.chunkM));
  const bool cellChanged = first_ || cx != curCx_ || cy != curCy_;

  if (cellChanged) {
    const int r = cfg_.renderRadius;
    desired_.clear();
    for (int j = cy - r; j <= cy + r; ++j)
      for (int i = cx - r; i <= cx + r; ++i)
        desired_.insert(keyOf(i, j));

    // Drop loaded chunks that left the disc. (In-flight chunks that left are
    // handled on arrival: markBuilt then a later plan removes them.)
    for (qint64 key : loaded_)
      if (desired_.find(key) == desired_.end())
        p.toRemove.push_back(key);
    for (qint64 key : p.toRemove)
      loaded_.erase(key);

    if (cx != colCx_ || cy != colCy_) {
      p.collisionDue = true;
      p.colCx = cx;
      p.colCy = cy;
      colCx_ = cx;
      colCy_ = cy;
    }
    curCx_ = cx;
    curCy_ = cy;
    first_ = false;
  }

  // Nearest-first list of chunks neither loaded nor already building.
  std::vector<qint64> missing;
  for (qint64 key : desired_)
    if (loaded_.find(key) == loaded_.end() &&
        inflight_.find(key) == inflight_.end())
      missing.push_back(key);
  if (!missing.empty()) {
    auto cheb = [cx, cy](qint64 key) {
      return std::max(std::abs(cxOf(key) - cx), std::abs(cyOf(key) - cy));
    };
    std::sort(missing.begin(), missing.end(),
              [&](qint64 a, qint64 b) { return cheb(a) < cheb(b); });
    int budget = cfg_.maxInFlight - static_cast<int>(inflight_.size());
    for (qint64 key : missing) {
      if (budget <= 0)
        break;
      p.toBuild.push_back(ChunkReq{key, cxOf(key), cyOf(key)});
      inflight_.insert(key);
      --budget;
    }
  }
  return p;
}

ChunkMeshData toChunkMeshData(qint64 key, const ProcMesh &m) {
  ChunkMeshData out;
  out.key = key;
  const std::size_t n = m.positions.size();
  out.positions.reserve(n);
  out.normals.reserve(n);
  out.colors.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    out.positions.emplace_back(m.positions[i].x, m.positions[i].y,
                               m.positions[i].z);
    out.normals.emplace_back(m.normals[i].x, m.normals[i].y, m.normals[i].z);
    out.colors.emplace_back(m.colors[i].x, m.colors[i].y, m.colors[i].z);
  }
  return out;
}

LoadedMesh
collisionMeshFromChunks(const std::vector<const ProcMesh *> &chunks) {
  LoadedMesh out;
  QVector3D lo(1e9f, 1e9f, 1e9f), hi(-1e9f, -1e9f, -1e9f);
  for (const ProcMesh *m : chunks) {
    if (!m)
      continue;
    for (std::size_t k = 0; k < m->positions.size(); ++k) {
      const procgen::PgVec3 &p = m->positions[k];
      const procgen::PgVec3 &nrm = m->normals[k];
      out.positions.emplace_back(p.x, p.y, p.z);
      out.normals.emplace_back(nrm.x, nrm.y, nrm.z);
      lo.setX(std::min(lo.x(), p.x));
      lo.setY(std::min(lo.y(), p.y));
      lo.setZ(std::min(lo.z(), p.z));
      hi.setX(std::max(hi.x(), p.x));
      hi.setY(std::max(hi.y(), p.y));
      hi.setZ(std::max(hi.z(), p.z));
    }
  }
  out.bboxMin = lo;
  out.bboxMax = hi;
  out.valid = !out.positions.empty();
  return out;
}

} // namespace vsim
