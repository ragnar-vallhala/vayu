#include "ProceduralWorld.h"

#include "procgen/TerrainGen.h"

#include <algorithm>

namespace vsim {

bool isKnownBiome(const QString& biome) {
  return biome.compare(QStringLiteral("meadow"), Qt::CaseInsensitive) == 0;
}

procgen::TerrainParams meadowParams(const WorldConfig& w) {
  procgen::TerrainParams p;
  p.seed = static_cast<uint32_t>(w.proceduralSeed);
  p.sizeM = w.proceduralSizeM > 1.0f ? w.proceduralSizeM : 256.0f;
  p.resolution = std::clamp(w.proceduralResolution, 2, 1024);
  return p;
}

procgen::Heightfield proceduralMeadowHeightfield(const WorldConfig& w) {
  return procgen::generateHeightfield(meadowParams(w));
}

LoadedMesh generateProceduralWorld(const WorldConfig& w) {
  LoadedMesh out;
  if (w.proceduralBiome.isEmpty() || !isKnownBiome(w.proceduralBiome))
    return out;  // valid == false

  const procgen::TerrainParams p = meadowParams(w);
  const procgen::ProcMesh m = procgen::generateMeadow(p);
  if (m.empty()) return out;

  const std::size_t nverts = m.positions.size();
  out.positions.reserve(nverts);
  out.normals.reserve(nverts);
  out.colors.reserve(nverts);

  QVector3D lo(1e9f, 1e9f, 1e9f), hi(-1e9f, -1e9f, -1e9f);
  for (std::size_t i = 0; i < nverts; ++i) {
    const procgen::PgVec3& v = m.positions[i];
    const procgen::PgVec3& nrm = m.normals[i];
    const procgen::PgVec3& c = m.colors[i];
    out.positions.emplace_back(v.x, v.y, v.z);
    out.normals.emplace_back(nrm.x, nrm.y, nrm.z);
    out.colors.emplace_back(c.x, c.y, c.z);
    lo.setX(std::min(lo.x(), v.x)); lo.setY(std::min(lo.y(), v.y));
    lo.setZ(std::min(lo.z(), v.z));
    hi.setX(std::max(hi.x(), v.x)); hi.setY(std::max(hi.y(), v.y));
    hi.setZ(std::max(hi.z(), v.z));
  }
  out.bboxMin = lo;
  out.bboxMax = hi;
  out.valid = true;
  return out;
}

}  // namespace vsim
