// tst_procgen.cpp — Phase 0 procedural terrain core.
//
// The procgen module is Qt-free; this QtTest binary just drives it and checks
// the invariants the render + collision pipelines rely on: determinism,
// triangle-soup integrity, in-bounds geometry, unit upward normals, and that
// the bilinear sampler agrees with the lattice it was built from.
#include <QtTest/QtTest>

#include "ChunkStreamer.h"
#include "procgen/Heightfield.h"
#include "procgen/Noise.h"
#include "procgen/TerrainField.h"
#include "procgen/TerrainGen.h"

#include <cmath>

using namespace vsim::procgen;

namespace {
TerrainParams smallParams(uint32_t seed = 1337u) {
  TerrainParams p;
  p.seed = seed;
  p.sizeM = 128.0f;
  p.resolution = 48;  // keep the test fast; invariants are resolution-agnostic
  return p;
}
}  // namespace

class TstProcgen : public QObject {
  Q_OBJECT
 private slots:
  void noiseIsDeterministicAndBounded();
  void fbmStaysInRange();
  void heightfieldHeightsBounded();
  void sampleWorldMatchesLatticeNodes();
  void meshSoupIntegrity();
  void meshGeometryInBounds();
  void normalsAreUnitAndUpward();
  void sameSeedSameMesh();
  void differentSeedDiffersMesh();
  void fieldIsDeterministicAndBounded();
  void fieldChunksTileSeamlessly();
  void fieldChunkSoupIntegrity();
  void streamerLoadsNeighborhood();
  void streamerStableWithinCell();
  void streamerPagesOnCrossing();
};

void TstProcgen::noiseIsDeterministicAndBounded() {
  Noise a(42u), b(42u), c(43u);
  bool differs = false;
  for (int i = 0; i < 200; ++i) {
    const float x = i * 0.37f, y = i * 0.19f;
    const float va = a.gradient(x, y);
    QCOMPARE(va, b.gradient(x, y));          // same seed -> identical
    QVERIFY(va >= -1.5f && va <= 1.5f);      // ~[-1,1] with margin
    if (std::fabs(va - c.gradient(x, y)) > 1e-6f) differs = true;
  }
  QVERIFY(differs);                          // a different seed decorrelates
}

void TstProcgen::fbmStaysInRange() {
  Noise n(7u);
  for (int i = 0; i < 500; ++i) {
    const float x = i * 0.123f, y = i * 0.071f;
    const float f = n.fbm(x, y, 6, 2.0f, 0.5f);
    QVERIFY(f >= -1.01f && f <= 1.01f);
    const float r = n.ridged(x, y, 6, 2.0f, 0.5f);
    QVERIFY(r >= -0.01f && r <= 1.01f);
  }
}

void TstProcgen::heightfieldHeightsBounded() {
  const TerrainParams p = smallParams();
  const Heightfield hf = generateHeightfield(p);
  QVERIFY(hf.valid());
  QCOMPARE(hf.n, p.resolution);
  float lo = 1e9f, hi = -1e9f;
  for (float v : hf.h) { lo = std::min(lo, v); hi = std::max(hi, v); }
  QVERIFY(lo >= -1e-4f);                 // never below the ground plane
  QVERIFY(hi <= p.heightM + 1e-3f);      // never above the peak cap
  QVERIFY(hi > p.heightM * 0.2f);        // and it actually rises (not flat)
}

void TstProcgen::sampleWorldMatchesLatticeNodes() {
  const Heightfield hf = generateHeightfield(smallParams());
  // Bilinear sampling exactly at a node must return that node's height.
  for (int iy = 0; iy < hf.n; iy += 7) {
    for (int ix = 0; ix < hf.n; ix += 7) {
      const PgVec3 w = hf.worldAt(ix, iy);
      QVERIFY(std::fabs(hf.sampleWorld(w.x, w.y) - hf.at(ix, iy)) < 1e-2f);
    }
  }
}

void TstProcgen::meshSoupIntegrity() {
  const TerrainParams p = smallParams();
  const ProcMesh m = generateMeadow(p);
  QCOMPARE(m.positions.size() % 3, std::size_t(0));
  QCOMPARE(m.normals.size(), m.positions.size());
  QCOMPARE(m.colors.size(), m.positions.size());
  // Two triangles per grid cell, (n-1)^2 cells.
  const std::size_t cells =
      static_cast<std::size_t>(p.resolution - 1) * (p.resolution - 1);
  QCOMPARE(m.triangleCount(), cells * 2);
}

void TstProcgen::meshGeometryInBounds() {
  const TerrainParams p = smallParams();
  const ProcMesh m = generateMeadow(p);
  const float half = p.sizeM * 0.5f + 1e-3f;
  for (const PgVec3& v : m.positions) {
    QVERIFY(v.x >= -half && v.x <= half);
    QVERIFY(v.y >= -half && v.y <= half);
    // Terrain is above the ground plane: z = -height <= 0.
    QVERIFY(v.z <= 1e-3f && v.z >= -(p.heightM + 1e-2f));
  }
  // Colors are valid RGB in [0,1].
  for (const PgVec3& c : m.colors) {
    QVERIFY(c.x >= 0.0f && c.x <= 1.0f);
    QVERIFY(c.y >= 0.0f && c.y <= 1.0f);
    QVERIFY(c.z >= 0.0f && c.z <= 1.0f);
  }
}

void TstProcgen::normalsAreUnitAndUpward() {
  const ProcMesh m = generateMeadow(smallParams());
  for (const PgVec3& nrm : m.normals) {
    const float len = std::sqrt(nrm.x * nrm.x + nrm.y * nrm.y + nrm.z * nrm.z);
    QVERIFY(std::fabs(len - 1.0f) < 1e-3f);
    // In NED, up is -Z; every terrain normal must point at least somewhat up.
    QVERIFY(nrm.z < 0.0f);
  }
}

void TstProcgen::sameSeedSameMesh() {
  const ProcMesh a = generateMeadow(smallParams(2024u));
  const ProcMesh b = generateMeadow(smallParams(2024u));
  QCOMPARE(a.positions.size(), b.positions.size());
  for (std::size_t i = 0; i < a.positions.size(); ++i) {
    QCOMPARE(a.positions[i].x, b.positions[i].x);
    QCOMPARE(a.positions[i].y, b.positions[i].y);
    QCOMPARE(a.positions[i].z, b.positions[i].z);
  }
}

void TstProcgen::differentSeedDiffersMesh() {
  const ProcMesh a = generateMeadow(smallParams(1u));
  const ProcMesh b = generateMeadow(smallParams(2u));
  QCOMPARE(a.positions.size(), b.positions.size());
  std::size_t differing = 0;
  for (std::size_t i = 0; i < a.positions.size(); ++i)
    if (a.positions[i].z != b.positions[i].z) ++differing;
  QVERIFY(differing > a.positions.size() / 10);  // fields are genuinely distinct
}

void TstProcgen::fieldIsDeterministicAndBounded() {
  FieldParams fp;
  fp.seed = 99u;
  TerrainField a(fp), b(fp);
  for (int i = 0; i < 300; ++i) {
    const float wx = i * 13.7f - 800.0f, wy = i * 7.1f - 400.0f;
    const float h = a.height(wx, wy);
    QCOMPARE(h, b.height(wx, wy));                 // deterministic
    QVERIFY(h >= -1e-4f && h <= fp.heightM + 1e-3f);  // bounded, never negative
  }
}

void TstProcgen::fieldChunksTileSeamlessly() {
  // Two horizontally-adjacent chunks must agree exactly along their shared
  // edge: same positions, normals, AND colors. This is what makes streaming
  // seamless.
  FieldParams fp;
  fp.seed = 5u;
  TerrainField f(fp);
  const float chunkM = 128.0f;
  const int res = 32;
  const ProcMesh left = meshFieldChunk(f, 0, 0, chunkM, res);
  const ProcMesh right = meshFieldChunk(f, 1, 0, chunkM, res);
  QVERIFY(!left.empty() && !right.empty());

  // Collect the vertices on the boundary x == chunkM from both chunks, keyed by
  // their y coordinate; every left-edge sample of `right` must coincide with a
  // right-edge sample of `left`.
  auto matchAt = [](const ProcMesh& m, float xWanted, float y) -> int {
    int found = -1;
    for (std::size_t i = 0; i < m.positions.size(); ++i) {
      if (std::fabs(m.positions[i].x - xWanted) < 1e-3f &&
          std::fabs(m.positions[i].y - y) < 1e-3f) { found = (int)i; break; }
    }
    return found;
  };
  int checked = 0;
  for (std::size_t i = 0; i < right.positions.size(); ++i) {
    if (std::fabs(right.positions[i].x - chunkM) > 1e-3f) continue;  // left edge
    const int li = matchAt(left, chunkM, right.positions[i].y);
    QVERIFY(li >= 0);
    QCOMPARE(left.positions[li].z, right.positions[i].z);
    QCOMPARE(left.normals[li].x, right.normals[i].x);
    QCOMPARE(left.normals[li].y, right.normals[i].y);
    QCOMPARE(left.normals[li].z, right.normals[i].z);
    QCOMPARE(left.colors[li].x, right.colors[i].x);
    if (++checked > 40) break;
  }
  QVERIFY(checked > 5);
}

void TstProcgen::fieldChunkSoupIntegrity() {
  FieldParams fp;
  TerrainField f(fp);
  const int res = 24;
  const ProcMesh m = meshFieldChunk(f, -3, 2, 160.0f, res);
  QCOMPARE(m.positions.size() % 3, std::size_t(0));
  QCOMPARE(m.normals.size(), m.positions.size());
  QCOMPARE(m.colors.size(), m.positions.size());
  QCOMPARE(m.triangleCount(),
           static_cast<std::size_t>(res) * res * 2);
  // Chunk (-3,2) occupies world x in [-480,-320], y in [320,480].
  for (const PgVec3& v : m.positions) {
    QVERIFY(v.x >= -480.1f && v.x <= -319.9f);
    QVERIFY(v.y >= 319.9f && v.y <= 480.1f);
    QVERIFY(v.z <= 1e-3f);  // terrain above ground plane
  }
}

static vsim::ChunkStreamer::Config streamCfg(int renderRadius) {
  vsim::ChunkStreamer::Config c;
  c.field.seed = 11u;
  c.chunkM = 100.0f;
  c.resolution = 8;        // tiny chunks -> fast test
  c.renderRadius = renderRadius;
  c.collisionRadius = 1;
  return c;
}

void TstProcgen::streamerLoadsNeighborhood() {
  vsim::ChunkStreamer s;
  s.configure(streamCfg(2));
  QVERIFY(s.active());
  const vsim::StreamDiff d = s.update(0.0f, 0.0f);
  // Chebyshev radius 2 -> a 5x5 window loads on the first update.
  QCOMPARE(d.add.size(), std::size_t(25));
  QVERIFY(d.remove.empty());
  QVERIFY(d.collisionChanged);
  QVERIFY(d.collision.valid);
  QVERIFY(!d.collision.positions.empty());
}

void TstProcgen::streamerStableWithinCell() {
  vsim::ChunkStreamer s;
  s.configure(streamCfg(2));
  s.update(0.0f, 0.0f);
  // Move within the same chunk cell (cell is 100 m): no new geometry, no
  // collision rebuild.
  const vsim::StreamDiff d = s.update(40.0f, 25.0f);
  QVERIFY(d.add.empty());
  QVERIFY(d.remove.empty());
  QVERIFY(!d.collisionChanged);
}

void TstProcgen::streamerPagesOnCrossing() {
  const int r = 2;
  vsim::ChunkStreamer s;
  s.configure(streamCfg(r));
  s.update(0.0f, 0.0f);
  // Cross one cell east (x: 0 -> cell 1). One column leaves, one enters; the
  // loaded-set size is conserved, so adds == removes == (2r+1).
  const vsim::StreamDiff d = s.update(150.0f, 0.0f);
  QCOMPARE(d.add.size(), static_cast<std::size_t>(2 * r + 1));
  QCOMPARE(d.remove.size(), static_cast<std::size_t>(2 * r + 1));
  QVERIFY(d.collisionChanged);  // centre changed cell -> collision reshipped
}

QTEST_MAIN(TstProcgen)
#include "tst_procgen.moc"
