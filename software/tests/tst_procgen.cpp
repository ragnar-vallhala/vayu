// tst_procgen.cpp — Phase 0 procedural terrain core.
//
// The procgen module is Qt-free; this QtTest binary just drives it and checks
// the invariants the render + collision pipelines rely on: determinism,
// triangle-soup integrity, in-bounds geometry, unit upward normals, and that
// the bilinear sampler agrees with the lattice it was built from.
#include <QtTest/QtTest>

#include "procgen/Heightfield.h"
#include "procgen/Noise.h"
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

QTEST_MAIN(TstProcgen)
#include "tst_procgen.moc"
