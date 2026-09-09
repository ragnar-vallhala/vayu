// vsim_worldmesh — headless world-collision-mesh builder for the SITL harness.
//
// Reuses the GCS's OWN code (vsim::loadMesh + vsim::buildWorldBvh) so the
// collision geometry the headless harness feeds vsim_d is byte-for-byte what
// the Navigator builds from the same world mesh. It loads the world mesh,
// bakes the import frame into NED exactly as SimulatorWidget does, builds the
// serialized BVH blob, writes it to a file, and prints the counts the
// VSIM_CTL_SET_WORLD_MESH ctl frame needs:  "<nverts> <ntris> <nodes> <bytes>".
//
// Usage:
//   vsim_worldmesh <mesh> <scale> <upAxis 0=Zup|1=Yup> <offX> <offY> <offZ> \
//                  <doubleSided 0|1> <out.bin>
#include "MeshLoader.h"
#include "WorldMeshBuilder.h"

#include <QMatrix4x4>
#include <QString>
#include <QVector3D>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

int main(int argc, char **argv) {
  if (argc < 9) {
    std::fprintf(stderr,
                 "usage: %s <mesh> <scale> <upAxis 0|1> <offX> <offY> <offZ> "
                 "<doubleSided 0|1> <out.bin>\n",
                 argv[0]);
    return 2;
  }
  const QString path = QString::fromLocal8Bit(argv[1]);
  const float scale = std::atof(argv[2]);
  const int upAxis = std::atoi(argv[3]);
  const float ox = std::atof(argv[4]), oy = std::atof(argv[5]),
              oz = std::atof(argv[6]);
  const bool doubleSided = std::atoi(argv[7]) != 0;
  const char *out = argv[8];

  // Identical to SimulatorWidget::loadWorldMeshToRenderer: translate (NED
  // offset) then bake the source up-axis into NED (up = -Z). M = T*R.
  QMatrix4x4 xform;
  xform.translate(QVector3D(ox, oy, oz));
  if (upAxis == 1)
    xform.rotate(-90.0f, 1, 0, 0); // Y-up glTF
  else
    xform.rotate(180.0f, 1, 0, 0); // Z-up Blender

  QString err;
  const vsim::LoadedMesh m = vsim::loadMesh(path, scale, xform, &err);
  if (!m.valid) {
    std::fprintf(stderr, "load failed: %s\n", err.toLocal8Bit().constData());
    return 1;
  }

  const uint32_t nverts = static_cast<uint32_t>(m.positions.size());
  std::vector<float> verts;
  verts.reserve(static_cast<size_t>(nverts) * 3);
  for (const QVector3D &p : m.positions) {
    verts.push_back(p.x());
    verts.push_back(p.y());
    verts.push_back(p.z());
  }
  uint32_t nodes = 0;
  const std::vector<uint8_t> blob =
      vsim::buildWorldBvh(verts.data(), nverts, doubleSided, nodes);
  if (blob.empty()) {
    std::fprintf(stderr, "bvh build failed\n");
    return 1;
  }

  FILE *f = std::fopen(out, "wb");
  if (!f) {
    std::fprintf(stderr, "cannot write %s\n", out);
    return 1;
  }
  std::fwrite(blob.data(), 1, blob.size(), f);
  std::fclose(f);

  std::printf("%u %u %u %zu\n", nverts, nverts / 3u, nodes, blob.size());
  return 0;
}
