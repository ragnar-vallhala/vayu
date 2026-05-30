#pragma once

#include <QString>
#include <QVector3D>
#include <vector>

// MeshLoader — thin assimp wrapper. Imports a tessellated airframe mesh
// (STL/OBJ/PLY/glTF) into a flat triangle soup the renderer can upload
// to a VBO and the mass-properties integral can integrate over. CAD
// B-rep formats (STEP/IGES) are NOT supported here -- export a mesh.
namespace vsim {

struct LoadedMesh {
  // Triangle soup: positions.size() == 3 * triangleCount(), grouped per
  // triangle. normals[] is the matching per-vertex normal (generated if
  // the file lacked them). Both already multiplied by `scale`.
  std::vector<QVector3D> positions;
  std::vector<QVector3D> normals;
  QVector3D bboxMin{0, 0, 0};
  QVector3D bboxMax{0, 0, 0};
  bool valid = false;

  int triangleCount() const { return static_cast<int>(positions.size() / 3); }
};

// Load `path` via assimp, scaling every vertex by `scale` (meters per
// mesh unit — use 0.001 for a mesh authored in millimetres). On failure
// returns {valid=false} and, if `error` is non-null, sets it to the
// assimp diagnostic.
LoadedMesh loadMesh(const QString& path, float scale, QString* error);

}  // namespace vsim
