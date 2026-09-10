#pragma once

#include <QMatrix4x4>
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
  // Matching per-vertex RGB in [0,1], baked from the source material's base
  // color (glTF/OBJ), or the mesh's vertex colors, or a neutral grey when the
  // file carries neither. Render-only — the collision BVH ignores it.
  std::vector<QVector3D> colors;
  QVector3D bboxMin{0, 0, 0};
  QVector3D bboxMax{0, 0, 0};
  bool valid = false;

  int triangleCount() const { return static_cast<int>(positions.size() / 3); }
};

// Load `path` via assimp, scaling every vertex by `scale` (meters per
// mesh unit — use 0.001 for a mesh authored in millimetres). On failure
// returns {valid=false} and, if `error` is non-null, sets it to the
// assimp diagnostic.
LoadedMesh loadMesh(const QString &path, float scale, QString *error);

// As above, but additionally applies `xform` to every vertex (and its
// rotation to normals) after scaling — used to bake an importer's frame into
// the sim's NED world frame (e.g. Blender Z-up / glTF Y-up -> NED). Pass an
// identity matrix for no transform.
LoadedMesh loadMesh(const QString &path, float scale, const QMatrix4x4 &xform,
                    QString *error);

} // namespace vsim
