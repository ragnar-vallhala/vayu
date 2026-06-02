#include "MeshLoader.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <algorithm>

namespace vsim {

LoadedMesh loadMesh(const QString& path, float scale, QString* error) {
  return loadMesh(path, scale, QMatrix4x4(), error);
}

LoadedMesh loadMesh(const QString& path, float scale, const QMatrix4x4& xform,
                    QString* error) {
  LoadedMesh out;

  Assimp::Importer importer;
  // Triangulate everything, bake node transforms into vertices (so a
  // multi-part assembly lands in one body frame), drop lines/points, and
  // synthesize smooth normals when the file has none.
  const aiScene* scene = importer.ReadFile(
      path.toStdString(),
      aiProcess_Triangulate | aiProcess_PreTransformVertices |
          aiProcess_JoinIdenticalVertices | aiProcess_GenSmoothNormals |
          aiProcess_FindDegenerates | aiProcess_SortByPType);

  if (!scene || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) ||
      scene->mNumMeshes == 0) {
    if (error) *error = QString::fromUtf8(importer.GetErrorString());
    if (error && error->isEmpty()) *error = QStringLiteral("no meshes in file");
    return out;
  }

  bool first = true;
  for (unsigned mi = 0; mi < scene->mNumMeshes; ++mi) {
    const aiMesh* mesh = scene->mMeshes[mi];
    if ((mesh->mPrimitiveTypes & aiPrimitiveType_TRIANGLE) == 0) continue;

    for (unsigned fi = 0; fi < mesh->mNumFaces; ++fi) {
      const aiFace& face = mesh->mFaces[fi];
      if (face.mNumIndices != 3) continue;  // post-Triangulate: should be 3
      for (unsigned k = 0; k < 3; ++k) {
        const unsigned idx = face.mIndices[k];
        const aiVector3D& v = mesh->mVertices[idx];
        const QVector3D p = xform.map(QVector3D(v.x, v.y, v.z) * scale);
        out.positions.push_back(p);

        if (mesh->HasNormals()) {
          const aiVector3D& n = mesh->mNormals[idx];
          out.normals.push_back(
              xform.mapVector(QVector3D(n.x, n.y, n.z)).normalized());
        } else {
          out.normals.emplace_back(0.0f, 0.0f, 1.0f);
        }

        if (first) {
          out.bboxMin = out.bboxMax = p;
          first = false;
        } else {
          out.bboxMin.setX(std::min(out.bboxMin.x(), p.x()));
          out.bboxMin.setY(std::min(out.bboxMin.y(), p.y()));
          out.bboxMin.setZ(std::min(out.bboxMin.z(), p.z()));
          out.bboxMax.setX(std::max(out.bboxMax.x(), p.x()));
          out.bboxMax.setY(std::max(out.bboxMax.y(), p.y()));
          out.bboxMax.setZ(std::max(out.bboxMax.z(), p.z()));
        }
      }
    }
  }

  if (out.positions.empty()) {
    if (error) *error = QStringLiteral("no triangles after import");
    return out;
  }
  out.valid = true;
  return out;
}

}  // namespace vsim
