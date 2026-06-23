// GpuGrass.h — GPU-driven grass (Ghost-of-Tsushima style).
//
// A compute shader regenerates grass blade instances around the camera EVERY
// frame: it evaluates the terrain (the procgen noise ported to GLSL), applies
// the same density (slope/altitude) + a distance falloff + frustum culling,
// and appends surviving blades to an SSBO via an atomic counter. An indirect
// draw then renders exactly that many instances — the CPU never touches a blade,
// so there is no scatter cost, no instance storage, and no upload hitches.
//
// Requires GL 4.3+ (compute + SSBO + indirect). init() returns false otherwise
// and the renderer falls back to the CPU flora path.
#pragma once

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QVector3D>

#include <cstdint>

class QOpenGLExtraFunctions;

namespace vsim {

class GpuGrass {
 public:
  struct Params {
    // Terrain field — MUST mirror procgen::TerrainField so blades sit on the
    // same surface the mesh is built from.
    uint32_t seed = 1337u;
    float heightM = 70.0f, featureM = 220.0f, macroM = 1400.0f;
    int octaves = 6;
    float lacunarity = 2.0f, gain = 0.5f, mountainMix = 0.55f;
    // Grass placement (mirrors procgen::FloraParams).
    float grassMaxFrac = 0.5f, slopeLo = 0.80f, slopeHi = 0.93f;
    float heightMean = 1.0f, heightStdDev = 0.3f, flowerFrac = 0.02f;
    // Generation grid around the camera.
    float cell = 0.16f;      // candidate spacing [m] (smaller = denser)
    int grid = 768;          // candidates per side
    float falloffStart = 55.0f, falloffEnd = 90.0f;  // distance density fade [m]
  };

  ~GpuGrass();
  bool init(QOpenGLExtraFunctions* gl);   // false if compute unsupported
  bool ready() const { return ready_; }
  void setParams(const Params& p);

  // Regenerate blades around camPos and draw them. proj/view are the render
  // matrices; sunDir/time feed lighting + wind.
  void render(QOpenGLExtraFunctions* gl, const QMatrix4x4& proj,
              const QMatrix4x4& view, const QVector3D& camPos,
              const QVector3D& sunDir, float time);

 public:
  // Concentric density rings (dense near .. coarse far) each get their own
  // instance buffer + indirect draw + a lower-LOD blade mesh (fewer segments
  // with distance), so far blades cost a fraction of the near ones.
  static constexpr int kRings = 4;

 private:
  void buildBlade(QOpenGLExtraFunctions* gl, int ring, int segments);

  bool ready_ = false;
  Params params_;
  QOpenGLShaderProgram fill_;   // fill the height texture (noise once per texel)
  QOpenGLShaderProgram comp_;   // generation (samples the height texture)
  QOpenGLShaderProgram draw_;   // render
  unsigned int heightTex_ = 0;  // R32F terrain-height image around the camera
  int texSize_ = 512;
  unsigned int ssbo_[kRings] = {0};      // per-ring blade instances
  unsigned int indirect_[kRings] = {0};  // per-ring DrawArraysIndirectCommand
  unsigned int counter_[kRings] = {0};   // per-ring atomic counter (-> indirect)
  QOpenGLBuffer bladeVbo_[kRings];       // per-ring LOD blade geometry
  QOpenGLVertexArrayObject vao_[kRings];
  int bladeVerts_[kRings] = {0};
  int maxBladesPerRing_ = 0;
};

}  // namespace vsim
