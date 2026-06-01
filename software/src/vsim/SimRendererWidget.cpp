#include "SimRendererWidget.h"

#include <QCursor>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QtMath>

#include <array>
#include <cmath>
#include <vector>

namespace vsim {

// All geometry lives in NED. The camera "up" is (0,0,-1) so the
// rendered "up" on screen is NED -Z = above ground. World axes look
// like: red=N (+X), green=E (+Y), blue=down (+Z).
namespace {

constexpr int kGridHalf   = 10;       // m, ground plane extends +/- this
constexpr float kGridStep = 1.0f;
constexpr float kBodyL    = 0.40f;    // body extents in X
constexpr float kBodyW    = 0.30f;    // body extents in Y
constexpr float kBodyH    = 0.06f;    // body extents in Z
constexpr float kRotorR   = 0.08f;
constexpr int   kRotorSeg = 24;

const char* kVertexShader = R"GLSL(
#version 330 core
layout(location=0) in vec3 a_pos;
uniform mat4 u_mvp;
void main() {
  gl_Position = u_mvp * vec4(a_pos, 1.0);
}
)GLSL";

const char* kFragmentShader = R"GLSL(
#version 330 core
out vec4 o_color;
uniform vec3 u_color;
void main() {
  o_color = vec4(u_color, 1.0);
}
)GLSL";

// Lit program for the imported airframe mesh: world-space directional
// Lambert + ambient so the solid reads as 3D rather than a flat blob.
const char* kLitVertexShader = R"GLSL(
#version 330 core
layout(location=0) in vec3 a_pos;
layout(location=1) in vec3 a_normal;
uniform mat4 u_mvp;
uniform mat3 u_nmat;
out vec3 v_normal;
void main() {
  gl_Position = u_mvp * vec4(a_pos, 1.0);
  v_normal = u_nmat * a_normal;
}
)GLSL";

const char* kLitFragmentShader = R"GLSL(
#version 330 core
in vec3 v_normal;
out vec4 o_color;
uniform vec3 u_color;
uniform vec3 u_lightdir;   // world-space direction toward the light
void main() {
  vec3 n = normalize(v_normal);
  float ndl = max(dot(n, normalize(u_lightdir)), 0.0);
  float ambient = 0.35;
  float intensity = ambient + (1.0 - ambient) * ndl;
  o_color = vec4(u_color * intensity, 1.0);
}
)GLSL";

}  // namespace

SimRendererWidget::SimRendererWidget(QWidget* parent)
    : QOpenGLWidget(parent) {
  setMouseTracking(false);
  setMinimumSize(400, 300);
  setFocusPolicy(Qt::StrongFocus);   // needs key focus for G/R/X/Y/Z/Esc
}

void SimRendererWidget::setMotorsEditable(bool on) {
  editable_ = on;
  if (!on) {            // leaving edit mode: drop any selection / live tool
    if (tool_ != Tool::None) commitTool(false);
    selected_ = -1;
  }
  update();
}

void SimRendererWidget::setObstacleEditMode(bool on) {
  obsMode_ = on;
  if (!on) {
    if (obsTool_ != Tool::None) commitObsTool(false);
    selObs_ = -1;
  }
  update();
}

void SimRendererWidget::selectObstacle(int index) {
  if (index < 0 || index >= obstacles_.size()) index = -1;
  if (index == selObs_) return;
  if (obsTool_ != Tool::None) commitObsTool(false);
  selObs_ = index;
  update();
}

SimRendererWidget::~SimRendererWidget() {
  // Need a current context to release GL resources cleanly.
  makeCurrent();
  for (Mesh* m : {&ground_, &axes_, &body_, &rotor_, &thrustLine_, &comMarker_,
                  &gizmoArrow_, &gizmoRing_, &digitMesh_[0], &digitMesh_[1],
                  &digitMesh_[2], &digitMesh_[3], &northArrow_, &glyphN_,
                  &droneMesh_}) {
    m->vbo.destroy();
    m->vao.destroy();
  }
  doneCurrent();
}

void SimRendererWidget::setSnapshot(const vsim::SimSnapshot& s) {
  snap_ = s;
  update();   // schedules paintGL on the GUI thread
}

void SimRendererWidget::setDroneMesh(const std::vector<QVector3D>& positions,
                                     const std::vector<QVector3D>& normals) {
  pendingPos_ = positions;
  pendingNrm_ = normals;
  meshDirty_ = true;          // uploaded lazily in paintGL (needs GL context)
  update();
}

void SimRendererWidget::setMotorLayout(const std::array<QVector3D, 4>& pos,
                                       const std::array<QVector3D, 4>& axis,
                                       const std::array<int, 4>& spin) {
  motorPos_ = pos;
  motorAxis_ = axis;
  motorSpin_ = spin;
  update();
}

void SimRendererWidget::setComMarker(const QVector3D& com) {
  comOffset_ = com;
  update();
}

void SimRendererWidget::initializeGL() {
  initializeOpenGLFunctions();
  glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LINE_SMOOTH);

  prog_.addShaderFromSourceCode(QOpenGLShader::Vertex,   kVertexShader);
  prog_.addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShader);
  prog_.link();
  u_mvp_   = prog_.uniformLocation("u_mvp");
  u_color_ = prog_.uniformLocation("u_color");

  progLit_.addShaderFromSourceCode(QOpenGLShader::Vertex,   kLitVertexShader);
  progLit_.addShaderFromSourceCode(QOpenGLShader::Fragment, kLitFragmentShader);
  progLit_.link();
  ul_mvp_   = progLit_.uniformLocation("u_mvp");
  ul_nmat_  = progLit_.uniformLocation("u_nmat");
  ul_color_ = progLit_.uniformLocation("u_color");
  ul_light_ = progLit_.uniformLocation("u_lightdir");

  buildGroundGrid();
  buildObstacleMeshes();
  buildAxes();
  buildDroneBody();
  buildRotorDisk();
  buildThrustLine();
  buildComMarker();
  buildGizmo();
  buildDigits();
  buildNorth();
}

void SimRendererWidget::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);
  proj_.setToIdentity();
  proj_.perspective(60.0f, float(w) / std::max(1, h), 0.05f, 200.0f);
}

QMatrix4x4 SimRendererWidget::cameraView() const {
  // Orbit around the drone's current world position. Convert
  // (radius, yaw, pitch) into a NED offset; remember NED up is -Z.
  const auto& tgt = snap_.pos_w;
  QVector3D offset(
      cam_radius_ * std::cos(cam_pitch_) * std::cos(cam_yaw_),
      cam_radius_ * std::cos(cam_pitch_) * std::sin(cam_yaw_),
      -cam_radius_ * std::sin(cam_pitch_)   // negative: pitch>0 -> above
  );
  QVector3D eye = tgt + offset;
  QMatrix4x4 view;
  view.lookAt(eye, tgt, QVector3D(0.0f, 0.0f, -1.0f));   // NED up = -Z
  return view;
}

void SimRendererWidget::paintGL() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (meshDirty_) uploadDroneMesh();

  QMatrix4x4 view = cameraView();

  // Ground at z=0, world axes at origin.
  drawMesh(ground_, view, QVector3D(0.25f, 0.27f, 0.32f));
  drawMesh(axes_,   view, QVector3D(1, 1, 1));

  // Static world obstacles (lit solids), each scaled/rotated/placed.
  for (int oi = 0; oi < obstacles_.size(); ++oi) {
    const vsim::Obstacle& o = obstacles_[oi];
    QMatrix4x4 m;
    m.translate(o.pos);
    m.rotate(o.rotate.x(), 1, 0, 0);
    m.rotate(o.rotate.y(), 0, 1, 0);
    m.rotate(o.rotate.z(), 0, 0, 1);
    const Mesh* mesh = &unitBox_;
    QVector3D col(0.40f, 0.45f, 0.55f);
    if (o.type == vsim::Obstacle::Sphere) {
      mesh = &unitSphere_;
      m.scale(o.size.x(), o.size.x(), o.size.x());   // radius = size.x
      col = QVector3D(0.50f, 0.42f, 0.55f);
    } else if (o.type == vsim::Obstacle::Cylinder) {
      mesh = &unitCyl_;
      m.scale(o.size.x(), o.size.x(), o.size.z());   // radius, height
      col = QVector3D(0.42f, 0.52f, 0.46f);
    } else {
      m.scale(o.size.x(), o.size.y(), o.size.z());   // box full extents
    }
    if (obsMode_ && oi == selObs_) col = QVector3D(0.95f, 0.80f, 0.30f);  // selected
    drawLit(*mesh, view, m, col);
  }

  // Drone body: apply pos+orientation. Quaternion is normalized by the
  // sim after every step.
  const QMatrix4x4 model = modelMatrix();

  if (hasMesh_) {
    drawLit(droneMesh_, view, model, QVector3D(0.80f, 0.81f, 0.85f));
  } else {
    drawMesh(body_, view * model, QVector3D(0.85f, 0.55f, 0.20f));
  }

  // Motor markers (disk colored by spin + activity) and a thrust-axis
  // line, at the editable body-frame positions/axes.
  for (int i = 0; i < 4; ++i) {
    QVector3D axisN = motorAxis_[i];
    if (axisN.lengthSquared() < 1e-12f) axisN = QVector3D(0, 0, -1);
    axisN.normalize();
    const QQuaternion ori = QQuaternion::rotationTo(QVector3D(0, 0, 1), axisN);

    QMatrix4x4 mr = model;
    mr.translate(motorPos_[i]);
    mr.rotate(ori);
    const QVector3D base = (motorSpin_[i] >= 0) ? QVector3D(0.30f, 0.85f, 0.30f)
                                                : QVector3D(0.85f, 0.30f, 0.30f);
    const float duty = snap_.motor_duty[i];
    const QVector3D c = (editable_ && i == selected_)
                            ? QVector3D(1.0f, 0.9f, 0.3f)  // selection highlight
                            : base * (0.4f + 0.6f * duty);
    drawMesh(rotor_, view * mr, c);

    // Thrust direction: unit +Z segment rotated onto the axis, shortened.
    QMatrix4x4 ml = model;
    ml.translate(motorPos_[i]);
    ml.rotate(ori);
    ml.scale(1.0f, 1.0f, 0.18f);
    drawMesh(thrustLine_, view * ml, QVector3D(0.95f, 0.95f, 0.40f));
  }

  // Center-of-mass crosshair (body frame, diagnostic).
  QMatrix4x4 mc = model;
  mc.translate(comOffset_);
  drawMesh(comMarker_, view * mc, QVector3D(0.95f, 0.35f, 0.95f));

  // Motor gizmo (only when editing and a motor is selected).
  if (editable_ && selected_ >= 0) drawGizmo(view);
  // Obstacle gizmo (World mode, an obstacle selected).
  if (obsMode_ && selObs_ >= 0) drawObsGizmo(view);

  // Motor numbers (1..4): only in Vehicle mode (editable_).
  if (editable_) drawMotorLabels(view);

  // Body +X / north arrow: shown in both Vehicle and World modes.
  drawNorthIndicator(view);
}

void SimRendererWidget::drawMesh(const Mesh& m, const QMatrix4x4& mvp,
                                 const QVector3D& color) {
  prog_.bind();
  prog_.setUniformValue(u_mvp_, proj_ * mvp);
  prog_.setUniformValue(u_color_, color);
  QOpenGLVertexArrayObject::Binder b(const_cast<QOpenGLVertexArrayObject*>(&m.vao));
  glDrawArrays(m.primitive, 0, m.vertex_count);
  prog_.release();
}

void SimRendererWidget::drawLit(const Mesh& m, const QMatrix4x4& view,
                                const QMatrix4x4& model,
                                const QVector3D& color) {
  if (m.vertex_count == 0) return;
  progLit_.bind();
  progLit_.setUniformValue(ul_mvp_, proj_ * view * model);
  progLit_.setUniformValue(ul_nmat_, model.normalMatrix());
  progLit_.setUniformValue(ul_color_, color);
  // Light mostly from above (NED up is -Z) with a slight side bias.
  progLit_.setUniformValue(ul_light_, QVector3D(0.3f, 0.2f, -1.0f));
  QOpenGLVertexArrayObject::Binder b(const_cast<QOpenGLVertexArrayObject*>(&m.vao));
  glDrawArrays(GL_TRIANGLES, 0, m.vertex_count);
  progLit_.release();
}

void SimRendererWidget::uploadDroneMesh() {
  meshDirty_ = false;
  hasMesh_ = !pendingPos_.empty();
  if (!hasMesh_) return;

  // Interleave [px,py,pz, nx,ny,nz] per vertex (triangle soup).
  std::vector<float> data;
  data.reserve(pendingPos_.size() * 6);
  for (size_t i = 0; i < pendingPos_.size(); ++i) {
    const QVector3D& p = pendingPos_[i];
    const QVector3D n = (i < pendingNrm_.size()) ? pendingNrm_[i]
                                                 : QVector3D(0, 0, 1);
    data.insert(data.end(), {p.x(), p.y(), p.z(), n.x(), n.y(), n.z()});
  }

  if (!droneMesh_.vao.isCreated()) droneMesh_.vao.create();
  droneMesh_.vao.bind();
  if (!droneMesh_.vbo.isCreated()) droneMesh_.vbo.create();
  droneMesh_.vbo.bind();
  droneMesh_.vbo.allocate(data.data(), int(data.size() * sizeof(float)));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        reinterpret_cast<void*>(3 * sizeof(float)));
  droneMesh_.vbo.release();
  droneMesh_.vao.release();
  droneMesh_.vertex_count = int(pendingPos_.size());
  droneMesh_.primitive = GL_TRIANGLES;

  pendingPos_.clear();
  pendingNrm_.clear();
}

// ---------- geometry generators ----------

void SimRendererWidget::uploadLitMesh(Mesh& m,
                                      const std::vector<float>& data) {
  if (!m.vao.isCreated()) m.vao.create();
  m.vao.bind();
  if (!m.vbo.isCreated()) m.vbo.create();
  m.vbo.bind();
  m.vbo.allocate(data.data(), int(data.size() * sizeof(float)));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        reinterpret_cast<void*>(3 * sizeof(float)));
  m.vbo.release();
  m.vao.release();
  m.vertex_count = int(data.size() / 6);
  m.primitive = GL_TRIANGLES;
}

void SimRendererWidget::buildObstacleMeshes() {
  auto tri = [](std::vector<float>& v, const QVector3D& a, const QVector3D& b,
                const QVector3D& c) {
    const QVector3D n = QVector3D::crossProduct(b - a, c - a).normalized();
    for (const QVector3D& p : {a, b, c})
      v.insert(v.end(), {p.x(), p.y(), p.z(), n.x(), n.y(), n.z()});
  };

  // --- unit box [-0.5,0.5]^3 (full-extent unit cube) ---
  {
    std::vector<float> v;
    const float h = 0.5f;
    const QVector3D c[8] = {
        {-h, -h, -h}, {h, -h, -h}, {h, h, -h}, {-h, h, -h},
        {-h, -h, h},  {h, -h, h},  {h, h, h},  {-h, h, h}};
    const int f[6][4] = {{0, 1, 2, 3}, {5, 4, 7, 6}, {4, 0, 3, 7},
                         {1, 5, 6, 2}, {4, 5, 1, 0}, {3, 2, 6, 7}};
    for (auto& q : f) {
      tri(v, c[q[0]], c[q[1]], c[q[2]]);
      tri(v, c[q[0]], c[q[2]], c[q[3]]);
    }
    uploadLitMesh(unitBox_, v);
  }

  // --- unit sphere (radius 1, UV tessellation) ---
  {
    std::vector<float> v;
    const int LA = 12, LO = 18;
    auto sph = [](double la, double lo) {
      return QVector3D(std::sin(la) * std::cos(lo), std::sin(la) * std::sin(lo),
                       std::cos(la));
    };
    for (int i = 0; i < LA; ++i)
      for (int j = 0; j < LO; ++j) {
        const double la0 = M_PI * i / LA, la1 = M_PI * (i + 1) / LA;
        const double lo0 = 2 * M_PI * j / LO, lo1 = 2 * M_PI * (j + 1) / LO;
        const QVector3D a = sph(la0, lo0), b = sph(la1, lo0),
                        c2 = sph(la1, lo1), d = sph(la0, lo1);
        // normals == positions for a unit sphere.
        for (const QVector3D& p : {a, b, c2})
          v.insert(v.end(), {p.x(), p.y(), p.z(), p.x(), p.y(), p.z()});
        for (const QVector3D& p : {a, c2, d})
          v.insert(v.end(), {p.x(), p.y(), p.z(), p.x(), p.y(), p.z()});
      }
    uploadLitMesh(unitSphere_, v);
  }

  // --- unit cylinder (radius 1, height 1 in z, +caps) ---
  {
    std::vector<float> v;
    const int N = 24;
    const float zt = 0.5f, zb = -0.5f;
    for (int j = 0; j < N; ++j) {
      const double a0 = 2 * M_PI * j / N, a1 = 2 * M_PI * (j + 1) / N;
      const QVector3D n0(std::cos(a0), std::sin(a0), 0),
          n1(std::cos(a1), std::sin(a1), 0);
      const QVector3D bt0(n0.x(), n0.y(), zt), bb0(n0.x(), n0.y(), zb),
          bt1(n1.x(), n1.y(), zt), bb1(n1.x(), n1.y(), zb);
      auto side = [&](const QVector3D& p, const QVector3D& n) {
        v.insert(v.end(), {p.x(), p.y(), p.z(), n.x(), n.y(), n.z()});
      };
      side(bb0, n0); side(bb1, n1); side(bt1, n1);
      side(bb0, n0); side(bt1, n1); side(bt0, n0);
      // caps (fan from axis point)
      tri(v, QVector3D(0, 0, zt), bt0, bt1);
      tri(v, QVector3D(0, 0, zb), bb1, bb0);
    }
    uploadLitMesh(unitCyl_, v);
  }
}

void SimRendererWidget::setObstacles(const QVector<vsim::Obstacle>& obs) {
  obstacles_ = obs;
  update();
}

void SimRendererWidget::buildGroundGrid() {
  std::vector<float> verts;
  for (int i = -kGridHalf; i <= kGridHalf; ++i) {
    float f = i * kGridStep;
    // Line along +X at y=f, z=0
    verts.insert(verts.end(), {-(float)kGridHalf, f, 0.0f});
    verts.insert(verts.end(), {+(float)kGridHalf, f, 0.0f});
    // Line along +Y at x=f, z=0
    verts.insert(verts.end(), {f, -(float)kGridHalf, 0.0f});
    verts.insert(verts.end(), {f, +(float)kGridHalf, 0.0f});
  }
  ground_.vao.create();
  ground_.vao.bind();
  ground_.vbo.create();
  ground_.vbo.bind();
  ground_.vbo.allocate(verts.data(),
                       int(verts.size() * sizeof(float)));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  ground_.vbo.release();
  ground_.vao.release();
  ground_.vertex_count = int(verts.size() / 3);
  ground_.primitive = GL_LINES;
}

void SimRendererWidget::buildAxes() {
  const float L = 1.0f;
  // X=red, Y=green, Z=blue per draw call - here just three line
  // segments and we'll re-tint per-axis later. For simplicity ship a
  // single white set; per-axis color would need separate draws.
  float verts[] = {
    0,0,0,  L,0,0,    // +X (north)
    0,0,0,  0,L,0,    // +Y (east)
    0,0,0,  0,0,L,    // +Z (down)
  };
  axes_.vao.create();
  axes_.vao.bind();
  axes_.vbo.create();
  axes_.vbo.bind();
  axes_.vbo.allocate(verts, sizeof(verts));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  axes_.vbo.release();
  axes_.vao.release();
  axes_.vertex_count = 6;
  axes_.primitive = GL_LINES;
}

void SimRendererWidget::buildDroneBody() {
  // Simple box from triangles, centered at body origin, extents kBodyL,
  // kBodyW, kBodyH. Body Z is "down" in NED; for the rendered body we
  // draw a thin plate so the orientation reads cleanly from above.
  const float x = kBodyL * 0.5f, y = kBodyW * 0.5f, z = kBodyH * 0.5f;
  float v[] = {
    // top
    -x,-y,-z,  x,-y,-z,  x, y,-z,
    -x,-y,-z,  x, y,-z, -x, y,-z,
    // bottom
    -x,-y, z,  x, y, z,  x,-y, z,
    -x,-y, z, -x, y, z,  x, y, z,
    // sides
    -x,-y,-z, -x, y,-z, -x, y, z,
    -x,-y,-z, -x, y, z, -x,-y, z,
     x,-y,-z,  x,-y, z,  x, y, z,
     x,-y,-z,  x, y, z,  x, y,-z,
    -x,-y,-z, -x,-y, z,  x,-y, z,
    -x,-y,-z,  x,-y, z,  x,-y,-z,
    -x, y,-z,  x, y,-z,  x, y, z,
    -x, y,-z,  x, y, z, -x, y, z,
  };
  body_.vao.create();
  body_.vao.bind();
  body_.vbo.create();
  body_.vbo.bind();
  body_.vbo.allocate(v, sizeof(v));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  body_.vbo.release();
  body_.vao.release();
  body_.vertex_count = sizeof(v) / (3 * sizeof(float));
  body_.primitive = GL_TRIANGLES;
}

void SimRendererWidget::buildRotorDisk() {
  // Disk as triangle fan around body Z = 0 (flat on the body plane).
  std::vector<float> v;
  v.insert(v.end(), {0.0f, 0.0f, 0.0f});
  for (int i = 0; i <= kRotorSeg; ++i) {
    float a = float(i) / kRotorSeg * 2.0f * float(M_PI);
    v.insert(v.end(), {kRotorR * std::cos(a),
                       kRotorR * std::sin(a),
                       0.0f});
  }
  rotor_.vao.create();
  rotor_.vao.bind();
  rotor_.vbo.create();
  rotor_.vbo.bind();
  rotor_.vbo.allocate(v.data(), int(v.size() * sizeof(float)));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  rotor_.vbo.release();
  rotor_.vao.release();
  rotor_.vertex_count = int(v.size() / 3);
  rotor_.primitive = GL_TRIANGLE_FAN;
}

void SimRendererWidget::buildThrustLine() {
  // Unit segment along +Z; paintGL rotates it onto each motor's axis.
  float v[] = {0, 0, 0, 0, 0, 1};
  thrustLine_.vao.create();
  thrustLine_.vao.bind();
  thrustLine_.vbo.create();
  thrustLine_.vbo.bind();
  thrustLine_.vbo.allocate(v, sizeof(v));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  thrustLine_.vbo.release();
  thrustLine_.vao.release();
  thrustLine_.vertex_count = 2;
  thrustLine_.primitive = GL_LINES;
}

void SimRendererWidget::buildComMarker() {
  // Small 3-axis crosshair centered at origin.
  const float s = 0.06f;
  float v[] = {
      -s, 0, 0,  s, 0, 0,
       0,-s, 0,  0, s, 0,
       0, 0,-s,  0, 0, s,
  };
  comMarker_.vao.create();
  comMarker_.vao.bind();
  comMarker_.vbo.create();
  comMarker_.vbo.bind();
  comMarker_.vbo.allocate(v, sizeof(v));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  comMarker_.vbo.release();
  comMarker_.vao.release();
  comMarker_.vertex_count = 6;
  comMarker_.primitive = GL_LINES;
}

void SimRendererWidget::buildGizmo() {
  // Unit +X arrow shaft (rotated per axis in drawGizmo).
  {
    float v[] = {0, 0, 0, 1, 0, 0};
    gizmoArrow_.vao.create();
    gizmoArrow_.vao.bind();
    gizmoArrow_.vbo.create();
    gizmoArrow_.vbo.bind();
    gizmoArrow_.vbo.allocate(v, sizeof(v));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    gizmoArrow_.vbo.release();
    gizmoArrow_.vao.release();
    gizmoArrow_.vertex_count = 2;
    gizmoArrow_.primitive = GL_LINES;
  }
  // Unit circle in the XY plane (line loop), rotated per axis for rings.
  {
    std::vector<float> v;
    const int seg = 48;
    for (int i = 0; i < seg; ++i) {
      float a = float(i) / seg * 2.0f * float(M_PI);
      v.insert(v.end(), {std::cos(a), std::sin(a), 0.0f});
    }
    gizmoRing_.vao.create();
    gizmoRing_.vao.bind();
    gizmoRing_.vbo.create();
    gizmoRing_.vbo.bind();
    gizmoRing_.vbo.allocate(v.data(), int(v.size() * sizeof(float)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    gizmoRing_.vbo.release();
    gizmoRing_.vao.release();
    gizmoRing_.vertex_count = seg;
    gizmoRing_.primitive = GL_LINE_LOOP;
  }
}

void SimRendererWidget::buildDigits() {
  // Each glyph is a set of 2D line segments (x0,y0,x1,y1,...) in a unit
  // cell, expanded to (x,y,0) vertices and drawn billboarded as GL_LINES.
  const std::vector<std::vector<float>> strokes = {
      // "1"
      {0, -0.5f, 0, 0.5f,  -0.18f, 0.32f, 0, 0.5f,  -0.2f, -0.5f, 0.2f, -0.5f},
      // "2"
      {-0.3f, 0.5f, 0.3f, 0.5f,  0.3f, 0.5f, 0.3f, 0.0f,  0.3f, 0.0f, -0.3f, 0.0f,
       -0.3f, 0.0f, -0.3f, -0.5f,  -0.3f, -0.5f, 0.3f, -0.5f},
      // "3"
      {-0.3f, 0.5f, 0.3f, 0.5f,  0.3f, 0.5f, 0.3f, -0.5f,  -0.25f, 0.0f, 0.3f, 0.0f,
       -0.3f, -0.5f, 0.3f, -0.5f},
      // "4"
      {-0.3f, 0.5f, -0.3f, 0.0f,  -0.3f, 0.0f, 0.3f, 0.0f,  0.3f, 0.5f, 0.3f, -0.5f},
  };
  for (int d = 0; d < 4; ++d) {
    std::vector<float> v;
    const auto& s = strokes[d];
    for (size_t k = 0; k + 1 < s.size(); k += 2) {
      v.insert(v.end(), {s[k], s[k + 1], 0.0f});
    }
    Mesh& m = digitMesh_[d];
    m.vao.create();
    m.vao.bind();
    m.vbo.create();
    m.vbo.bind();
    m.vbo.allocate(v.data(), int(v.size() * sizeof(float)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    m.vbo.release();
    m.vao.release();
    m.vertex_count = int(v.size() / 3);
    m.primitive = GL_LINES;
  }
}

void SimRendererWidget::drawMotorLabels(const QMatrix4x4& view) {
  QVector3D eye, tgt;
  cameraEyeTarget(&eye, &tgt);
  const QVector3D fwd = (tgt - eye).normalized();
  const QVector3D worldUp(0, 0, -1);   // NED up
  QVector3D right = QVector3D::crossProduct(fwd, worldUp);
  if (right.lengthSquared() < 1e-6f) right = QVector3D(1, 0, 0);
  right.normalize();
  const QVector3D up = QVector3D::crossProduct(right, fwd).normalized();

  const QMatrix4x4 model = modelMatrix();
  const float s = 0.07f;   // glyph size [m]
  glDisable(GL_DEPTH_TEST);
  for (int i = 0; i < 4; ++i) {
    const QVector3D pos = model.map(motorPos_[i]) + up * 0.12f;  // above marker
    QMatrix4x4 m;
    m.setColumn(0, QVector4D(right * s, 0));
    m.setColumn(1, QVector4D(up * s, 0));
    m.setColumn(2, QVector4D(-fwd * s, 0));
    m.setColumn(3, QVector4D(pos, 1));
    const QVector3D col = (editable_ && i == selected_)
                              ? QVector3D(1.0f, 0.95f, 0.4f)   // selected
                              : QVector3D(0.92f, 0.92f, 0.96f);
    drawMesh(digitMesh_[i], view * m, col);
  }
  glEnable(GL_DEPTH_TEST);
}

void SimRendererWidget::buildNorth() {
  // Arrow along body +X (north), in the body XY plane: shaft + two head
  // segments. Drawn through the model matrix so it rotates with the body.
  {
    float v[] = {
        0.0f, 0.0f, 0.0f,  0.35f, 0.0f, 0.0f,     // shaft
        0.35f, 0.0f, 0.0f, 0.27f, 0.05f, 0.0f,    // head
        0.35f, 0.0f, 0.0f, 0.27f, -0.05f, 0.0f,
    };
    northArrow_.vao.create();
    northArrow_.vao.bind();
    northArrow_.vbo.create();
    northArrow_.vbo.bind();
    northArrow_.vbo.allocate(v, sizeof(v));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    northArrow_.vbo.release();
    northArrow_.vao.release();
    northArrow_.vertex_count = 6;
    northArrow_.primitive = GL_LINES;
  }
  // "N" glyph (billboarded at the tip).
  {
    float v[] = {
        -0.3f, -0.5f, 0.0f, -0.3f, 0.5f, 0.0f,    // left vertical
        -0.3f, 0.5f, 0.0f,  0.3f, -0.5f, 0.0f,    // diagonal
        0.3f, -0.5f, 0.0f,  0.3f, 0.5f, 0.0f,     // right vertical
    };
    glyphN_.vao.create();
    glyphN_.vao.bind();
    glyphN_.vbo.create();
    glyphN_.vbo.bind();
    glyphN_.vbo.allocate(v, sizeof(v));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glyphN_.vbo.release();
    glyphN_.vao.release();
    glyphN_.vertex_count = 6;
    glyphN_.primitive = GL_LINES;
  }
}

void SimRendererWidget::drawNorthIndicator(const QMatrix4x4& view) {
  const QMatrix4x4 model = modelMatrix();
  const QVector3D north(1.0f, 0.45f, 0.45f);   // matches +X axis colouring
  glDisable(GL_DEPTH_TEST);

  // Arrow points along body +X in 3D (not billboarded — it shows heading).
  drawMesh(northArrow_, view * model, north);

  // "N" at the tip, billboarded so it's always readable.
  QVector3D eye, tgt;
  cameraEyeTarget(&eye, &tgt);
  const QVector3D fwd = (tgt - eye).normalized();
  QVector3D right = QVector3D::crossProduct(fwd, QVector3D(0, 0, -1));
  if (right.lengthSquared() < 1e-6f) right = QVector3D(1, 0, 0);
  right.normalize();
  const QVector3D up = QVector3D::crossProduct(right, fwd).normalized();
  const float s = 0.055f;
  const QVector3D pos = model.map(QVector3D(0.44f, 0.0f, 0.0f)) + up * 0.05f;
  QMatrix4x4 m;
  m.setColumn(0, QVector4D(right * s, 0));
  m.setColumn(1, QVector4D(up * s, 0));
  m.setColumn(2, QVector4D(-fwd * s, 0));
  m.setColumn(3, QVector4D(pos, 1));
  drawMesh(glyphN_, view * m, north);

  glEnable(GL_DEPTH_TEST);
}

// ---------- camera controls ----------

void SimRendererWidget::mousePressEvent(QMouseEvent* e) {
  setFocus();
  last_mouse_ = e->pos();

  // Obstacle editing (World mode) takes precedence over camera orbit.
  if (obsMode_) {
    if (obsTool_ != Tool::None) {
      if (e->button() == Qt::LeftButton)       commitObsTool(true);
      else if (e->button() == Qt::RightButton) commitObsTool(false);
      return;
    }
    if (e->button() == Qt::LeftButton) {
      int idx = -1;
      if (pickObstacle(e->pos(), &idx)) {
        selObs_ = idx;
        emit obstacleSelected(idx);
        update();
        return;  // consumed — don't orbit
      }
      if (selObs_ != -1) { selObs_ = -1; emit obstacleSelected(-1); update(); }
    }
    return;  // empty click: mouseMove (button held) orbits the camera
  }

  // A live tool: left-click confirms the transform, right-click cancels.
  if (editable_ && tool_ != Tool::None) {
    if (e->button() == Qt::LeftButton)       commitTool(true);
    else if (e->button() == Qt::RightButton) commitTool(false);
    return;
  }

  // Otherwise left-click tries to select a motor; an empty click
  // deselects and falls through to camera orbit.
  if (editable_ && e->button() == Qt::LeftButton) {
    int idx = -1;
    if (pickMotor(e->pos(), &idx)) {
      selected_ = idx;
      emit motorSelected(idx);
      update();
      return;   // consumed — don't orbit
    }
    if (selected_ != -1) { selected_ = -1; update(); }
  }
}

void SimRendererWidget::mouseMoveEvent(QMouseEvent* e) {
  // While a tool is live the mouse drives it (no button held, Blender-style).
  if (obsMode_ && obsTool_ != Tool::None) {
    updateObsToolFromMouse(e->pos());
    return;
  }
  if (editable_ && tool_ != Tool::None) {
    updateToolFromMouse(e->pos());
    return;
  }
  QPoint d = e->pos() - last_mouse_;
  last_mouse_ = e->pos();
  if (e->buttons() & Qt::LeftButton) {
    cam_yaw_   -= d.x() * 0.01f;
    cam_pitch_ += d.y() * 0.01f;
    const float lim = 1.4f;
    if (cam_pitch_ >  lim) cam_pitch_ =  lim;
    if (cam_pitch_ < -lim) cam_pitch_ = -lim;
    update();
  }
}

void SimRendererWidget::keyPressEvent(QKeyEvent* e) {
  if (obsMode_) {
    switch (e->key()) {
      case Qt::Key_G: if (selObs_ >= 0) beginObsTool(Tool::Move);   break;
      case Qt::Key_R: if (selObs_ >= 0) beginObsTool(Tool::Rotate); break;
      case Qt::Key_S: if (selObs_ >= 0) beginObsTool(Tool::Scale);  break;
      case Qt::Key_X: if (obsTool_ != Tool::None) { obsAxis_ = 0; updateObsToolFromMouse(last_mouse_); } break;
      case Qt::Key_Y: if (obsTool_ != Tool::None) { obsAxis_ = 1; updateObsToolFromMouse(last_mouse_); } break;
      case Qt::Key_Z: if (obsTool_ != Tool::None) { obsAxis_ = 2; updateObsToolFromMouse(last_mouse_); } break;
      case Qt::Key_Escape:
        if (obsTool_ != Tool::None) commitObsTool(false);
        else if (selObs_ != -1) { selObs_ = -1; emit obstacleSelected(-1); update(); }
        break;
      case Qt::Key_Return:
      case Qt::Key_Enter:
        if (obsTool_ != Tool::None) commitObsTool(true);
        break;
      default: QOpenGLWidget::keyPressEvent(e); return;
    }
    return;
  }
  if (!editable_) { QOpenGLWidget::keyPressEvent(e); return; }
  switch (e->key()) {
    case Qt::Key_G: if (selected_ >= 0) beginTool(Tool::Move);   break;
    case Qt::Key_R: if (selected_ >= 0) beginTool(Tool::Rotate); break;
    case Qt::Key_X: if (tool_ != Tool::None) { axisLock_ = 0; updateToolFromMouse(last_mouse_); } break;
    case Qt::Key_Y: if (tool_ != Tool::None) { axisLock_ = 1; updateToolFromMouse(last_mouse_); } break;
    case Qt::Key_Z: if (tool_ != Tool::None) { axisLock_ = 2; updateToolFromMouse(last_mouse_); } break;
    case Qt::Key_Escape:
      if (tool_ != Tool::None) commitTool(false);
      else if (selected_ != -1) { selected_ = -1; update(); }
      break;
    case Qt::Key_Return:
    case Qt::Key_Enter:
      if (tool_ != Tool::None) commitTool(true);
      break;
    default:
      QOpenGLWidget::keyPressEvent(e);
      return;
  }
}

// ---------- gizmo editing ----------

QMatrix4x4 SimRendererWidget::modelMatrix() const {
  QMatrix4x4 m;
  m.translate(snap_.pos_w);
  m.rotate(snap_.att);
  return m;
}

void SimRendererWidget::cameraEyeTarget(QVector3D* eye, QVector3D* target) const {
  const QVector3D tgt = snap_.pos_w;
  const QVector3D offset(
      cam_radius_ * std::cos(cam_pitch_) * std::cos(cam_yaw_),
      cam_radius_ * std::cos(cam_pitch_) * std::sin(cam_yaw_),
      -cam_radius_ * std::sin(cam_pitch_));
  if (target) *target = tgt;
  if (eye)    *eye = tgt + offset;
}

void SimRendererWidget::rayThroughPixel(const QPoint& px, QVector3D* o,
                                        QVector3D* d) const {
  const float w = std::max(1, width()), h = std::max(1, height());
  const float ndcx = 2.0f * px.x() / w - 1.0f;
  const float ndcy = 1.0f - 2.0f * px.y() / h;
  bool ok = false;
  const QMatrix4x4 inv = (proj_ * cameraView()).inverted(&ok);
  const QVector3D np = (inv * QVector4D(ndcx, ndcy, -1.0f, 1.0f)).toVector3DAffine();
  const QVector3D fp = (inv * QVector4D(ndcx, ndcy,  1.0f, 1.0f)).toVector3DAffine();
  if (o) *o = np;
  if (d) *d = (fp - np).normalized();
}

bool SimRendererWidget::pickMotor(const QPoint& px, int* outIndex) const {
  const QMatrix4x4 vp = proj_ * cameraView();
  const QMatrix4x4 model = modelMatrix();
  const float w = std::max(1, width()), h = std::max(1, height());
  float best = 22.0f;   // pixel radius
  int bestI = -1;
  for (int i = 0; i < 4; ++i) {
    const QVector3D wp = model.map(motorPos_[i]);
    const QVector4D clip = vp * QVector4D(wp, 1.0f);
    if (clip.w() <= 0.0f) continue;
    const float sx = (clip.x() / clip.w() * 0.5f + 0.5f) * w;
    const float sy = (1.0f - (clip.y() / clip.w() * 0.5f + 0.5f)) * h;
    const float dpix = std::hypot(sx - px.x(), sy - px.y());
    if (dpix < best) { best = dpix; bestI = i; }
  }
  if (bestI >= 0 && outIndex) *outIndex = bestI;
  return bestI >= 0;
}

void SimRendererWidget::beginTool(Tool t) {
  if (selected_ < 0) return;
  tool_ = t;
  axisLock_ = -1;
  startPos_  = motorPos_[selected_];
  startAxis_ = motorAxis_[selected_];
  toolStartMouse_ = last_mouse_ = mapFromGlobal(QCursor::pos());
  startWorld_ = modelMatrix().map(startPos_);
  // Drag plane: through the motor, facing the camera.
  QVector3D eye, tgt;
  cameraEyeTarget(&eye, &tgt);
  const QVector3D fwd = (tgt - eye).normalized();
  QVector3D o, d;
  rayThroughPixel(toolStartMouse_, &o, &d);
  const float denom = QVector3D::dotProduct(d, fwd);
  const float tt = (std::fabs(denom) > 1e-6f)
                       ? QVector3D::dotProduct(startWorld_ - o, fwd) / denom
                       : 0.0f;
  planeHit0_ = o + d * tt;
  setMouseTracking(true);
  update();
}

void SimRendererWidget::updateToolFromMouse(const QPoint& px) {
  if (selected_ < 0 || tool_ == Tool::None) return;
  last_mouse_ = px;
  const QMatrix4x4 model = modelMatrix();

  if (tool_ == Tool::Move) {
    QVector3D eye, tgt;
    cameraEyeTarget(&eye, &tgt);
    const QVector3D fwd = (tgt - eye).normalized();
    QVector3D o, d;
    rayThroughPixel(px, &o, &d);
    const float denom = QVector3D::dotProduct(d, fwd);
    const float tt = (std::fabs(denom) > 1e-6f)
                         ? QVector3D::dotProduct(startWorld_ - o, fwd) / denom
                         : 0.0f;
    QVector3D delta = (o + d * tt) - planeHit0_;
    if (axisLock_ >= 0) {
      const QVector3D axisB(axisLock_ == 0 ? 1.f : 0.f, axisLock_ == 1 ? 1.f : 0.f,
                            axisLock_ == 2 ? 1.f : 0.f);
      const QVector3D axisW = model.mapVector(axisB).normalized();
      delta = axisW * QVector3D::dotProduct(delta, axisW);
    }
    motorPos_[selected_] = model.inverted().map(startWorld_ + delta);
  } else {  // Rotate the thrust axis about a body axis (default X).
    const int k = (axisLock_ >= 0) ? axisLock_ : 0;
    const QVector3D kk(k == 0 ? 1.f : 0.f, k == 1 ? 1.f : 0.f, k == 2 ? 1.f : 0.f);
    const float ang = (px.x() - toolStartMouse_.x()) * 0.01f;  // rad
    const QVector3D a = startAxis_;
    const QVector3D r = a * std::cos(ang) +
                        QVector3D::crossProduct(kk, a) * std::sin(ang) +
                        kk * QVector3D::dotProduct(kk, a) * (1.0f - std::cos(ang));
    motorAxis_[selected_] = r.normalized();
  }
  update();
}

void SimRendererWidget::commitTool(bool confirm) {
  if (tool_ == Tool::None) return;
  tool_ = Tool::None;
  axisLock_ = -1;
  setMouseTracking(false);
  if (confirm && selected_ >= 0) {
    emit motorEdited(selected_, motorPos_[selected_], motorAxis_[selected_]);
  } else if (selected_ >= 0) {     // cancelled: restore the working copy
    motorPos_[selected_]  = startPos_;
    motorAxis_[selected_] = startAxis_;
  }
  update();
}

void SimRendererWidget::drawGizmo(const QMatrix4x4& view) {
  if (selected_ < 0) return;
  QMatrix4x4 base = modelMatrix();
  base.translate(motorPos_[selected_]);
  const float L = 0.18f;
  const QVector3D cX(0.90f, 0.25f, 0.25f), cY(0.25f, 0.85f, 0.25f),
      cZ(0.30f, 0.55f, 1.0f), hi(1.0f, 0.95f, 0.40f);

  // Gizmo draws on top of the body for grabbability.
  glDisable(GL_DEPTH_TEST);
  if (tool_ == Tool::Rotate) {
    { QMatrix4x4 m = base; m.scale(L); drawMesh(gizmoRing_, view * m, axisLock_ == 2 ? hi : cZ); }
    { QMatrix4x4 m = base; m.rotate(90, 1, 0, 0); m.scale(L); drawMesh(gizmoRing_, view * m, axisLock_ == 1 ? hi : cY); }
    { QMatrix4x4 m = base; m.rotate(90, 0, 1, 0); m.scale(L); drawMesh(gizmoRing_, view * m, axisLock_ == 0 ? hi : cX); }
  } else {  // Move (or just-selected): three axis arrows.
    { QMatrix4x4 m = base; m.scale(L); drawMesh(gizmoArrow_, view * m, axisLock_ == 0 ? hi : cX); }
    { QMatrix4x4 m = base; m.rotate(90, 0, 0, 1); m.scale(L); drawMesh(gizmoArrow_, view * m, axisLock_ == 1 ? hi : cY); }
    { QMatrix4x4 m = base; m.rotate(-90, 0, 1, 0); m.scale(L); drawMesh(gizmoArrow_, view * m, axisLock_ == 2 ? hi : cZ); }
  }
  glEnable(GL_DEPTH_TEST);
}

// ---------- obstacle gizmo editing ----------

bool SimRendererWidget::pickObstacle(const QPoint& px, int* outIndex) const {
  const QMatrix4x4 vp = proj_ * cameraView();
  const float w = std::max(1, width()), h = std::max(1, height());
  float best = 40.0f;   // pixel radius (obstacles are larger than motors)
  int bestI = -1;
  for (int i = 0; i < obstacles_.size(); ++i) {
    const QVector4D clip = vp * QVector4D(obstacles_[i].pos, 1.0f);
    if (clip.w() <= 0.0f) continue;
    const float sx = (clip.x() / clip.w() * 0.5f + 0.5f) * w;
    const float sy = (1.0f - (clip.y() / clip.w() * 0.5f + 0.5f)) * h;
    const float dpix = std::hypot(sx - px.x(), sy - px.y());
    if (dpix < best) { best = dpix; bestI = i; }
  }
  if (bestI >= 0 && outIndex) *outIndex = bestI;
  return bestI >= 0;
}

void SimRendererWidget::beginObsTool(Tool t) {
  if (selObs_ < 0 || selObs_ >= obstacles_.size()) return;
  obsTool_ = t;
  obsAxis_ = -1;
  obsStartPos_  = obstacles_[selObs_].pos;
  obsStartSize_ = obstacles_[selObs_].size;
  obsStartRot_  = obstacles_[selObs_].rotate;
  obsStartMouse_ = last_mouse_ = mapFromGlobal(QCursor::pos());
  // Drag plane: through the obstacle, facing the camera (world frame).
  QVector3D eye, tgt;
  cameraEyeTarget(&eye, &tgt);
  const QVector3D fwd = (tgt - eye).normalized();
  QVector3D o, d;
  rayThroughPixel(obsStartMouse_, &o, &d);
  const float denom = QVector3D::dotProduct(d, fwd);
  const float tt = (std::fabs(denom) > 1e-6f)
                       ? QVector3D::dotProduct(obsStartPos_ - o, fwd) / denom
                       : 0.0f;
  obsPlaneHit0_ = o + d * tt;
  setMouseTracking(true);
  update();
}

void SimRendererWidget::updateObsToolFromMouse(const QPoint& px) {
  if (selObs_ < 0 || obsTool_ == Tool::None) return;
  last_mouse_ = px;
  vsim::Obstacle& ob = obstacles_[selObs_];

  if (obsTool_ == Tool::Move) {
    QVector3D eye, tgt;
    cameraEyeTarget(&eye, &tgt);
    const QVector3D fwd = (tgt - eye).normalized();
    QVector3D o, d;
    rayThroughPixel(px, &o, &d);
    const float denom = QVector3D::dotProduct(d, fwd);
    const float tt = (std::fabs(denom) > 1e-6f)
                         ? QVector3D::dotProduct(obsStartPos_ - o, fwd) / denom
                         : 0.0f;
    QVector3D delta = (o + d * tt) - obsPlaneHit0_;
    if (obsAxis_ >= 0) {
      QVector3D ax(obsAxis_ == 0 ? 1.f : 0.f, obsAxis_ == 1 ? 1.f : 0.f,
                   obsAxis_ == 2 ? 1.f : 0.f);
      delta = ax * QVector3D::dotProduct(delta, ax);
    }
    ob.pos = obsStartPos_ + delta;
  } else if (obsTool_ == Tool::Rotate) {
    const int k = (obsAxis_ >= 0) ? obsAxis_ : 2;  // default about Z
    const float deg = (px.x() - obsStartMouse_.x()) * 0.5f;
    ob.rotate = obsStartRot_;
    ob.rotate[k] = obsStartRot_[k] + deg;
  } else {  // Scale
    const float f = std::clamp(1.0f + (px.x() - obsStartMouse_.x()) * 0.01f,
                               0.05f, 20.0f);
    if (obsAxis_ >= 0) {
      ob.size = obsStartSize_;
      ob.size[obsAxis_] = std::max(0.05f, obsStartSize_[obsAxis_] * f);
    } else {
      ob.size = obsStartSize_ * f;
      for (int i = 0; i < 3; ++i) ob.size[i] = std::max(0.05f, ob.size[i]);
    }
  }
  update();
}

void SimRendererWidget::commitObsTool(bool confirm) {
  if (obsTool_ == Tool::None) return;
  obsTool_ = Tool::None;
  obsAxis_ = -1;
  setMouseTracking(false);
  if (selObs_ >= 0 && selObs_ < obstacles_.size()) {
    if (confirm) {
      const vsim::Obstacle& o = obstacles_[selObs_];
      emit obstacleEdited(selObs_, o.pos, o.size, o.rotate);
    } else {
      obstacles_[selObs_].pos = obsStartPos_;
      obstacles_[selObs_].size = obsStartSize_;
      obstacles_[selObs_].rotate = obsStartRot_;
    }
  }
  update();
}

void SimRendererWidget::drawObsGizmo(const QMatrix4x4& view) {
  if (selObs_ < 0 || selObs_ >= obstacles_.size()) return;
  QMatrix4x4 base;
  base.translate(obstacles_[selObs_].pos);
  const float L = 0.7f;
  const QVector3D cX(0.90f, 0.25f, 0.25f), cY(0.25f, 0.85f, 0.25f),
      cZ(0.30f, 0.55f, 1.0f), hi(1.0f, 0.95f, 0.40f);
  glDisable(GL_DEPTH_TEST);
  if (obsTool_ == Tool::Rotate) {
    { QMatrix4x4 m = base; m.scale(L); drawMesh(gizmoRing_, view * m, obsAxis_ == 2 ? hi : cZ); }
    { QMatrix4x4 m = base; m.rotate(90, 1, 0, 0); m.scale(L); drawMesh(gizmoRing_, view * m, obsAxis_ == 1 ? hi : cY); }
    { QMatrix4x4 m = base; m.rotate(90, 0, 1, 0); m.scale(L); drawMesh(gizmoRing_, view * m, obsAxis_ == 0 ? hi : cX); }
  } else {  // Move / Scale / just-selected: three axis arrows.
    { QMatrix4x4 m = base; m.scale(L); drawMesh(gizmoArrow_, view * m, obsAxis_ == 0 ? hi : cX); }
    { QMatrix4x4 m = base; m.rotate(90, 0, 0, 1); m.scale(L); drawMesh(gizmoArrow_, view * m, obsAxis_ == 1 ? hi : cY); }
    { QMatrix4x4 m = base; m.rotate(-90, 0, 1, 0); m.scale(L); drawMesh(gizmoArrow_, view * m, obsAxis_ == 2 ? hi : cZ); }
  }
  glEnable(GL_DEPTH_TEST);
}

void SimRendererWidget::wheelEvent(QWheelEvent* e) {
  const float k = (e->angleDelta().y() > 0) ? 0.9f : 1.1f;
  cam_radius_ *= k;
  if (cam_radius_ < 0.5f)  cam_radius_ = 0.5f;
  if (cam_radius_ > 50.0f) cam_radius_ = 50.0f;
  update();
}

}  // namespace vsim
