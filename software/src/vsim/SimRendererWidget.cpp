#include "SimRendererWidget.h"

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
}

SimRendererWidget::~SimRendererWidget() {
  // Need a current context to release GL resources cleanly.
  makeCurrent();
  for (Mesh* m : {&ground_, &axes_, &body_, &rotor_, &thrustLine_, &comMarker_,
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
  buildAxes();
  buildDroneBody();
  buildRotorDisk();
  buildThrustLine();
  buildComMarker();
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

  // Drone body: apply pos+orientation. Quaternion is normalized by the
  // sim after every step.
  QMatrix4x4 model;
  model.translate(snap_.pos_w);
  model.rotate(snap_.att);

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
    drawMesh(rotor_, view * mr, base * (0.4f + 0.6f * duty));

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

// ---------- camera controls ----------

void SimRendererWidget::mousePressEvent(QMouseEvent* e) {
  last_mouse_ = e->pos();
}

void SimRendererWidget::mouseMoveEvent(QMouseEvent* e) {
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

void SimRendererWidget::wheelEvent(QWheelEvent* e) {
  const float k = (e->angleDelta().y() > 0) ? 0.9f : 1.1f;
  cam_radius_ *= k;
  if (cam_radius_ < 0.5f)  cam_radius_ = 0.5f;
  if (cam_radius_ > 50.0f) cam_radius_ = 50.0f;
  update();
}

}  // namespace vsim
