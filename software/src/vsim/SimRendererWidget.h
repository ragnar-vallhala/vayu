#pragma once

#include "SimWorker.h"

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>

#include <array>
#include <vector>

namespace vsim {

// OpenGL 3.3 core profile renderer for the in-app sim. Draws the
// drone in NED world coordinates directly (camera "up" vector is
// (0,0,-1)), so the world axes you see line up with what the IMU /
// firmware actually sees.
class SimRendererWidget : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT
 public:
  explicit SimRendererWidget(QWidget* parent = nullptr);
  ~SimRendererWidget() override;

 public slots:
  void setSnapshot(const vsim::SimSnapshot& s);

 public:
  // Replace the procedural box with an imported airframe mesh (triangle
  // soup + per-vertex normals, body frame, meters). Upload is deferred
  // to the next paintGL so this is safe to call from the UI thread.
  // Empty positions reverts to the procedural body.
  void setDroneMesh(const std::vector<QVector3D>& positions,
                    const std::vector<QVector3D>& normals);
  // Editable motor markers: body-frame position, unit thrust axis, and
  // spin (+1 CCW / -1 CW) per rotor. Drawn over the body.
  void setMotorLayout(const std::array<QVector3D, 4>& pos,
                      const std::array<QVector3D, 4>& axis,
                      const std::array<int, 4>& spin);
  // Body-frame center-of-mass marker (crosshair); diagnostic for how far
  // the modeled origin sits from the true CoM.
  void setComMarker(const QVector3D& com);

 protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;

  void mousePressEvent(QMouseEvent* e) override;
  void mouseMoveEvent (QMouseEvent* e) override;
  void wheelEvent     (QWheelEvent* e) override;

 private:
  // -- GL helpers --
  struct Mesh {
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer            vbo{QOpenGLBuffer::VertexBuffer};
    int vertex_count = 0;
    GLenum primitive = GL_TRIANGLES;
  };

  void buildGroundGrid();
  void buildAxes();
  void buildDroneBody();
  void buildRotorDisk();
  void buildThrustLine();
  void buildComMarker();
  void uploadDroneMesh();   // flushes pending_* into droneMesh_ (GL-current)

  void drawMesh(const Mesh& m, const QMatrix4x4& model,
                const QVector3D& color);
  // Draws a pos+normal mesh through the lit program. view and model are
  // kept separate so normals transform by mat3(model) in world space.
  void drawLit(const Mesh& m, const QMatrix4x4& view,
               const QMatrix4x4& model, const QVector3D& color);

  QMatrix4x4 cameraView() const;

  // Flat shader: vec3 position + uniform color + MVP (grid/axes/markers).
  QOpenGLShaderProgram prog_;
  int u_mvp_   = -1;
  int u_color_ = -1;

  // Lit shader: position + normal, directional Lambert + ambient. Used
  // for the imported airframe mesh so a real solid reads as 3D.
  QOpenGLShaderProgram progLit_;
  int ul_mvp_   = -1;
  int ul_nmat_  = -1;
  int ul_color_ = -1;
  int ul_light_ = -1;

  Mesh ground_;
  Mesh axes_;
  Mesh body_;
  Mesh rotor_;
  Mesh thrustLine_;   // unit segment along +Z, scaled per motor axis
  Mesh comMarker_;    // small 3-axis crosshair

  // Imported mesh (pos+normal interleaved). hasMesh_ gates body vs mesh.
  Mesh droneMesh_;
  bool hasMesh_   = false;
  bool meshDirty_ = false;
  std::vector<QVector3D> pendingPos_;
  std::vector<QVector3D> pendingNrm_;

  // Editable motor layout (defaults mirror the firmware quad geometry).
  std::array<QVector3D, 4> motorPos_ = {
      QVector3D( 0.13f, +0.22f, 0.0f), QVector3D(-0.13f, +0.20f, 0.0f),
      QVector3D(-0.13f, -0.20f, 0.0f), QVector3D( 0.13f, -0.22f, 0.0f)};
  std::array<QVector3D, 4> motorAxis_ = {
      QVector3D(0, 0, -1), QVector3D(0, 0, -1),
      QVector3D(0, 0, -1), QVector3D(0, 0, -1)};
  std::array<int, 4> motorSpin_ = {+1, -1, +1, -1};
  QVector3D comOffset_{0, 0, 0};

  // Camera state (orbit around drone).
  float cam_radius_ = 4.0f;
  float cam_yaw_    = 0.7f;   // around world -Z (NED up)
  float cam_pitch_  = -0.5f;  // tilt
  QPoint last_mouse_;

  QMatrix4x4 proj_;
  vsim::SimSnapshot snap_;
};

}  // namespace vsim
