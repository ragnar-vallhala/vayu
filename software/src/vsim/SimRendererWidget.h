#pragma once

#include "SimWorker.h"

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>

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

  void drawMesh(const Mesh& m, const QMatrix4x4& model,
                const QVector3D& color);

  QMatrix4x4 cameraView() const;

  // Shader: takes vec3 position + uniform color + MVP.
  QOpenGLShaderProgram prog_;
  int u_mvp_   = -1;
  int u_color_ = -1;

  Mesh ground_;
  Mesh axes_;
  Mesh body_;
  Mesh rotor_;

  // Camera state (orbit around drone).
  float cam_radius_ = 4.0f;
  float cam_yaw_    = 0.7f;   // around world -Z (NED up)
  float cam_pitch_  = -0.5f;  // tilt
  QPoint last_mouse_;

  QMatrix4x4 proj_;
  vsim::SimSnapshot snap_;
};

}  // namespace vsim
