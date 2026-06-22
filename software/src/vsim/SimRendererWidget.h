#pragma once

#include "SimWorker.h"
#include "TrainingCourse.h"

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>

#include <array>
#include <map>
#include <memory>
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

  // Static world obstacles (boxes/spheres/cylinders) in the NED world frame.
  // Stored + redrawn each frame; safe to call from the UI thread.
  void setObstacles(const QVector<vsim::Obstacle>& obs);

  // Training course: glowing halo gates to fly through. setTrainingGates
  // replaces the gate layout; setTrainingActive updates which gate to aim for
  // (highlighted) and whether to draw the guidance arrow (drone -> next gate).
  // Empty gates clears the course.
  void setTrainingGates(const QVector<vsim::RingGate>& gates);
  void setTrainingActive(int activeIndex, bool showArrow);

  // Imported static world mesh (triangle soup + normals, NED world frame).
  // Deferred upload like the drone mesh; empty positions clears it. `colors`
  // is optional per-vertex RGB (baked from the source materials) — pass {} to
  // fall back to neutral grey.
  void setWorldMesh(const std::vector<QVector3D>& positions,
                    const std::vector<QVector3D>& normals,
                    const std::vector<QVector3D>& colors = {});

  // Streaming world chunks (endless procedural terrain). Each chunk is an
  // independently uploaded colored mesh keyed by a packed (cx,cy). The streamer
  // adds/removes chunks as the drone moves; uploads are deferred to the next
  // paintGL like the single world mesh. Coexists with obstacles/training; an
  // imported world mesh and chunks are mutually exclusive in practice.
  void setWorldChunk(qint64 key, const std::vector<QVector3D>& positions,
                     const std::vector<QVector3D>& normals,
                     const std::vector<QVector3D>& colors);
  void removeWorldChunk(qint64 key);
  void clearWorldChunks();
  bool hasWorldChunks() const {
    return !worldChunks_.empty() || !pendingChunkUploads_.empty();
  }
  // World-space XY the terrain streamer should centre on: the free-fly camera
  // when roaming (sim stopped), otherwise the drone. Lets endless terrain follow
  // both WASD navigation and actual flight.
  QVector3D streamCenter() const { return freeFly_ ? camPos_ : snap_.pos_w; }
  // NED heading (radians, 0 = north) the minimap should orient by: the free-fly
  // look direction when roaming, otherwise the drone's body heading.
  float viewHeadingRad() const;

  // Enable Blender-style obstacle gizmo editing (click-select, G move /
  // R rotate / S scale, X/Y/Z constrain) — World mode while the sim is
  // stopped. Mutually exclusive with motor editing.
  void setObstacleEditMode(bool on);
  void selectObstacle(int index);  // drive selection from the list

  // Onboard FPV camera: ride the drone, looking forward along body +X (the
  // body mesh is hidden so it doesn't fill the lens). Off = orbit camera.
  void setFpv(bool on) { fpv_ = on; update(); }
  bool fpv() const { return fpv_; }

  // Bird's-eye "down-cam": sit a fixed height above the drone and look straight
  // down (NED +Z), north up. Used by the Down-Cam PiP. Takes precedence over
  // fpv/orbit while on.
  void setDownCam(bool on) { downCam_ = on; update(); }
  bool downCam() const { return downCam_; }

  // Free-fly camera: WASD/QE to move, left-drag to look, NOT locked to the
  // drone. Used in World mode while the sim is stopped so you can roam the
  // imported world. Off = the camera orbits/follows the drone (3rd person).
  // Enabling seeds the free camera at the current orbit eye for a smooth swap.
  void setFreeFly(bool on);
  bool freeFly() const { return freeFly_; }

  // Show the imported world mesh + obstacles. Off in Vehicle mode (just the
  // airframe); on in World mode (the whole world).
  void setWorldVisible(bool on);

  // Enable Blender-style motor gizmo editing (click-select, G move /
  // R rotate, X/Y/Z constrain). Only meaningful when the sim is stopped;
  // the SimulatorWidget toggles this on stop / off on start.
  void setMotorsEditable(bool on);

 signals:
  // Emitted on gizmo confirm. pos is CoM-frame (same frame as
  // setMotorLayout); axis is the unit thrust axis. index in [0,3].
  void motorEdited(int index, QVector3D posComFrame, QVector3D axis);
  void motorSelected(int index);
  // Obstacle gizmo: new pos/size/rotate (world frame) on confirm; selection.
  void obstacleEdited(int index, QVector3D pos, QVector3D size,
                      QVector3D rotate);
  void obstacleSelected(int index);

 protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;

  void mousePressEvent(QMouseEvent* e) override;
  void mouseMoveEvent (QMouseEvent* e) override;
  void wheelEvent     (QWheelEvent* e) override;
  void keyPressEvent  (QKeyEvent* e) override;

 private:
  // -- GL helpers --
  struct Mesh {
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer            vbo{QOpenGLBuffer::VertexBuffer};
    int vertex_count = 0;
    GLenum primitive = GL_TRIANGLES;
  };

  void buildGroundGrid();
  void buildObstacleMeshes();   // unit box / sphere / cylinder (pos+normal)
  void buildAxes();
  void buildDroneBody();
  void buildRotorDisk();
  void buildThrustLine();
  void buildComMarker();
  void buildGizmo();        // unit +X arrow + unit XY ring for the gizmo
  void buildDigits();       // line-stroke glyphs for motor numbers 1..4
  void drawMotorLabels(const QMatrix4x4& view);  // billboarded numbers
  void buildNorth();        // body +X arrow + "N" glyph
  void drawNorthIndicator(const QMatrix4x4& view);
  void buildRing();         // unit torus (major R=1, in local XY plane)
  void buildGuideArrow();   // unit solid arrow along +X (shaft + cone head)
  void drawTraining(const QMatrix4x4& view);  // halo gates + guidance arrow
  void uploadDroneMesh();   // flushes pending_* into droneMesh_ (GL-current)
  void uploadWorldMesh();   // flushes pendingWorld_* into worldMesh_
  void flushChunkUpdates(); // applies queued chunk uploads/removals (GL-current)
  // Upload an interleaved [px,py,pz,nx,ny,nz] array into a lit-shader mesh.
  void uploadLitMesh(Mesh& m, const std::vector<float>& interleaved);
  // As uploadLitMesh, but the array is [px,py,pz,nx,ny,nz,r,g,b] and vertex
  // colors (attribute 2) are enabled — used for the colored world mesh.
  void uploadColoredMesh(Mesh& m, const std::vector<float>& interleaved);

  // ---- gizmo editing (Blender-style; active only when editable_) ----
  enum class Tool { None, Move, Rotate, Scale };
  QMatrix4x4 modelMatrix() const;                 // body -> world
  void cameraEyeTarget(QVector3D* eye, QVector3D* target) const;
  void rayThroughPixel(const QPoint& px, QVector3D* o, QVector3D* d) const;
  bool pickMotor(const QPoint& px, int* outIndex) const;
  void beginTool(Tool t);
  void updateToolFromMouse(const QPoint& px);
  void commitTool(bool confirm);
  void drawGizmo(const QMatrix4x4& view);

  // ---- obstacle gizmo editing (active only when obsMode_) ----
  bool pickObstacle(const QPoint& px, int* outIndex) const;
  void beginObsTool(Tool t);
  void updateObsToolFromMouse(const QPoint& px);
  void commitObsTool(bool confirm);
  void drawObsGizmo(const QMatrix4x4& view);

  void drawMesh(const Mesh& m, const QMatrix4x4& model,
                const QVector3D& color);
  // Draws a pos+normal mesh through the lit program. view and model are
  // kept separate so normals transform by mat3(model) in world space.
  void drawLit(const Mesh& m, const QMatrix4x4& view,
               const QMatrix4x4& model, const QVector3D& color);

  QMatrix4x4 cameraView() const;
  QVector3D  freeForward() const;        // free-cam look direction from yaw/pitch
  float      bodyYawRad() const;         // drone heading (yaw about world Z, NED)
  // Third-person orbit eye offset from the drone. The azimuth is locked to the
  // drone's heading (cam_yaw_ is the offset RELATIVE to the body, changed only
  // by mouse drag) so the camera yaws with the vehicle and stays easy to track.
  QVector3D  orbitOffset() const;
  bool       freeFlyMove(int key, bool fast);  // WASD/QE → move camPos_; true if used

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

  // Sky shader: attribute-less fullscreen triangle, view-ray gradient.
  QOpenGLShaderProgram progSky_;
  int us_invvp_ = -1;
  QOpenGLVertexArrayObject skyVao_;

  Mesh ground_;
  Mesh unitBox_;       // [-0.5,0.5]^3, pos+normal (lit) — scaled per obstacle
  Mesh unitSphere_;    // radius-1 UV sphere, pos+normal
  Mesh unitCyl_;       // radius-1, height-1 cylinder (+caps), pos+normal
  QVector<vsim::Obstacle> obstacles_;
  Mesh axes_;
  Mesh body_;
  Mesh rotor_;
  Mesh thrustLine_;   // unit segment along +Z, scaled per motor axis
  Mesh comMarker_;    // small 3-axis crosshair
  Mesh gizmoArrow_;   // unit segment along +X (rotated per axis)
  Mesh gizmoRing_;    // unit circle in XY plane (rotated per axis)
  Mesh digitMesh_[4]; // glyphs for "1".."4", billboarded over each motor
  Mesh northArrow_;   // body +X arrow (shaft + head), rotates with the body
  Mesh glyphN_;       // "N" glyph billboarded at the arrow tip
  Mesh ring_;         // unit torus for training halo gates (lit, pos+normal)
  Mesh guideArrow_;   // unit solid arrow (lit) pointing drone -> next gate

  // Training course state (gameplay overlay; populated only in training mode).
  QVector<vsim::RingGate> gates_;
  int   trainActive_ = 0;       // index of the gate to aim for next
  bool  trainArrow_  = false;   // draw the guidance arrow this frame
  float glowPhase_   = 0.0f;    // advances per paint for the gate pulse

  // Imported mesh (pos+normal interleaved). hasMesh_ gates body vs mesh.
  Mesh droneMesh_;
  bool hasMesh_   = false;
  bool meshDirty_ = false;
  std::vector<QVector3D> pendingPos_;
  std::vector<QVector3D> pendingNrm_;

  // Imported world mesh (pos+normal interleaved; same deferred-upload path).
  Mesh worldMesh_;
  bool hasWorldMesh_   = false;
  bool worldMeshDirty_ = false;
  std::vector<QVector3D> pendingWorldPos_;
  std::vector<QVector3D> pendingWorldNrm_;
  std::vector<QVector3D> pendingWorldCol_;  // per-vertex RGB (may be empty)

  // Streaming terrain chunks. Heap-allocated Meshes so addresses stay stable in
  // the map (and GL handles aren't moved). Uploads/removals are queued from the
  // UI thread and flushed in paintGL where the GL context is current.
  std::map<qint64, std::unique_ptr<Mesh>> worldChunks_;
  struct PendingChunk { qint64 key; std::vector<float> data; };  // 9 floats/vert
  std::vector<PendingChunk> pendingChunkUploads_;
  std::vector<qint64> pendingChunkRemovals_;
  bool chunksDirty_ = false;
  bool clearAllChunks_ = false;

  // Editable motor layout (defaults mirror the firmware quad geometry).
  std::array<QVector3D, 4> motorPos_ = {
      QVector3D( 0.13f, +0.22f, 0.0f), QVector3D(-0.13f, +0.20f, 0.0f),
      QVector3D(-0.13f, -0.20f, 0.0f), QVector3D( 0.13f, -0.22f, 0.0f)};
  std::array<QVector3D, 4> motorAxis_ = {
      QVector3D(0, 0, -1), QVector3D(0, 0, -1),
      QVector3D(0, 0, -1), QVector3D(0, 0, -1)};
  std::array<int, 4> motorSpin_ = {+1, -1, +1, -1};
  QVector3D comOffset_{0, 0, 0};

  // Gizmo edit state.
  bool   editable_  = true;     // toggled by SimulatorWidget (sim stopped)
  int    selected_  = -1;       // selected motor index, -1 = none
  Tool   tool_      = Tool::None;
  int    axisLock_  = -1;       // -1 = view-plane / default, 0/1/2 = X/Y/Z
  QPoint toolStartMouse_;
  QVector3D startPos_;          // CoM-frame motor pos at tool start
  QVector3D startAxis_;         // unit thrust axis at tool start
  QVector3D startWorld_;        // world motor pos at tool start
  QVector3D planeHit0_;         // world drag-plane hit at tool start

  // Obstacle gizmo edit state (World mode).
  bool   obsMode_  = false;
  int    selObs_   = -1;
  Tool   obsTool_  = Tool::None;
  int    obsAxis_  = -1;
  QPoint obsStartMouse_;
  QVector3D obsStartPos_, obsStartSize_, obsStartRot_;
  QVector3D obsPlaneHit0_;

  // Camera state (orbit around drone).
  float cam_radius_ = 4.0f;
  float cam_yaw_    = 0.7f;   // orbit azimuth RELATIVE to the drone heading
  float cam_pitch_  = 0.5f;   // tilt; >0 = eye above ground looking down (NED)
  bool  fpv_        = false;  // onboard FPV camera vs orbit
  bool  downCam_    = false;  // belly-mounted downward camera (Down-Cam PiP)
  float downCamOffset_ = 0.05f;  // metres below the CoM the belly lens sits
  bool  freeFly_    = false;  // WASD free-roam camera (sim stopped, World mode)
  bool  worldVisible_ = true; // draw world mesh + obstacles (World mode)
  QVector3D camPos_{-4.0f, -4.0f, -3.0f};  // free-cam world position (NED)
  QPoint last_mouse_;

  QMatrix4x4 proj_;
  vsim::SimSnapshot snap_;
};

}  // namespace vsim
