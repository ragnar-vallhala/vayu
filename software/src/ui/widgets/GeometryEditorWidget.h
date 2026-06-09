#pragma once

#include "../../vsim/VsimTypes.h"

#include <QMatrix4x4>
#include <QVector3D>
#include <QWidget>

#include <array>
#include <vector>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;

// GeometryEditorWidget — Gazebo-style airframe setup for the in-app sim.
// Pick a mesh (STL/OBJ/PLY/glTF), compute its mass properties (CoM + full
// inertia tensor) at a target mass, and edit the 4-motor layout (body
// position, thrust axis, spin, coefficients) in a numeric grid.
//
// Emits previewUpdated() when the loaded mesh / CoM / motor layout change
// (so the SimulatorWidget can refresh the 3D preview) and geometryApplied()
// when the user commits (so the SimulatorWidget pushes it to the daemon
// and persists it).
class GeometryEditorWidget : public QWidget {
  Q_OBJECT
 public:
  explicit GeometryEditorWidget(QWidget* parent = nullptr);

  const vsim::GeometryConfig& config() const { return cfg_; }
  // File name of the last loaded/saved .vveh vehicle (empty if none), so the
  // autotuner can show which vehicle it's tuning.
  QString loadedVehicleName() const { return loadedName_; }
  // Body placement transform (translate + XYZ-Euler rotate, degrees) applied
  // to the imported mesh in loadAndCompute(). The SAME transform must be
  // applied to the motor layout so the motors stay rigidly attached to the
  // airframe when it is moved/rotated.
  QMatrix4x4 bodyXform() const {
    QMatrix4x4 x;
    x.translate(cfg_.translate);
    x.rotate(cfg_.rotate.x(), 1, 0, 0);
    x.rotate(cfg_.rotate.y(), 0, 1, 0);
    x.rotate(cfg_.rotate.z(), 0, 0, 1);
    return x;
  }

  // The config expressed about the center of mass: the motors are run through
  // the body placement transform (so they move/rotate WITH the mesh) and then
  // shifted to be CoM-relative, with com zeroed. This is what the daemon +
  // renderer consume, so ALL dynamics (inertia, torque arms, the tracked
  // point) and the drawn rotor markers are rigidly attached to the airframe
  // about its CoM. cfg_ itself stays in the user's model-origin frame for
  // display/persistence; the mesh (meshPositions()) is already transformed +
  // recentered on the CoM in loadAndCompute().
  vsim::GeometryConfig physicsConfig() const {
    vsim::GeometryConfig c = cfg_;
    const QMatrix4x4 x = bodyXform();
    for (auto& m : c.motors) {
      m.pos  = x.map(m.pos) - cfg_.com;            // placed, then CoM-relative
      m.axis = x.mapVector(m.axis).normalized();   // rotate the thrust axis too
    }
    c.translate = QVector3D(0, 0, 0);
    c.rotate    = QVector3D(0, 0, 0);
    c.com       = QVector3D(0, 0, 0);
    return c;
  }
  // Load a persisted config: populates the form and, if the mesh path
  // still resolves, loads + recomputes so the preview is ready.
  void setConfig(const vsim::GeometryConfig& c);

  const std::vector<QVector3D>& meshPositions() const { return meshPos_; }
  const std::vector<QVector3D>& meshNormals() const { return meshNrm_; }
  bool hasMesh() const { return !meshPos_.empty(); }

 public slots:
  // Apply a gizmo edit from the 3D view. posComFrame is CoM-frame (as the
  // renderer works in); converted back to the model-origin frame here.
  void setMotorFromGizmo(int index, QVector3D posComFrame, QVector3D axis);

 signals:
  void previewUpdated();    // mesh / CoM / motors changed → refresh preview
  void geometryApplied();   // user committed → push to daemon + persist

 private slots:
  void onBrowse();
  void onCompute();
  void onApply();
  void onSaveFile();   // export the vehicle to a portable .json
  void onLoadFile();   // import a vehicle .json

 private:
  void buildUi();
  void syncConfigToUi();    // cfg_ → widgets
  void syncUiToConfig();    // widgets → cfg_ (mass, scale, motors)
  void showMassProps();     // refresh inertia / CoM labels from cfg_
  bool loadAndCompute(QString* err);  // load mesh + recompute inertia into cfg_

  vsim::GeometryConfig cfg_;
  QString loadedName_;        // last loaded/saved .vveh file name (for display)
  std::vector<QVector3D> meshPos_;
  std::vector<QVector3D> meshNrm_;

  QLineEdit* meshEdit_ = nullptr;
  QDoubleSpinBox* scaleSpin_ = nullptr;
  QDoubleSpinBox* massSpin_ = nullptr;
  QDoubleSpinBox* transX_ = nullptr; QDoubleSpinBox* transY_ = nullptr; QDoubleSpinBox* transZ_ = nullptr;
  QDoubleSpinBox* rotX_ = nullptr;   QDoubleSpinBox* rotY_ = nullptr;   QDoubleSpinBox* rotZ_ = nullptr;
  QLabel* inertiaLbl_ = nullptr;
  QLabel* comLbl_ = nullptr;
  QLabel* statusLbl_ = nullptr;

  struct MotorRow {
    QDoubleSpinBox* px = nullptr; QDoubleSpinBox* py = nullptr; QDoubleSpinBox* pz = nullptr;
    QDoubleSpinBox* ax = nullptr; QDoubleSpinBox* ay = nullptr; QDoubleSpinBox* az = nullptr;
    QComboBox* spin = nullptr;
    QDoubleSpinBox* kt = nullptr; QDoubleSpinBox* km = nullptr; QDoubleSpinBox* wmax = nullptr;
  };
  std::array<MotorRow, 4> rows_{};
};
