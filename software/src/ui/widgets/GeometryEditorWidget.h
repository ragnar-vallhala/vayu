#pragma once

#include "../../vsim/VsimTypes.h"

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
  // The config expressed about the center of mass: motor arms shifted to
  // be CoM-relative and com zeroed. This is what the daemon + renderer
  // consume, so ALL dynamics (inertia, torque arms, the tracked point)
  // are about the CoM. cfg_ itself stays in the user's model-origin frame
  // for display/persistence; the mesh (meshPositions()) is already
  // recentered on the CoM in loadAndCompute().
  vsim::GeometryConfig physicsConfig() const {
    vsim::GeometryConfig c = cfg_;
    for (auto& m : c.motors) m.pos -= cfg_.com;
    c.com = QVector3D(0, 0, 0);
    return c;
  }
  // Load a persisted config: populates the form and, if the mesh path
  // still resolves, loads + recomputes so the preview is ready.
  void setConfig(const vsim::GeometryConfig& c);

  const std::vector<QVector3D>& meshPositions() const { return meshPos_; }
  const std::vector<QVector3D>& meshNormals() const { return meshNrm_; }
  bool hasMesh() const { return !meshPos_.empty(); }

 signals:
  void previewUpdated();    // mesh / CoM / motors changed → refresh preview
  void geometryApplied();   // user committed → push to daemon + persist

 private slots:
  void onBrowse();
  void onCompute();
  void onApply();

 private:
  void buildUi();
  void syncConfigToUi();    // cfg_ → widgets
  void syncUiToConfig();    // widgets → cfg_ (mass, scale, motors)
  void showMassProps();     // refresh inertia / CoM labels from cfg_
  bool loadAndCompute(QString* err);  // load mesh + recompute inertia into cfg_

  vsim::GeometryConfig cfg_;
  std::vector<QVector3D> meshPos_;
  std::vector<QVector3D> meshNrm_;

  QLineEdit* meshEdit_ = nullptr;
  QDoubleSpinBox* scaleSpin_ = nullptr;
  QDoubleSpinBox* massSpin_ = nullptr;
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
