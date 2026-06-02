#pragma once

#include "../../vsim/VsimTypes.h"

#include <QWidget>

class QComboBox;
class QDoubleSpinBox;
class QListWidget;
class QVBoxLayout;

// WorldEditorWidget — the "World" tab's environment + aerodynamics, plus a
// list of static 3D obstacles (boxes/spheres/cylinders). Apply pushes a
// WorldConfig to the daemon; obstacle edits emit obstaclesChanged() live so
// the renderer + persistence stay in sync without an explicit Apply.
class WorldEditorWidget : public QWidget {
  Q_OBJECT
 public:
  explicit WorldEditorWidget(QWidget* parent = nullptr);

  const vsim::WorldConfig& config() const { return cfg_; }
  void setConfig(const vsim::WorldConfig& c);

  // Apply a 3D-gizmo edit of obstacle `index` (world pos/size/rotate).
  void setObstacleFromGizmo(int index, QVector3D pos, QVector3D size,
                            QVector3D rotate);
  // Select an obstacle row (driven by a click in the 3D view).
  void selectObstacleRow(int index);

 signals:
  void worldApplied();        // user committed env → push to daemon + persist
  void obstaclesChanged();    // obstacle list/edit changed → preview + persist
  void obstacleSelectionChanged(int index);  // list selection → 3D highlight
  void worldMeshChanged();    // imported mesh path/scale/up-axis changed

 private slots:
  void onApply();
  void onImportWorldMesh();
  void onClearWorldMesh();
  void onAddObstacle(int type);
  void onRemoveObstacle();
  void onObstacleSelected(int row);
  void onObstacleFieldChanged();
  void onSaveWorld();   // export env + obstacles to a portable .vworld (JSON)
  void onLoadWorld();   // import a .vworld

 private:
  void buildUi();
  void buildWorldMeshSection(QVBoxLayout* root);
  void buildObstacleSection(QVBoxLayout* root);
  void syncConfigToUi();
  void syncUiToConfig();
  void refreshObstacleList();      // list labels from cfg_.obstacles
  void syncFormFromSelection();    // selected obstacle → form fields

  vsim::WorldConfig cfg_;

  QDoubleSpinBox* gravity_ = nullptr;
  QDoubleSpinBox* groundZ_ = nullptr;
  QDoubleSpinBox* rest_ = nullptr;
  QDoubleSpinBox* linDrag_ = nullptr;
  QDoubleSpinBox* angDrag_ = nullptr;
  QDoubleSpinBox* rightGain_ = nullptr;  // ground-contact righting gain
  QDoubleSpinBox* rightDamp_ = nullptr;  // righting damping

  // Obstacle editor.
  QListWidget* obsList_ = nullptr;
  QDoubleSpinBox* obsPos_[3] = {nullptr, nullptr, nullptr};
  QDoubleSpinBox* obsSize_[3] = {nullptr, nullptr, nullptr};
  QDoubleSpinBox* obsRot_[3] = {nullptr, nullptr, nullptr};
  QDoubleSpinBox* obsRest_ = nullptr;
  bool obsSyncing_ = false;        // guard form-sync from re-emitting
  class QLabel* fileStatus_ = nullptr;  // save/load feedback

  // World-mesh import controls.
  class QLabel* worldMeshLabel_ = nullptr;
  QDoubleSpinBox* worldScale_ = nullptr;
  QComboBox* worldUpAxis_ = nullptr;
  bool worldMeshSyncing_ = false;
};
