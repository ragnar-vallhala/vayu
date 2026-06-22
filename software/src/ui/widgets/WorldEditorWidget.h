#pragma once

#include "../../vsim/VsimTypes.h"

#include <QWidget>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QListWidget;
class QPushButton;
class QSpinBox;
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

  // Wind & turbulence (its own staged Apply button, mockup World tab).
  const vsim::WindConfig& windConfig() const { return wind_; }
  void setWindConfig(const vsim::WindConfig& w);
  // Live wind readout fed from the pose stream (instantaneous speed + heading).
  void setWindReadout(float speedMs, float dirDeg);

  // Apply a 3D-gizmo edit of obstacle `index` (world pos/size/rotate).
  void setObstacleFromGizmo(int index, QVector3D pos, QVector3D size,
                            QVector3D rotate);
  // Select an obstacle row (driven by a click in the 3D view).
  void selectObstacleRow(int index);

 signals:
  void worldApplied();        // user committed env → push to daemon + persist
  void windApplied();         // user committed wind → push to daemon + persist
  void obstaclesChanged();    // obstacle list/edit changed → preview + persist
  void obstacleSelectionChanged(int index);  // list selection → 3D highlight
  void worldMeshChanged();    // imported mesh path/scale/up-axis changed

 private slots:
  void onApply();
  void onApplyWind();
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
  void buildWindSection(QVBoxLayout* root);
  void buildProceduralSection(QVBoxLayout* root);
  void buildWorldMeshSection(QVBoxLayout* root);
  void buildObstacleSection(QVBoxLayout* root);
  void syncConfigToUi();
  void syncUiToConfig();
  void syncWindToUi();             // wind_ -> spinboxes/checkbox
  void syncUiToWind();             // spinboxes/checkbox -> wind_
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

  // Wind & turbulence.
  vsim::WindConfig wind_;
  QDoubleSpinBox* windN_ = nullptr;
  QDoubleSpinBox* windE_ = nullptr;
  QDoubleSpinBox* windD_ = nullptr;
  QDoubleSpinBox* gustAmp_ = nullptr;
  QDoubleSpinBox* gustPeriod_ = nullptr;
  QDoubleSpinBox* turbSigma_ = nullptr;
  QCheckBox* windEnable_ = nullptr;
  QPushButton* windApplyBtn_ = nullptr;   // enabled only when dirty
  class QLabel* windReadout_ = nullptr;   // live speed/dir from the pose stream
  bool windSyncing_ = false;              // guard sync from marking dirty

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
  QDoubleSpinBox* worldOffset_[3] = {nullptr, nullptr, nullptr};  // NED placement
  bool worldMeshSyncing_ = false;

  // Procedural world controls (Phase 0). Biome "None" = off (use imported
  // mesh); selecting a biome generates and takes precedence.
  QComboBox* procBiome_ = nullptr;
  QSpinBox* procSeed_ = nullptr;
  QDoubleSpinBox* procSizeM_ = nullptr;
  QSpinBox* procResolution_ = nullptr;
  bool procSyncing_ = false;
};
