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

 signals:
  void worldApplied();        // user committed env → push to daemon + persist
  void obstaclesChanged();    // obstacle list/edit changed → preview + persist

 private slots:
  void onApply();
  void onAddObstacle(int type);
  void onRemoveObstacle();
  void onObstacleSelected(int row);
  void onObstacleFieldChanged();

 private:
  void buildUi();
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

  // Obstacle editor.
  QListWidget* obsList_ = nullptr;
  QDoubleSpinBox* obsPos_[3] = {nullptr, nullptr, nullptr};
  QDoubleSpinBox* obsSize_[3] = {nullptr, nullptr, nullptr};
  QDoubleSpinBox* obsRot_[3] = {nullptr, nullptr, nullptr};
  QDoubleSpinBox* obsRest_ = nullptr;
  bool obsSyncing_ = false;        // guard form-sync from re-emitting
};
