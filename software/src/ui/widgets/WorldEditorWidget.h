#pragma once

#include "../../vsim/VsimTypes.h"

#include <QWidget>

class QDoubleSpinBox;

// WorldEditorWidget — the "World" tab's environment + aerodynamics
// properties (gravity, ground plane, restitution, drag). Categorized into
// collapsible sections; Apply pushes a WorldConfig to the daemon via
// SimWorker::sendWorld and persists it.
class WorldEditorWidget : public QWidget {
  Q_OBJECT
 public:
  explicit WorldEditorWidget(QWidget* parent = nullptr);

  const vsim::WorldConfig& config() const { return cfg_; }
  void setConfig(const vsim::WorldConfig& c);

 signals:
  void worldApplied();   // user committed → push to daemon + persist

 private slots:
  void onApply();

 private:
  void buildUi();
  void syncConfigToUi();
  void syncUiToConfig();

  vsim::WorldConfig cfg_;

  QDoubleSpinBox* gravity_ = nullptr;
  QDoubleSpinBox* groundZ_ = nullptr;
  QDoubleSpinBox* rest_ = nullptr;
  QDoubleSpinBox* linDrag_ = nullptr;
  QDoubleSpinBox* angDrag_ = nullptr;
};
