#include "WorldEditorWidget.h"

#include "../../core/ui/Buttons.h"
#include "CollapsibleSection.h"

#include <QAbstractSpinBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QVBoxLayout>

namespace {
QDoubleSpinBox* spin(double lo, double hi, int decimals, double step,
                     double val, const QString& suffix = QString()) {
  auto* s = new QDoubleSpinBox();
  s->setRange(lo, hi);
  s->setDecimals(decimals);
  s->setSingleStep(step);
  s->setValue(val);
  if (!suffix.isEmpty()) s->setSuffix(suffix);
  s->setButtonSymbols(QAbstractSpinBox::NoButtons);
  return s;
}
}  // namespace

WorldEditorWidget::WorldEditorWidget(QWidget* parent) : QWidget(parent) {
  buildUi();
  syncConfigToUi();
}

void WorldEditorWidget::buildUi() {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(6);

  // -- Environment --
  {
    auto* sec = new CollapsibleSection(tr("Environment"), this);
    auto* body = new QWidget();
    auto* form = new QFormLayout(body);
    gravity_ = spin(0.0, 30.0, 2, 0.1, cfg_.gravity, QStringLiteral(" m/s²"));
    groundZ_ = spin(-50.0, 50.0, 2, 0.1, cfg_.ground_z, QStringLiteral(" m"));
    rest_    = spin(0.0, 1.0, 2, 0.05, cfg_.restitution);
    form->addRow(tr("Gravity:"), gravity_);
    form->addRow(tr("Ground Z (NED):"), groundZ_);
    form->addRow(tr("Restitution:"), rest_);
    sec->setContentWidget(body);
    root->addWidget(sec);
  }

  // -- Aerodynamics --
  {
    auto* sec = new CollapsibleSection(tr("Aerodynamics"), this);
    auto* body = new QWidget();
    auto* form = new QFormLayout(body);
    linDrag_ = spin(0.0, 5.0, 3, 0.01, cfg_.linear_drag, QStringLiteral(" N·s/m"));
    angDrag_ = spin(0.0, 1.0, 4, 0.001, cfg_.angular_drag, QStringLiteral(" N·m·s"));
    form->addRow(tr("Linear drag:"), linDrag_);
    form->addRow(tr("Angular drag:"), angDrag_);
    sec->setContentWidget(body);
    root->addWidget(sec);
  }

  // -- Apply --
  {
    auto* row = new QHBoxLayout();
    row->addStretch();
    auto* apply = new ui::SuccessButton(tr("Apply to Sim"), this);
    apply->setToolTip(tr("Push environment + aerodynamics to the simulator."));
    row->addWidget(apply);
    root->addLayout(row);
    connect(apply, &QPushButton::clicked, this, &WorldEditorWidget::onApply);
  }

  // -- Obstacles --
  buildObstacleSection(root);

  root->addStretch();
}

void WorldEditorWidget::buildObstacleSection(QVBoxLayout* root) {
  auto* sec = new CollapsibleSection(tr("Obstacles"), this);
  auto* body = new QWidget();
  auto* col = new QVBoxLayout(body);

  obsList_ = new QListWidget();
  obsList_->setMaximumHeight(120);
  obsList_->setToolTip(tr("Static world shapes. Drone flies through them for "
                          "now (collision is a later phase)."));
  connect(obsList_, &QListWidget::currentRowChanged, this,
          &WorldEditorWidget::onObstacleSelected);
  col->addWidget(obsList_);

  {
    auto* btns = new QHBoxLayout();
    auto* addBox = new ui::GhostButton(tr("+ Box"));
    auto* addSph = new ui::GhostButton(tr("+ Sphere"));
    auto* addCyl = new ui::GhostButton(tr("+ Cyl"));
    auto* del = new ui::DangerButton(tr("Remove"));
    connect(addBox, &QPushButton::clicked, this,
            [this] { onAddObstacle(vsim::Obstacle::Box); });
    connect(addSph, &QPushButton::clicked, this,
            [this] { onAddObstacle(vsim::Obstacle::Sphere); });
    connect(addCyl, &QPushButton::clicked, this,
            [this] { onAddObstacle(vsim::Obstacle::Cylinder); });
    connect(del, &QPushButton::clicked, this,
            &WorldEditorWidget::onRemoveObstacle);
    btns->addWidget(addBox);
    btns->addWidget(addSph);
    btns->addWidget(addCyl);
    btns->addStretch();
    btns->addWidget(del);
    col->addLayout(btns);
  }

  // Per-obstacle edit grid. size: box=full extents (X,Y,Z); sphere=radius is
  // X; cylinder=radius X, height Z. pos/rot in the NED world frame.
  auto* grid = new QGridLayout();
  grid->setHorizontalSpacing(4);
  grid->setVerticalSpacing(2);
  auto field = [&](int row, const QString& label, QDoubleSpinBox** out,
                   int n) {
    grid->addWidget(new QLabel(label), row, 0);
    for (int i = 0; i < n; ++i) {
      out[i] = spin(-100.0, 100.0, 3, 0.1, 0.0);
      connect(out[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
              [this] { onObstacleFieldChanged(); });
      grid->addWidget(out[i], row, 1 + i);
    }
  };
  field(0, tr("Pos (m):"), obsPos_, 3);
  field(1, tr("Size (m):"), obsSize_, 3);
  for (auto* s : obsSize_) s->setRange(0.05, 100.0);
  field(2, tr("Rot (°):"), obsRot_, 3);
  for (auto* s : obsRot_) s->setRange(-180.0, 180.0);
  grid->addWidget(new QLabel(tr("Restit.:")), 3, 0);
  obsRest_ = spin(0.0, 1.0, 2, 0.05, 0.3);
  connect(obsRest_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [this] { onObstacleFieldChanged(); });
  grid->addWidget(obsRest_, 3, 1);
  col->addLayout(grid);

  sec->setContentWidget(body);
  root->addWidget(sec);

  refreshObstacleList();
  syncFormFromSelection();
}

void WorldEditorWidget::onAddObstacle(int type) {
  vsim::Obstacle o;
  o.type = type;
  if (type == vsim::Obstacle::Sphere) o.size = {0.5f, 0.5f, 0.5f};
  if (type == vsim::Obstacle::Cylinder) o.size = {0.4f, 0.4f, 1.0f};
  cfg_.obstacles.push_back(o);
  refreshObstacleList();
  if (obsList_) obsList_->setCurrentRow(cfg_.obstacles.size() - 1);
  emit obstaclesChanged();
}

void WorldEditorWidget::onRemoveObstacle() {
  const int r = obsList_ ? obsList_->currentRow() : -1;
  if (r < 0 || r >= cfg_.obstacles.size()) return;
  cfg_.obstacles.remove(r);
  refreshObstacleList();
  emit obstaclesChanged();
}

void WorldEditorWidget::onObstacleSelected(int) { syncFormFromSelection(); }

void WorldEditorWidget::onObstacleFieldChanged() {
  if (obsSyncing_) return;
  const int r = obsList_ ? obsList_->currentRow() : -1;
  if (r < 0 || r >= cfg_.obstacles.size()) return;
  vsim::Obstacle& o = cfg_.obstacles[r];
  o.pos = QVector3D(obsPos_[0]->value(), obsPos_[1]->value(), obsPos_[2]->value());
  o.size = QVector3D(obsSize_[0]->value(), obsSize_[1]->value(), obsSize_[2]->value());
  o.rotate = QVector3D(obsRot_[0]->value(), obsRot_[1]->value(), obsRot_[2]->value());
  o.restitution = static_cast<float>(obsRest_->value());
  refreshObstacleList();
  if (obsList_) obsList_->setCurrentRow(r);  // refresh keeps selection
  emit obstaclesChanged();
}

void WorldEditorWidget::refreshObstacleList() {
  if (!obsList_) return;
  const int keep = obsList_->currentRow();
  QSignalBlocker b(obsList_);
  obsList_->clear();
  static const char* kName[] = {"Box", "Sphere", "Cylinder"};
  for (const vsim::Obstacle& o : cfg_.obstacles) {
    const int t = (o.type >= 0 && o.type <= 2) ? o.type : 0;
    obsList_->addItem(QString("%1  (%2, %3, %4)")
                          .arg(kName[t])
                          .arg(o.pos.x(), 0, 'f', 1)
                          .arg(o.pos.y(), 0, 'f', 1)
                          .arg(o.pos.z(), 0, 'f', 1));
  }
  if (keep >= 0 && keep < cfg_.obstacles.size()) obsList_->setCurrentRow(keep);
}

void WorldEditorWidget::syncFormFromSelection() {
  const int r = obsList_ ? obsList_->currentRow() : -1;
  const bool ok = (r >= 0 && r < cfg_.obstacles.size());
  obsSyncing_ = true;
  if (ok) {
    const vsim::Obstacle& o = cfg_.obstacles[r];
    const QVector3D pv[3] = {o.pos, o.size, o.rotate};
    QDoubleSpinBox** grp[3] = {obsPos_, obsSize_, obsRot_};
    for (int g = 0; g < 3; ++g)
      for (int i = 0; i < 3; ++i)
        grp[g][i]->setValue(pv[g][i]);
    obsRest_->setValue(o.restitution);
  }
  for (int i = 0; i < 3; ++i) {
    obsPos_[i]->setEnabled(ok);
    obsSize_[i]->setEnabled(ok);
    obsRot_[i]->setEnabled(ok);
  }
  obsRest_->setEnabled(ok);
  obsSyncing_ = false;
}

void WorldEditorWidget::syncConfigToUi() {
  gravity_->setValue(cfg_.gravity);
  groundZ_->setValue(cfg_.ground_z);
  rest_->setValue(cfg_.restitution);
  linDrag_->setValue(cfg_.linear_drag);
  angDrag_->setValue(cfg_.angular_drag);
}

void WorldEditorWidget::syncUiToConfig() {
  cfg_.gravity = static_cast<float>(gravity_->value());
  cfg_.ground_z = static_cast<float>(groundZ_->value());
  cfg_.restitution = static_cast<float>(rest_->value());
  cfg_.linear_drag = static_cast<float>(linDrag_->value());
  cfg_.angular_drag = static_cast<float>(angDrag_->value());
}

void WorldEditorWidget::onApply() {
  syncUiToConfig();
  emit worldApplied();
}

void WorldEditorWidget::setConfig(const vsim::WorldConfig& c) {
  cfg_ = c;
  syncConfigToUi();
  refreshObstacleList();
  syncFormFromSelection();
}
