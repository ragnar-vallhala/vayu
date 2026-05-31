#include "WorldEditorWidget.h"

#include "../../core/ui/Buttons.h"
#include "CollapsibleSection.h"

#include <QAbstractSpinBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
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

  root->addStretch();
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
}
