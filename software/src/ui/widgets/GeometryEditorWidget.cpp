#include "GeometryEditorWidget.h"

#include "../../core/Theme.h"
#include "../../core/ui/Buttons.h"
#include "../../vsim/MassProperties.h"
#include "../../vsim/MeshLoader.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

#include <cmath>

namespace {

QDoubleSpinBox* spin(double lo, double hi, int decimals, double step,
                     double val, const QString& suffix = QString()) {
  auto* s = new QDoubleSpinBox();
  s->setRange(lo, hi);
  s->setDecimals(decimals);
  s->setSingleStep(step);
  s->setValue(val);
  if (!suffix.isEmpty()) s->setSuffix(suffix);
  s->setMaximumWidth(78);
  s->setButtonSymbols(QAbstractSpinBox::NoButtons);
  return s;
}

QLabel* gridHeader(const QString& t) {
  auto* l = new QLabel(t);
  l->setStyleSheet(QString("color:%1; font-size:11px;")
                       .arg(Theme::hex(Theme::kTextMuted)));
  return l;
}

}  // namespace

GeometryEditorWidget::GeometryEditorWidget(QWidget* parent) : QWidget(parent) {
  buildUi();
  syncConfigToUi();
  showMassProps();
}

void GeometryEditorWidget::buildUi() {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);

  // -- Mesh + mass --
  {
    auto* g = new QGroupBox(tr("Airframe geometry"), this);
    auto* v = new QVBoxLayout(g);

    auto* meshRow = new QHBoxLayout();
    meshRow->addWidget(new QLabel(tr("Mesh:"), g));
    meshEdit_ = new QLineEdit(g);
    meshEdit_->setPlaceholderText(tr("STL / OBJ / PLY / glTF…"));
    meshRow->addWidget(meshEdit_, 1);
    auto* browse = new ui::GhostButton(tr("Browse"), g);
    meshRow->addWidget(browse);
    v->addLayout(meshRow);

    auto* numRow = new QHBoxLayout();
    numRow->addWidget(new QLabel(tr("Scale (m/unit):"), g));
    scaleSpin_ = spin(1e-4, 1e4, 4, 0.1, 1.0);
    numRow->addWidget(scaleSpin_);
    numRow->addSpacing(12);
    numRow->addWidget(new QLabel(tr("Mass:"), g));
    massSpin_ = spin(0.01, 100.0, 3, 0.05, 1.0, QStringLiteral(" kg"));
    numRow->addWidget(massSpin_);
    numRow->addStretch();
    auto* compute = new ui::PrimaryButton(tr("Load && Compute"), g);
    numRow->addWidget(compute);
    v->addLayout(numRow);

    statusLbl_ = new QLabel(tr("No mesh loaded — using default box."), g);
    statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kTextMuted)));
    statusLbl_->setWordWrap(true);
    v->addWidget(statusLbl_);

    comLbl_ = new QLabel(g);
    comLbl_->setStyleSheet("font-family:monospace;");
    v->addWidget(comLbl_);

    inertiaLbl_ = new QLabel(g);
    inertiaLbl_->setStyleSheet(QString("font-family:monospace; color:%1;")
                                   .arg(Theme::hex(Theme::kAccent)));
    v->addWidget(inertiaLbl_);

    root->addWidget(g);

    connect(browse, &QPushButton::clicked, this, &GeometryEditorWidget::onBrowse);
    connect(compute, &QPushButton::clicked, this, &GeometryEditorWidget::onCompute);
  }

  // -- Motor grid --
  {
    auto* g = new QGroupBox(tr("Motors (body frame)"), this);
    auto* grid = new QGridLayout(g);
    grid->setHorizontalSpacing(4);
    grid->setVerticalSpacing(3);

    const char* heads[] = {"", "x", "y", "z", "ax", "ay", "az",
                           "spin", "kT", "kM", "ω max"};
    for (int c = 0; c < 11; ++c) grid->addWidget(gridHeader(QString::fromUtf8(heads[c])), 0, c);

    for (int i = 0; i < 4; ++i) {
      auto& r = rows_[i];
      grid->addWidget(new QLabel(QString("M%1").arg(i + 1), g), i + 1, 0);
      r.px = spin(-2, 2, 3, 0.005, 0, QStringLiteral(" m"));
      r.py = spin(-2, 2, 3, 0.005, 0, QStringLiteral(" m"));
      r.pz = spin(-2, 2, 3, 0.005, 0, QStringLiteral(" m"));
      r.ax = spin(-1, 1, 3, 0.05, 0);
      r.ay = spin(-1, 1, 3, 0.05, 0);
      r.az = spin(-1, 1, 3, 0.05, -1);
      r.spin = new QComboBox(g);
      r.spin->addItem(tr("CCW +"), +1);
      r.spin->addItem(tr("CW −"), -1);
      r.spin->setMaximumWidth(78);
      r.kt = spin(0, 1, 8, 1e-6, 1.522e-5);
      r.km = spin(0, 1, 9, 1e-7, 2.44e-7);
      r.wmax = spin(0, 5000, 0, 50, 1200, QStringLiteral(" r/s"));
      QWidget* cells[] = {r.px, r.py, r.pz, r.ax, r.ay, r.az,
                          r.spin, r.kt, r.km, r.wmax};
      for (int c = 0; c < 10; ++c) grid->addWidget(cells[c], i + 1, c + 1);
    }
    root->addWidget(g);
  }

  // -- Apply --
  {
    auto* row = new QHBoxLayout();
    row->addStretch();
    auto* apply = new ui::SuccessButton(tr("Apply to Sim"), this);
    apply->setToolTip(tr("Push mass, inertia, and motor layout to the running simulator."));
    row->addWidget(apply);
    root->addLayout(row);
    connect(apply, &QPushButton::clicked, this, &GeometryEditorWidget::onApply);
  }
}

void GeometryEditorWidget::syncConfigToUi() {
  meshEdit_->setText(cfg_.meshPath);
  scaleSpin_->setValue(cfg_.scale);
  massSpin_->setValue(cfg_.mass);
  for (int i = 0; i < 4; ++i) {
    const auto& m = cfg_.motors[i];
    auto& r = rows_[i];
    r.px->setValue(m.pos.x()); r.py->setValue(m.pos.y()); r.pz->setValue(m.pos.z());
    r.ax->setValue(m.axis.x()); r.ay->setValue(m.axis.y()); r.az->setValue(m.axis.z());
    r.spin->setCurrentIndex(m.spin >= 0 ? 0 : 1);
    r.kt->setValue(m.k_thrust); r.km->setValue(m.k_moment); r.wmax->setValue(m.max_omega);
  }
}

void GeometryEditorWidget::syncUiToConfig() {
  cfg_.meshPath = meshEdit_->text();
  cfg_.scale = static_cast<float>(scaleSpin_->value());
  cfg_.mass = static_cast<float>(massSpin_->value());
  for (int i = 0; i < 4; ++i) {
    auto& m = cfg_.motors[i];
    const auto& r = rows_[i];
    m.pos = QVector3D(r.px->value(), r.py->value(), r.pz->value());
    m.axis = QVector3D(r.ax->value(), r.ay->value(), r.az->value());
    m.spin = r.spin->currentData().toInt();
    m.k_thrust = static_cast<float>(r.kt->value());
    m.k_moment = static_cast<float>(r.km->value());
    m.max_omega = static_cast<float>(r.wmax->value());
  }
}

void GeometryEditorWidget::showMassProps() {
  comLbl_->setText(tr("CoM: (%1, %2, %3) mm")
                       .arg(cfg_.com.x() * 1000.0, 0, 'f', 1)
                       .arg(cfg_.com.y() * 1000.0, 0, 'f', 1)
                       .arg(cfg_.com.z() * 1000.0, 0, 'f', 1));
  const auto& I = cfg_.inertia;
  inertiaLbl_->setText(
      QString("I [kg·m²] =\n"
              " %1  %2  %3\n %4  %5  %6\n %7  %8  %9")
          .arg(I[0], 9, 'g', 3).arg(I[1], 9, 'g', 3).arg(I[2], 9, 'g', 3)
          .arg(I[3], 9, 'g', 3).arg(I[4], 9, 'g', 3).arg(I[5], 9, 'g', 3)
          .arg(I[6], 9, 'g', 3).arg(I[7], 9, 'g', 3).arg(I[8], 9, 'g', 3));
}

bool GeometryEditorWidget::loadAndCompute(QString* err) {
  syncUiToConfig();
  vsim::LoadedMesh mesh = vsim::loadMesh(cfg_.meshPath, cfg_.scale, err);
  if (!mesh.valid) return false;

  vsim::MassProperties mp =
      vsim::computeMassProperties(mesh.positions, cfg_.mass);
  if (!mp.valid) {
    if (err) *err = tr("mesh is not a closed solid (zero volume)");
    return false;
  }

  meshPos_ = mesh.positions;
  meshNrm_ = mesh.normals;
  cfg_.com = mp.com;
  // Row-major symmetric tensor into the 9-float config.
  cfg_.inertia = {mp.ixx, mp.ixy, mp.ixz,
                  mp.ixy, mp.iyy, mp.iyz,
                  mp.ixz, mp.iyz, mp.izz};
  return true;
}

void GeometryEditorWidget::onBrowse() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Select airframe mesh"), QFileInfo(meshEdit_->text()).absolutePath(),
      tr("Meshes (*.stl *.obj *.ply *.glb *.gltf *.dae *.fbx);;All files (*)"));
  if (!path.isEmpty()) meshEdit_->setText(path);
}

void GeometryEditorWidget::onCompute() {
  QString err;
  if (!loadAndCompute(&err)) {
    statusLbl_->setText(tr("Load failed: %1").arg(err));
    statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kDanger)));
    return;
  }
  const double comMag = cfg_.com.length();
  statusLbl_->setText(tr("Loaded %1 triangles.").arg(meshPos_.size() / 3));
  // Warn if the model origin is far from the CoM — physics rotates about
  // the body origin, so a large offset means the sim won't match reality.
  if (comMag > 0.02) {
    statusLbl_->setText(statusLbl_->text() +
                        tr("  ⚠ CoM is %1 mm from origin — recenter the model.")
                            .arg(comMag * 1000.0, 0, 'f', 0));
    statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kWarn)));
  } else {
    statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kOk)));
  }
  showMassProps();
  emit previewUpdated();
}

void GeometryEditorWidget::onApply() {
  syncUiToConfig();
  emit geometryApplied();
}

void GeometryEditorWidget::setConfig(const vsim::GeometryConfig& c) {
  cfg_ = c;
  syncConfigToUi();
  // If the persisted mesh still resolves, load it so the preview + inertia
  // are live without making the user click Compute again.
  if (!cfg_.meshPath.isEmpty() && QFileInfo::exists(cfg_.meshPath)) {
    QString err;
    if (loadAndCompute(&err)) {
      statusLbl_->setText(tr("Loaded %1 triangles.").arg(meshPos_.size() / 3));
      statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kOk)));
      emit previewUpdated();
    }
  }
  showMassProps();
}
