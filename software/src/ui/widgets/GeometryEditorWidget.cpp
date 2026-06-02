#include "GeometryEditorWidget.h"

#include "../../core/Theme.h"
#include "../../core/ui/Buttons.h"
#include "../../vsim/MassProperties.h"
#include "../../vsim/MeshLoader.h"
#include "CollapsibleSection.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMatrix4x4>
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
  s->setButtonSymbols(QAbstractSpinBox::NoButtons);
  return s;
}

// ---- portable vehicle JSON (cross-project import/export) ----
QJsonObject vec3ToJson(const QVector3D& v) {
  return QJsonObject{{"x", v.x()}, {"y", v.y()}, {"z", v.z()}};
}
QVector3D vec3FromJson(const QJsonValue& v, const QVector3D& def = {}) {
  if (!v.isObject()) return def;
  const QJsonObject o = v.toObject();
  return QVector3D(o.value("x").toDouble(def.x()),
                   o.value("y").toDouble(def.y()),
                   o.value("z").toDouble(def.z()));
}

QJsonObject configToJson(const vsim::GeometryConfig& c) {
  QJsonObject root;
  root["format"] = "vayu-vehicle";
  root["version"] = 1;
  root["meshPath"] = c.meshPath;
  root["scale"] = c.scale;
  root["mass"] = c.mass;
  root["translate"] = vec3ToJson(c.translate);
  root["rotate"] = vec3ToJson(c.rotate);
  root["com"] = vec3ToJson(c.com);
  QJsonArray inertia;
  for (float v : c.inertia) inertia.append(v);
  root["inertia"] = inertia;
  QJsonArray motors;
  for (const auto& m : c.motors) {
    motors.append(QJsonObject{{"pos", vec3ToJson(m.pos)},
                              {"axis", vec3ToJson(m.axis)},
                              {"spin", m.spin},
                              {"k_thrust", m.k_thrust},
                              {"k_moment", m.k_moment},
                              {"max_omega", m.max_omega}});
  }
  root["motors"] = motors;
  return root;
}

vsim::GeometryConfig configFromJson(const QJsonObject& root) {
  vsim::GeometryConfig c;   // defaults fill anything missing
  c.meshPath = root.value("meshPath").toString(c.meshPath);
  c.scale = root.value("scale").toDouble(c.scale);
  c.mass = root.value("mass").toDouble(c.mass);
  c.translate = vec3FromJson(root.value("translate"), c.translate);
  c.rotate = vec3FromJson(root.value("rotate"), c.rotate);
  c.com = vec3FromJson(root.value("com"), c.com);
  const QJsonArray inertia = root.value("inertia").toArray();
  for (int i = 0; i < 9 && i < inertia.size(); ++i)
    c.inertia[i] = inertia[i].toDouble(c.inertia[i]);
  const QJsonArray motors = root.value("motors").toArray();
  for (int i = 0; i < 4 && i < motors.size(); ++i) {
    const QJsonObject m = motors[i].toObject();
    auto& mc = c.motors[i];
    mc.pos = vec3FromJson(m.value("pos"), mc.pos);
    mc.axis = vec3FromJson(m.value("axis"), mc.axis);
    mc.spin = m.value("spin").toInt(mc.spin);
    mc.k_thrust = m.value("k_thrust").toDouble(mc.k_thrust);
    mc.k_moment = m.value("k_moment").toDouble(mc.k_moment);
    mc.max_omega = m.value("max_omega").toDouble(mc.max_omega);
  }
  return c;
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

  // -- Save / Load vehicle (portable JSON, cross-project) --
  {
    auto* row = new QHBoxLayout();
    auto* save = new ui::GhostButton(tr("Save Vehicle…"), this);
    save->setToolTip(tr("Export this vehicle (mesh path, transform, mass, "
                        "inertia, motors) to a portable .vveh file."));
    auto* load = new ui::GhostButton(tr("Load Vehicle…"), this);
    load->setToolTip(tr("Import a vehicle .vveh file, e.g. from another project."));
    row->addWidget(save);
    row->addWidget(load);
    row->addStretch();
    root->addLayout(row);
    connect(save, &QPushButton::clicked, this, &GeometryEditorWidget::onSaveFile);
    connect(load, &QPushButton::clicked, this, &GeometryEditorWidget::onLoadFile);
  }

  // -- Mesh, Mass & Inertia --
  {
    auto* sec = new CollapsibleSection(tr("Mesh, Mass & Inertia"), this);
    auto* g = new QWidget();
    auto* v = new QVBoxLayout(g);
    v->setContentsMargins(0, 0, 0, 0);

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

    // Body-frame placement of the mesh: applied (after scale) before mass
    // properties, so the inertia tensor follows the chosen orientation.
    auto triplet = [this, g, v](QDoubleSpinBox** x, QDoubleSpinBox** y,
                                QDoubleSpinBox** z, const QString& label, double lo,
                                double hi, int dec, double step, const QString& sfx) {
      auto* row = new QHBoxLayout();
      row->addWidget(new QLabel(label, g));
      *x = spin(lo, hi, dec, step, 0, sfx);
      *y = spin(lo, hi, dec, step, 0, sfx);
      *z = spin(lo, hi, dec, step, 0, sfx);
      for (auto* s : {*x, *y, *z}) s->setMaximumWidth(70);
      row->addWidget(*x); row->addWidget(*y); row->addWidget(*z);
      row->addStretch();
      v->addLayout(row);
    };
    triplet(&transX_, &transY_, &transZ_, tr("Translate:"), -100, 100, 3, 0.01,
            QStringLiteral(" m"));
    triplet(&rotX_, &rotY_, &rotZ_, tr("Rotate (°):"), -360, 360, 1, 1.0,
            QStringLiteral("°"));

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

    sec->setContentWidget(g);
    root->addWidget(sec);

    connect(browse, &QPushButton::clicked, this, &GeometryEditorWidget::onBrowse);
    connect(compute, &QPushButton::clicked, this, &GeometryEditorWidget::onCompute);
  }

  // -- Motor grid --
  {
    // Gizmo hint, then one collapsible section per rotor with its fields
    // stacked vertically (a form) so the panel never scrolls horizontally.
    auto* hint = new QLabel(
        tr("Tip: in the 3D view (sim stopped) click a motor, then "
           "G = move · R = rotate axis · X/Y/Z lock · click confirm · Esc."),
        this);
    hint->setWordWrap(true);
    hint->setStyleSheet(QString("color:%1; font-size:11px;")
                            .arg(Theme::hex(Theme::kTextMuted)));
    root->addWidget(hint);

    const char* names[4] = {"Motor 1 (FR)", "Motor 2 (RR)", "Motor 3 (RL)",
                            "Motor 4 (FL)"};
    for (int i = 0; i < 4; ++i) {
      auto& r = rows_[i];
      auto* sec = new CollapsibleSection(tr(names[i]), this, /*expanded=*/i == 0);
      auto* body = new QWidget();
      auto* form = new QFormLayout(body);
      form->setContentsMargins(0, 0, 0, 0);
      form->setLabelAlignment(Qt::AlignRight);

      r.px = spin(-2, 2, 3, 0.005, 0, QStringLiteral(" m"));
      r.py = spin(-2, 2, 3, 0.005, 0, QStringLiteral(" m"));
      r.pz = spin(-2, 2, 3, 0.005, 0, QStringLiteral(" m"));
      r.ax = spin(-1, 1, 3, 0.05, 0);
      r.ay = spin(-1, 1, 3, 0.05, 0);
      r.az = spin(-1, 1, 3, 0.05, -1);
      r.spin = new QComboBox();
      r.spin->addItem(tr("CCW (+)"), +1);
      r.spin->addItem(tr("CW (−)"), -1);
      r.kt = spin(0, 1, 8, 1e-6, 1.522e-5);
      r.km = spin(0, 1, 9, 1e-7, 2.44e-7);
      r.wmax = spin(0, 5000, 0, 50, 1200, QStringLiteral(" rad/s"));

      form->addRow(tr("Pos X"), r.px);
      form->addRow(tr("Pos Y"), r.py);
      form->addRow(tr("Pos Z"), r.pz);
      form->addRow(tr("Axis X"), r.ax);
      form->addRow(tr("Axis Y"), r.ay);
      form->addRow(tr("Axis Z"), r.az);
      form->addRow(tr("Spin"), r.spin);
      form->addRow(tr("k_thrust"), r.kt);
      form->addRow(tr("k_moment"), r.km);
      form->addRow(tr("max ω"), r.wmax);

      sec->setContentWidget(body);
      root->addWidget(sec);
    }
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

  // Pack sections to the top; without this the QVBoxLayout spreads the
  // slack between collapsed sections so they sit evenly apart.
  root->addStretch(1);
}

void GeometryEditorWidget::syncConfigToUi() {
  meshEdit_->setText(cfg_.meshPath);
  scaleSpin_->setValue(cfg_.scale);
  massSpin_->setValue(cfg_.mass);
  transX_->setValue(cfg_.translate.x()); transY_->setValue(cfg_.translate.y()); transZ_->setValue(cfg_.translate.z());
  rotX_->setValue(cfg_.rotate.x()); rotY_->setValue(cfg_.rotate.y()); rotZ_->setValue(cfg_.rotate.z());
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
  cfg_.translate = QVector3D(transX_->value(), transY_->value(), transZ_->value());
  cfg_.rotate = QVector3D(rotX_->value(), rotY_->value(), rotZ_->value());
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

void GeometryEditorWidget::setMotorFromGizmo(int i, QVector3D posComFrame,
                                             QVector3D axis) {
  if (i < 0 || i >= 4) return;
  auto& m = cfg_.motors[i];
  // The gizmo works in the rendered CoM/body frame; invert the body placement
  // transform so the stored value is in the raw model-origin frame (the frame
  // physicsConfig() re-applies bodyXform()+CoM to). render = x*raw - com.
  bool ok = false;
  const QMatrix4x4 xi = bodyXform().inverted(&ok);
  const QVector3D placed = posComFrame + cfg_.com;   // CoM frame -> placed frame
  m.pos  = ok ? xi.map(placed)              : placed;
  m.axis = ok ? xi.mapVector(axis).normalized() : axis;
  // Reflect in the row's spinboxes (block in case anything is wired).
  auto& r = rows_[i];
  for (auto* s : {r.px, r.py, r.pz, r.ax, r.ay, r.az}) s->blockSignals(true);
  r.px->setValue(m.pos.x()); r.py->setValue(m.pos.y()); r.pz->setValue(m.pos.z());
  r.ax->setValue(m.axis.x()); r.ay->setValue(m.axis.y()); r.az->setValue(m.axis.z());
  for (auto* s : {r.px, r.py, r.pz, r.ax, r.ay, r.az}) s->blockSignals(false);
  emit previewUpdated();
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

  // Body-frame placement: rotate (XYZ Euler, deg) then translate, on top
  // of the scale already applied by the loader. The SAME transform is applied
  // to the motor layout in physicsConfig() so motors stay attached to the body.
  const QMatrix4x4 xform = bodyXform();
  meshPos_.resize(mesh.positions.size());
  meshNrm_.resize(mesh.normals.size());
  for (size_t i = 0; i < mesh.positions.size(); ++i)
    meshPos_[i] = xform.map(mesh.positions[i]);
  for (size_t i = 0; i < mesh.normals.size(); ++i)
    meshNrm_[i] = xform.mapVector(mesh.normals[i]).normalized();

  vsim::MassProperties mp = vsim::computeMassProperties(meshPos_, cfg_.mass);
  if (!mp.valid) {
    if (err) *err = tr("mesh is not a closed solid (zero volume)");
    return false;
  }

  cfg_.com = mp.com;
  // Recenter the mesh on the CoM so the renderer draws it about the same
  // point the physics integrates. Motor arms get the matching shift via
  // physicsConfig(); the inertia tensor is already about the CoM.
  for (auto& v : meshPos_) v -= mp.com;
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
  // The mesh + motor arms are recentered on the CoM, so the offset is
  // handled, not a problem — report it as information.
  const double comMag = cfg_.com.length();
  statusLbl_->setText(
      tr("Loaded %1 triangles. CoM offset %2 mm from model origin "
         "(handled — dynamics about CoM).")
          .arg(meshPos_.size() / 3)
          .arg(comMag * 1000.0, 0, 'f', 1));
  statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kOk)));
  showMassProps();
  emit previewUpdated();
}

void GeometryEditorWidget::onApply() {
  syncUiToConfig();
  emit geometryApplied();
}

void GeometryEditorWidget::onSaveFile() {
  syncUiToConfig();
  QString path = QFileDialog::getSaveFileName(
      this, tr("Save vehicle"), QString(), tr("Vayu vehicle (*.vveh)"));
  if (path.isEmpty()) return;
  if (!path.endsWith(".vveh", Qt::CaseInsensitive)) path += ".vveh";
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    statusLbl_->setText(tr("Save failed: %1").arg(f.errorString()));
    statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kDanger)));
    return;
  }
  f.write(QJsonDocument(configToJson(cfg_)).toJson(QJsonDocument::Indented));
  statusLbl_->setText(tr("Saved vehicle to %1").arg(QFileInfo(path).fileName()));
  statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kOk)));
}

void GeometryEditorWidget::onLoadFile() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Load vehicle"), QString(),
      tr("Vayu vehicle (*.vveh);;All files (*)"));
  if (path.isEmpty()) return;
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) {
    statusLbl_->setText(tr("Load failed: %1").arg(f.errorString()));
    statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kDanger)));
    return;
  }
  QJsonParseError pe;
  const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
  if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
    statusLbl_->setText(tr("Load failed: invalid vehicle JSON"));
    statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kDanger)));
    return;
  }
  // setConfig syncs the form, re-loads the mesh + recomputes if its path
  // resolves (else keeps the stored inertia/CoM), and fires previewUpdated.
  setConfig(configFromJson(doc.object()));
  statusLbl_->setText(tr("Loaded vehicle from %1").arg(QFileInfo(path).fileName()));
  statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kOk)));
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
