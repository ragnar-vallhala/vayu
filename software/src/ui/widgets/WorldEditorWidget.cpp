#include "WorldEditorWidget.h"

#include "../../core/ui/Buttons.h"
#include "CollapsibleSection.h"

#include "../../core/Theme.h"

#include <QAbstractSpinBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
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
  s->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
  return s;
}

QJsonObject vec3ToJson(const QVector3D& v) {
  return QJsonObject{{"x", v.x()}, {"y", v.y()}, {"z", v.z()}};
}
QVector3D vec3FromJson(const QJsonValue& v, const QVector3D& def) {
  if (!v.isObject()) return def;
  const QJsonObject o = v.toObject();
  return QVector3D(o.value("x").toDouble(def.x()), o.value("y").toDouble(def.y()),
                   o.value("z").toDouble(def.z()));
}

QJsonObject worldToJson(const vsim::WorldConfig& w) {
  QJsonObject root;
  root["format"] = "vayu-world";
  root["version"] = 1;
  root["gravity"] = w.gravity;
  root["ground_z"] = w.ground_z;
  root["restitution"] = w.restitution;
  root["linear_drag"] = w.linear_drag;
  root["angular_drag"] = w.angular_drag;
  root["ground_right_gain"] = w.ground_right_gain;
  root["ground_right_damp"] = w.ground_right_damp;
  root["world_mesh_path"] = w.worldMeshPath;
  root["world_scale"] = w.worldScale;
  root["world_up_axis"] = w.worldUpAxis;
  root["world_mesh_restitution"] = w.worldMeshRestitution;
  root["world_mesh_double_sided"] = w.worldMeshDoubleSided;
  root["world_mesh_offset"] = vec3ToJson(w.worldMeshOffset);
  QJsonArray obs;
  for (const vsim::Obstacle& o : w.obstacles) {
    obs.append(QJsonObject{{"type", o.type},
                           {"pos", vec3ToJson(o.pos)},
                           {"size", vec3ToJson(o.size)},
                           {"rotate", vec3ToJson(o.rotate)},
                           {"restitution", o.restitution}});
  }
  root["obstacles"] = obs;
  return root;
}

vsim::WorldConfig worldFromJson(const QJsonObject& root) {
  vsim::WorldConfig w;  // defaults fill anything missing
  w.gravity = root.value("gravity").toDouble(w.gravity);
  w.ground_z = root.value("ground_z").toDouble(w.ground_z);
  w.restitution = root.value("restitution").toDouble(w.restitution);
  w.linear_drag = root.value("linear_drag").toDouble(w.linear_drag);
  w.angular_drag = root.value("angular_drag").toDouble(w.angular_drag);
  w.ground_right_gain = root.value("ground_right_gain").toDouble(w.ground_right_gain);
  w.ground_right_damp = root.value("ground_right_damp").toDouble(w.ground_right_damp);
  w.worldMeshPath = root.value("world_mesh_path").toString(w.worldMeshPath);
  w.worldScale = root.value("world_scale").toDouble(w.worldScale);
  w.worldUpAxis = root.value("world_up_axis").toInt(w.worldUpAxis);
  w.worldMeshRestitution = root.value("world_mesh_restitution").toDouble(w.worldMeshRestitution);
  w.worldMeshDoubleSided = root.value("world_mesh_double_sided").toBool(w.worldMeshDoubleSided);
  w.worldMeshOffset = vec3FromJson(root.value("world_mesh_offset"), w.worldMeshOffset);
  for (const QJsonValue& v : root.value("obstacles").toArray()) {
    const QJsonObject o = v.toObject();
    vsim::Obstacle ob;
    ob.type = o.value("type").toInt(ob.type);
    ob.pos = vec3FromJson(o.value("pos"), ob.pos);
    ob.size = vec3FromJson(o.value("size"), ob.size);
    ob.rotate = vec3FromJson(o.value("rotate"), ob.rotate);
    ob.restitution = o.value("restitution").toDouble(ob.restitution);
    w.obstacles.push_back(ob);
  }
  return w;
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
    rightGain_ = spin(0.0, 200.0, 1, 1.0, cfg_.ground_right_gain);
    rightGain_->setToolTip(tr("How hard a tipped airframe topples to level on "
                              "the ground (0 = stays as it lands)."));
    rightDamp_ = spin(0.0, 50.0, 1, 0.5, cfg_.ground_right_damp);
    rightDamp_->setToolTip(tr("Damping of the righting rotation (higher = "
                              "settles flatter, less overshoot)."));
    form->addRow(tr("Linear drag:"), linDrag_);
    form->addRow(tr("Angular drag:"), angDrag_);
    form->addRow(tr("Ground righting gain:"), rightGain_);
    form->addRow(tr("Ground righting damp:"), rightDamp_);
    sec->setContentWidget(body);
    root->addWidget(sec);
  }

  // -- Apply / Save / Load --
  {
    auto* row = new QHBoxLayout();
    auto* save = new ui::GhostButton(tr("Save…"), this);
    save->setToolTip(tr("Export the world (environment + obstacles) to a "
                        "portable .vworld file."));
    auto* load = new ui::GhostButton(tr("Load…"), this);
    load->setToolTip(tr("Import a .vworld file."));
    row->addWidget(save);
    row->addWidget(load);
    row->addStretch();
    auto* apply = new ui::SuccessButton(tr("Apply to Sim"), this);
    apply->setToolTip(tr("Push environment + aerodynamics to the simulator."));
    row->addWidget(apply);
    root->addLayout(row);
    connect(apply, &QPushButton::clicked, this, &WorldEditorWidget::onApply);
    connect(save, &QPushButton::clicked, this, &WorldEditorWidget::onSaveWorld);
    connect(load, &QPushButton::clicked, this, &WorldEditorWidget::onLoadWorld);

    fileStatus_ = new QLabel(this);
    fileStatus_->setStyleSheet(
        QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextMuted)));
    root->addWidget(fileStatus_);
  }

  // -- World mesh --
  buildWorldMeshSection(root);

  // -- Obstacles --
  buildObstacleSection(root);

  root->addStretch();
}

void WorldEditorWidget::buildWorldMeshSection(QVBoxLayout* root) {
  auto* sec = new CollapsibleSection(tr("World mesh"), this);
  auto* body = new QWidget();
  auto* col = new QVBoxLayout(body);

  worldMeshLabel_ = new QLabel(tr("(none)"), body);
  worldMeshLabel_->setWordWrap(true);
  worldMeshLabel_->setStyleSheet(
      QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextMuted)));
  col->addWidget(worldMeshLabel_);

  {
    auto* row = new QHBoxLayout();
    auto* imp = new ui::GhostButton(tr("Import…"), body);
    imp->setToolTip(tr("Import a world mesh (.glb/.gltf/.obj/.stl/.blend). "
                       "Rendered now; collision is a later phase."));
    auto* clr = new ui::DangerButton(tr("Clear"), body);
    connect(imp, &QPushButton::clicked, this, &WorldEditorWidget::onImportWorldMesh);
    connect(clr, &QPushButton::clicked, this, &WorldEditorWidget::onClearWorldMesh);
    row->addWidget(imp);
    row->addWidget(clr);
    row->addStretch();
    col->addLayout(row);
  }
  {
    auto* form = new QFormLayout();
    worldScale_ = spin(0.0001, 10000.0, 4, 0.1, cfg_.worldScale);
    worldScale_->setToolTip(tr("Mesh units → metres."));
    worldUpAxis_ = new QComboBox(body);
    worldUpAxis_->addItem(tr("Z-up (Blender)"), 0);
    worldUpAxis_->addItem(tr("Y-up (glTF)"), 1);
    worldUpAxis_->setCurrentIndex(cfg_.worldUpAxis == 1 ? 1 : 0);
    auto onEdit = [this] {
      if (worldMeshSyncing_) return;
      cfg_.worldScale = static_cast<float>(worldScale_->value());
      cfg_.worldUpAxis = worldUpAxis_->currentData().toInt();
      cfg_.worldMeshOffset = QVector3D(worldOffset_[0]->value(),
                                       worldOffset_[1]->value(),
                                       worldOffset_[2]->value());
      if (!cfg_.worldMeshPath.isEmpty()) emit worldMeshChanged();
    };
    connect(worldScale_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [onEdit] { onEdit(); });
    connect(worldUpAxis_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [onEdit] { onEdit(); });
    form->addRow(tr("Scale:"), worldScale_);
    form->addRow(tr("Up axis:"), worldUpAxis_);

    // World placement (NED metres): move the imported world so the drone's
    // spawn (the origin) isn't trapped inside/under it. X=north, Y=east,
    // Z=down — so a negative Z lifts the world above the spawn.
    auto* offRow = new QHBoxLayout();
    const char* lbl[3] = {"N", "E", "D"};
    for (int i = 0; i < 3; ++i) {
      worldOffset_[i] = spin(-10000.0, 10000.0, 2, 0.5, cfg_.worldMeshOffset[i],
                             QStringLiteral(" m"));
      worldOffset_[i]->setToolTip(tr("World placement offset (NED): N(orth)/"
                                     "E(ast)/D(own). Negative D lifts it up."));
      offRow->addWidget(new QLabel(tr(lbl[i]), body));
      offRow->addWidget(worldOffset_[i]);
      connect(worldOffset_[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged),
              this, [onEdit] { onEdit(); });
    }
    form->addRow(tr("Offset:"), offRow);
    col->addLayout(form);
  }

  sec->setContentWidget(body);
  root->addWidget(sec);
}

void WorldEditorWidget::onImportWorldMesh() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Import world mesh"), QString(),
      tr("Meshes (*.glb *.gltf *.obj *.stl *.ply *.blend);;All files (*)"));
  if (path.isEmpty()) return;
  cfg_.worldMeshPath = path;
  if (worldMeshLabel_) worldMeshLabel_->setText(QFileInfo(path).fileName());
  emit worldMeshChanged();
}

void WorldEditorWidget::onClearWorldMesh() {
  cfg_.worldMeshPath.clear();
  if (worldMeshLabel_) worldMeshLabel_->setText(tr("(none)"));
  emit worldMeshChanged();
}

void WorldEditorWidget::onSaveWorld() {
  syncUiToConfig();
  QString path = QFileDialog::getSaveFileName(this, tr("Save world"), QString(),
                                              tr("Vayu world (*.vworld)"));
  if (path.isEmpty()) return;
  if (!path.endsWith(".vworld", Qt::CaseInsensitive)) path += ".vworld";
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    if (fileStatus_) fileStatus_->setText(tr("Save failed: %1").arg(f.errorString()));
    return;
  }
  f.write(QJsonDocument(worldToJson(cfg_)).toJson(QJsonDocument::Indented));
  if (fileStatus_)
    fileStatus_->setText(tr("Saved %1 (%2 obstacles)")
                             .arg(QFileInfo(path).fileName())
                             .arg(cfg_.obstacles.size()));
}

void WorldEditorWidget::onLoadWorld() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Load world"), QString(), tr("Vayu world (*.vworld);;All files (*)"));
  if (path.isEmpty()) return;
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) {
    if (fileStatus_) fileStatus_->setText(tr("Load failed: %1").arg(f.errorString()));
    return;
  }
  QJsonParseError pe;
  const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
  if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
    if (fileStatus_) fileStatus_->setText(tr("Load failed: invalid world JSON"));
    return;
  }
  setConfig(worldFromJson(doc.object()));
  if (fileStatus_)
    fileStatus_->setText(tr("Loaded %1 (%2 obstacles)")
                             .arg(QFileInfo(path).fileName())
                             .arg(cfg_.obstacles.size()));
  // Push to the daemon (env) + renderer/persistence (obstacles) + reload the
  // imported world mesh (a .vworld can carry world_mesh_path/scale/up-axis, so
  // without this the mesh fields update but the geometry never (re)loads).
  emit worldApplied();
  emit obstaclesChanged();
  emit worldMeshChanged();
}

void WorldEditorWidget::buildObstacleSection(QVBoxLayout* root) {
  auto* sec = new CollapsibleSection(tr("Obstacles"), this);
  auto* body = new QWidget();
  auto* col = new QVBoxLayout(body);

  auto* hint = new QLabel(
      tr("Click a shape in the 3D view to select; G move · R rotate · S scale, "
         "X/Y/Z to constrain, click to confirm / Esc to cancel."));
  hint->setWordWrap(true);
  hint->setStyleSheet(
      QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextDim)));
  col->addWidget(hint);

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

void WorldEditorWidget::onObstacleSelected(int row) {
  syncFormFromSelection();
  emit obstacleSelectionChanged(row);
}

void WorldEditorWidget::setObstacleFromGizmo(int index, QVector3D pos,
                                             QVector3D size, QVector3D rotate) {
  if (index < 0 || index >= cfg_.obstacles.size()) return;
  vsim::Obstacle& o = cfg_.obstacles[index];
  o.pos = pos;
  o.size = size;
  o.rotate = rotate;
  if (obsList_ && obsList_->currentRow() != index)
    obsList_->setCurrentRow(index);
  syncFormFromSelection();
  refreshObstacleList();
  emit obstaclesChanged();
}

void WorldEditorWidget::selectObstacleRow(int index) {
  if (!obsList_) return;
  if (index >= 0 && index < cfg_.obstacles.size()) {
    if (obsList_->currentRow() != index) obsList_->setCurrentRow(index);
  } else {
    obsList_->setCurrentRow(-1);
  }
}

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
  rightGain_->setValue(cfg_.ground_right_gain);
  rightDamp_->setValue(cfg_.ground_right_damp);
}

void WorldEditorWidget::syncUiToConfig() {
  cfg_.gravity = static_cast<float>(gravity_->value());
  cfg_.ground_z = static_cast<float>(groundZ_->value());
  cfg_.restitution = static_cast<float>(rest_->value());
  cfg_.linear_drag = static_cast<float>(linDrag_->value());
  cfg_.angular_drag = static_cast<float>(angDrag_->value());
  cfg_.ground_right_gain = static_cast<float>(rightGain_->value());
  cfg_.ground_right_damp = static_cast<float>(rightDamp_->value());
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
  worldMeshSyncing_ = true;
  if (worldMeshLabel_)
    worldMeshLabel_->setText(cfg_.worldMeshPath.isEmpty()
                                 ? tr("(none)")
                                 : QFileInfo(cfg_.worldMeshPath).fileName());
  if (worldScale_) worldScale_->setValue(cfg_.worldScale);
  if (worldUpAxis_) worldUpAxis_->setCurrentIndex(cfg_.worldUpAxis == 1 ? 1 : 0);
  for (int i = 0; i < 3; ++i)
    if (worldOffset_[i]) worldOffset_[i]->setValue(cfg_.worldMeshOffset[i]);
  worldMeshSyncing_ = false;
}
