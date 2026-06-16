#include "CalibrationWidget.h"

#include "../protocol/CommandCodec.h"
#include "core/Theme.h"
#include "core/ui/Buttons.h"

#include <QDateTime>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QRegularExpression>
#include <QStyle>

CalibrationWidget::CalibrationWidget(QWidget *parent) : QWidget(parent) {
  auto *mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(20, 20, 20, 20);
  mainLayout->setSpacing(20);

  // --- Sensor Selection Area ---
  m_sensorSelectArea = new QWidget(this);
  auto *sensorLayout = new QHBoxLayout(m_sensorSelectArea);
  sensorLayout->setContentsMargins(0, 0, 0, 0);
  sensorLayout->setSpacing(15);

  m_accBtn = new QPushButton("ACCELEROMETER", this);
  m_gyrBtn = new QPushButton("GYROSCOPE", this);
  m_magBtn = new QPushButton("MAGNETOMETER", this);

  // Big checkable cards — styling lives in dark.qss under #SensorCard.
  for (auto *btn : {m_accBtn, m_gyrBtn, m_magBtn}) {
    btn->setObjectName("SensorCard");
    btn->setCheckable(true);
    sensorLayout->addWidget(btn);
  }
  m_accBtn->setToolTip(tr("Accelerometer — bias + 6-axis full calibration"));
  m_gyrBtn->setToolTip(tr("Gyroscope — bias-only zeroing"));
  m_magBtn->setToolTip(tr("Magnetometer — rotate the airframe for axis coverage"));

  m_typeGroup = new QButtonGroup(this);
  m_typeGroup->addButton(m_accBtn, 1);
  m_typeGroup->addButton(m_gyrBtn, 2);
  m_typeGroup->addButton(m_magBtn, 3);
  m_gyrBtn->setChecked(true); // Default
  connect(m_typeGroup, &QButtonGroup::idClicked, this,
          &CalibrationWidget::onSensorSelected);

  mainLayout->addWidget(m_sensorSelectArea);

  // --- Configuration & Axis Status ---
  auto *midLayout = new QHBoxLayout();

  // GroupBox chrome comes from global QSS.
  m_configGroup = new QGroupBox("Calibration Mode", this);
  auto *configVBox = new QVBoxLayout(m_configGroup);
  m_biasOnlyRadio = new QRadioButton("Bias-Only (Zeroing)", this);
  m_fullCalibRadio =
      new QRadioButton("Full Calibration (Scale + Offset)", this);
  m_biasOnlyRadio->setChecked(true);
  m_fullCalibRadio->setEnabled(false); // Default is Gyroscope
  configVBox->addWidget(m_biasOnlyRadio);
  configVBox->addWidget(m_fullCalibRadio);
  midLayout->addWidget(m_configGroup, 1);

  m_axisStatusArea = new QWidget(this);
  auto *axisGrid = new QGridLayout(m_axisStatusArea);
  // Idle style for the six axis pills; active pill is re-styled when
  // the firmware advances the calibration step.
  const QString axisStyle =
      QString("QLabel { background: %1; color: %2; border: 1px solid %3; "
              "border-radius: 4px; padding: 5px; font-weight: bold; }")
          .arg(Theme::hex(Theme::kSurface),
               Theme::hex(Theme::kTextDim),
               Theme::hex(Theme::kBorderStrong));

  m_axisLabelX = new QLabel("X", this);
  m_axisLabelNX = new QLabel("-X", this);
  m_axisLabelY = new QLabel("Y", this);
  m_axisLabelNY = new QLabel("-Y", this);
  m_axisLabelZ = new QLabel("Z", this);
  m_axisLabelNZ = new QLabel("-Z", this);

  QList<QLabel *> labels = {m_axisLabelX,  m_axisLabelNX, m_axisLabelY,
                            m_axisLabelNY, m_axisLabelZ,  m_axisLabelNZ};
  QList<CalibUpdateType> types = {
      CalibUpdateType::NoseUp,    CalibUpdateType::NoseDown,
      CalibUpdateType::RightDown, CalibUpdateType::LeftDown,
      CalibUpdateType::Upright,   CalibUpdateType::UpsideDown};

  for (int i = 0; i < 6; ++i) {
    labels[i]->setStyleSheet(axisStyle);
    labels[i]->setAlignment(Qt::AlignCenter);
    labels[i]->setMinimumWidth(60);
    axisGrid->addWidget(labels[i], i / 2, i % 2);
    m_axisMap[types[i]] = labels[i];
  }

  midLayout->addWidget(m_axisStatusArea, 1);
  mainLayout->addLayout(midLayout);

  // --- Instruction Panel ---
  // Accented frame with title centered — this is intentionally louder
  // than the global QGroupBox style.
  m_instructionGroup = new QGroupBox("Instruction Panel", this);
  m_instructionGroup->setStyleSheet(
      QString("QGroupBox { color: %1; font-weight: bold; border: 2px solid %1; "
              "border-radius: 8px; margin-top: 15px; padding: 20px; } "
              "QGroupBox::title { subcontrol-origin: margin; "
              "subcontrol-position: top center; }")
          .arg(Theme::hex(Theme::kAccent)));
  auto *instrVBox = new QVBoxLayout(m_instructionGroup);

  m_instructionText = new QLabel("SELECT A SENSOR AND PRESS START", this);
  m_instructionText->setAlignment(Qt::AlignCenter);
  m_instructionText->setWordWrap(true);
  m_instructionText->setStyleSheet(
      QString("font-size: 20px; font-weight: bold; color: %1;")
          .arg(Theme::hex(Theme::kText)));
  instrVBox->addWidget(m_instructionText);

  m_progressBar = new QProgressBar(this);
  m_progressBar->setFixedHeight(12);
  m_progressBar->setTextVisible(false);
  // Custom chunk colour (success-green) overrides the QSS default
  // (accent). Worth the inline rule for the visual cue.
  m_progressBar->setStyleSheet(
      QString("QProgressBar { background: %1; border: 1px solid %2; "
              "border-radius: 6px; } "
              "QProgressBar::chunk { background: %3; border-radius: 5px; }")
          .arg(Theme::hex(Theme::kSurface),
               Theme::hex(Theme::kBorderStrong),
               Theme::hex(Theme::kOk)));
  instrVBox->addWidget(m_progressBar);

  // Guided step checklist (FR: gated calibration wizard). Populated on Start
  // from CalibrationWizard and updated as the firmware drives each orientation.
  m_stepList = new QListWidget(this);
  m_stepList->setFocusPolicy(Qt::NoFocus);
  m_stepList->setSelectionMode(QAbstractItemView::NoSelection);
  m_stepList->setStyleSheet(
      QString("QListWidget { background: %1; border: 1px solid %2; "
              "border-radius: 6px; } QListWidget::item { padding: 3px; }")
          .arg(Theme::hex(Theme::kSurface),
               Theme::hex(Theme::kBorderStrong)));
  instrVBox->addWidget(m_stepList);

  mainLayout->addWidget(m_instructionGroup);

  // --- Footer Controls ---
  auto *footerLayout = new QHBoxLayout();
  m_statusLabel = new QLabel("SYSTEM READY", this);
  m_statusLabel->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(Theme::hex(Theme::kAccent)));
  footerLayout->addWidget(m_statusLabel);
  footerLayout->addStretch();

  m_cancelBtn = new ui::DangerButton(tr("CANCEL"), this);
  m_cancelBtn->setFixedSize(120, 40);
  m_cancelBtn->setToolTip(tr("Abort the running calibration"));
  m_cancelBtn->setVisible(false);
  connect(m_cancelBtn, &QPushButton::clicked, this,
          &CalibrationWidget::onCancelClicked);
  footerLayout->addWidget(m_cancelBtn);

  m_startBtn = new ui::SuccessButton(tr("START CALIBRATION"), this);
  m_startBtn->setFixedSize(200, 40);
  m_startBtn->setToolTip(tr("Begin calibrating the selected sensor"));
  connect(m_startBtn, &QPushButton::clicked, this,
          &CalibrationWidget::onStartClicked);
  footerLayout->addWidget(m_startBtn);

  mainLayout->addLayout(footerLayout);
  mainLayout->addStretch();

  // Default to disconnected. MainWindow::setConnected drives the real
  // value as soon as the serial port opens.
  setConnected(false);
}

void CalibrationWidget::setConnected(bool connected) {
  m_connected = connected;
  if (m_sensorSelectArea) m_sensorSelectArea->setEnabled(connected);
  if (m_configGroup)      m_configGroup->setEnabled(connected);
  if (m_startBtn)         m_startBtn->setEnabled(connected);
  // Cancel only makes sense mid-calibration, which can't happen while
  // disconnected — leave it visibility-driven elsewhere.
  if (!connected && m_statusLabel) {
    m_statusLabel->setText(tr("NOT CONNECTED"));
    m_statusLabel->setStyleSheet(
        QString("color: %1; font-weight: bold;")
            .arg(Theme::hex(Theme::kDanger)));
    if (m_instructionText)
      m_instructionText->setText(tr("CONNECT A DRONE TO CALIBRATE"));
  } else if (connected && m_statusLabel) {
    m_statusLabel->setText(tr("SYSTEM READY"));
    m_statusLabel->setStyleSheet(
        QString("color: %1; font-weight: bold;")
            .arg(Theme::hex(Theme::kAccent)));
    if (m_instructionText)
      m_instructionText->setText(tr("SELECT A SENSOR AND PRESS START"));
  }
}

void CalibrationWidget::setProtocol(DroneProtocol *protocol) {
  m_protocol = protocol;
  connect(m_protocol, &DroneProtocol::statusReceived, this,
          &CalibrationWidget::onStatusReceived);
  connect(m_protocol, &DroneProtocol::calibrationUpdateReceived, this,
          [this](const CalibrationUpdate &update) {
            if (update.type == CalibUpdateType::Progress) {
              onProgressReceived(update.data);
            } else if (update.type == CalibUpdateType::MagAxisCoverage) {
              m_statusLabel->setText(QString("COVERAGE: X:%1 Y:%2 Z:%3")
                                         .arg(update.values[0], 0, 'f', 0)
                                         .arg(update.values[1], 0, 'f', 0)
                                         .arg(update.values[2], 0, 'f', 0));
            } else {
              onInstructionReceived(static_cast<int>(update.type));
            }
          });
}

void CalibrationWidget::onSensorSelected(int id) {
  m_selectedImuId = id;
  m_statusLabel->setText(QString("SENSOR SELECTED: %1")
                             .arg(id == 1   ? "ACCEL"
                                  : id == 2 ? "GYRO"
                                            : "MAG"));

  // Reset to defaults
  m_biasOnlyRadio->setText("Bias-Only (Zeroing)");
  m_fullCalibRadio->setText("Full Calibration (Scale + Offset)");
  m_fullCalibRadio->setEnabled(true);
  m_axisStatusArea->setVisible(true);

  if (id == 2) {
    // Gyro: Bias-Only only
    m_fullCalibRadio->setEnabled(false);
    m_biasOnlyRadio->setChecked(true);
  } else if (id == 3) {
    // Mag has a single routine (hard + soft iron ellipsoid fit); the firmware
    // ignores the mode arg, so don't offer a misleading second option.
    m_biasOnlyRadio->setText("Hard + Soft Iron (Ellipsoid)");
    m_biasOnlyRadio->setChecked(true);
    m_fullCalibRadio->setEnabled(false);
    m_axisStatusArea->setVisible(false);
  }
}

void CalibrationWidget::onStartClicked() {
  if (!m_protocol)
    return;

  m_startBtn->setEnabled(false);
  m_cancelBtn->setVisible(true);
  m_sensorSelectArea->setEnabled(false);
  m_configGroup->setEnabled(false);
  m_progressBar->setValue(0);

  // Clear axis statuses back to the muted idle look.
  const QString idle =
      QString("background: %1; color: %2; border: 1px solid %3; "
              "border-radius: 4px; padding: 5px; font-weight: bold;")
          .arg(Theme::hex(Theme::kSurface),
               Theme::hex(Theme::kTextDim),
               Theme::hex(Theme::kBorderStrong));
  for (auto *lbl : m_axisMap) lbl->setStyleSheet(idle);

  int calType = m_fullCalibRadio->isChecked() ? 1 : 0;
  sendCalibrationCommand(m_selectedImuId, calType);

  // Start the guided wizard for this mode and show the step checklist.
  m_wizard.begin(currentMode());
  refreshSteps();
}

CalibMode CalibrationWidget::currentMode() const {
  const bool full = m_fullCalibRadio && m_fullCalibRadio->isChecked();
  switch (m_selectedImuId) {
    case 1:  // accelerometer
      return full ? CalibMode::Accel6Axis : CalibMode::AccelBias;
    case 3:  // magnetometer
      return CalibMode::Mag;
    case 2:  // gyroscope
    default:
      return CalibMode::Gyro;
  }
}

void CalibrationWidget::refreshSteps() {
  if (!m_stepList) return;
  m_stepList->clear();
  const QVector<CalibStep> &steps = m_wizard.steps();
  for (int i = 0; i < steps.size(); ++i) {
    QString mark;
    if (m_wizard.isComplete() || i < m_wizard.doneCount())
      mark = QStringLiteral("✓  ");  // ✓ done
    else if (i == m_wizard.currentIndex())
      mark = QStringLiteral("▶  ");  // ▶ current
    else
      mark = QStringLiteral("○  ");  // ○ pending
    auto *it = new QListWidgetItem(mark + steps[i].label + " — " + steps[i].hint,
                                   m_stepList);
    if (i == m_wizard.currentIndex() && !m_wizard.isComplete()) {
      QFont f = it->font();
      f.setBold(true);
      it->setFont(f);
    }
  }
  m_progressBar->setValue(int(m_wizard.progress() * 100.0));
}

void CalibrationWidget::sendCalibrationCommand(int imu_id, int type) {
  // NavLink v2 CMD_CALIBRATE_IMU carries a single `which` byte. Pack the sensor
  // selector in the high nibble (1=accel, 2=gyro, 3=mag) and the mode in the
  // low nibble (0=bias, 1=full); the firmware router unpacks both.
  quint8 which = static_cast<quint8>(((imu_id & 0x0F) << 4) | (type & 0x0F));
  emit commandRequested(CommandCodec::encodeCalibrate(which));
}

void CalibrationWidget::onProgressReceived(float pct) {
  m_progressBar->setValue(static_cast<int>(pct));
  m_statusLabel->setText(QString("CALIBRATING... %1%").arg(pct, 0, 'f', 1));

  if (pct >= 100.0f && m_currentAxis != CalibUpdateType::Progress) {
    if (m_axisMap.contains(m_currentAxis)) {
      m_axisMap[m_currentAxis]->setStyleSheet(
          QString("background: %1; color: %2; border: 2px solid %2; "
                  "border-radius: 4px; padding: 5px; font-weight: bold;")
              .arg(QColor(45, 74, 45).name(QColor::HexRgb),
                   Theme::hex(Theme::kOk)));
    }
  }
}

void CalibrationWidget::onInstructionReceived(int type) {
  CalibUpdateType instruction = static_cast<CalibUpdateType>(type);
  m_currentAxis = instruction;

  // Advance the guided wizard checklist to the prompted orientation.
  m_wizard.onInstruction(instruction);
  refreshSteps();

  // Highlight target axis with the warn accent.
  if (m_axisMap.contains(instruction)) {
    m_axisMap[instruction]->setStyleSheet(
        QString("background: %1; color: %2; border: 2px solid %2; "
                "border-radius: 4px; padding: 5px; font-weight: bold;")
            .arg(QColor(74, 74, 45).name(QColor::HexRgb),
                 Theme::hex(Theme::kWarn)));
  }

  QString msg;
  switch (instruction) {
  case CalibUpdateType::NoseUp:
    msg = "PLACE DRONE NOSE UP";
    break;
  case CalibUpdateType::NoseDown:
    msg = "PLACE DRONE NOSE DOWN";
    break;
  case CalibUpdateType::RightDown:
    msg = "PLACE DRONE RIGHT SIDE DOWN";
    break;
  case CalibUpdateType::LeftDown:
    msg = "PLACE DRONE LEFT SIDE DOWN";
    break;
  case CalibUpdateType::Upright:
    msg = "PLACE DRONE LEVEL (UPRIGHT)";
    break;
  case CalibUpdateType::UpsideDown:
    msg = "PLACE DRONE UPSIDE DOWN";
    break;
  case CalibUpdateType::FreeRot:
    msg = "ROTATE DRONE FREELY IN ALL AXES";
    break;
  default:
    msg = "STAY STILL...";
    break;
  }

  m_instructionText->setText(msg);
  m_instructionGroup->setStyleSheet(
      QString("QGroupBox { color: %1; font-weight: bold; border: 2px solid %1; "
              "border-radius: 8px; margin-top: 15px; padding: 20px; } "
              "QGroupBox::title { subcontrol-origin: margin; "
              "subcontrol-position: top center; }")
          .arg(Theme::hex(Theme::kWarn)));
}

void CalibrationWidget::onCancelClicked() {
  if (!m_protocol)
    return;

  // Cancel: NavLink v2 CMD_CALIBRATE_IMU with the 0xFF sentinel (stop routine).
  emit commandRequested(CommandCodec::encodeCalibrate(0xFF));
  m_startBtn->setEnabled(true);
  m_cancelBtn->setVisible(false);
  m_sensorSelectArea->setEnabled(true);
  m_configGroup->setEnabled(true);
  m_statusLabel->setText("READY");
  m_instructionText->setText("CALIBRATION CANCELED");
}

void CalibrationWidget::onStatusReceived(const QString &msg) {
  if (msg == "STANDBY") {
    if (!m_startBtn->isEnabled()) {
      m_statusLabel->setText("SUCCESSFUL");
      m_instructionText->setText("CALIBRATION COMPLETE!");
      m_instructionGroup->setStyleSheet(
          QString("QGroupBox { color: %1; font-weight: bold; "
                  "border: 2px solid %1; border-radius: 8px; "
                  "margin-top: 15px; padding: 20px; } "
                  "QGroupBox::title { subcontrol-origin: margin; "
                  "subcontrol-position: top center; }")
              .arg(Theme::hex(Theme::kOk)));
      m_startBtn->setEnabled(true);
      m_cancelBtn->setVisible(false);
      m_sensorSelectArea->setEnabled(true);
      m_configGroup->setEnabled(true);
      m_progressBar->setValue(100);
      m_wizard.markComplete();  // tick every step done
      refreshSteps();
    }
  }
}
