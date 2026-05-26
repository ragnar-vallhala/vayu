#include "CalibrationWidget.h"

#include "../core/crc.h"
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
    m_biasOnlyRadio->setText(
        "Quick Calibration"); // Offset + Scale depending on what you implement
    m_fullCalibRadio->setText("Advanced Calibration"); // Ellipsoid fit
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
}

void CalibrationWidget::sendCalibrationCommand(int imu_id, int type) {
  uint32_t now = static_cast<uint32_t>(QDateTime::currentMSecsSinceEpoch());
  uint8_t dev_id = 42;

  QByteArray pkt;
  pkt.append(static_cast<char>(0x56)); // sync
  pkt.append(static_cast<char>(0x31)); // type 3 (command), ver 1
  pkt.append(
      static_cast<char>(11)); // length 11 (cmd_id[2] + argc[1] + args[2*4])
  pkt.append(static_cast<char>(dev_id));
  pkt.append(reinterpret_cast<const char *>(&now), 4);

  uint16_t cmd_id = 0x0001; // CMD_CALIBRATE_IMU
  pkt.append(reinterpret_cast<const char *>(&cmd_id), 2);
  pkt.append(static_cast<char>(0x02)); // argc = 2

  float f_imu = static_cast<float>(imu_id);
  float f_type = static_cast<float>(type);
  pkt.append(reinterpret_cast<const char *>(&f_imu), 4);
  pkt.append(reinterpret_cast<const char *>(&f_type), 4);

  uint32_t crc = CRC32::calculate(
      reinterpret_cast<const uint8_t *>(pkt.constData()), pkt.size());
  pkt.append(reinterpret_cast<const char *>(&crc), 4);

  emit commandRequested(pkt);
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

  // Send empty CMD_CALIBRATE_IMU to cancel
  uint32_t now = static_cast<uint32_t>(QDateTime::currentMSecsSinceEpoch());
  QByteArray pkt;
  pkt.append(static_cast<char>(0x56));
  pkt.append(static_cast<char>(0x31));
  pkt.append(static_cast<char>(3)); // cmd_id[2] + argc[1]
  pkt.append(static_cast<char>(42));
  pkt.append(reinterpret_cast<const char *>(&now), 4);
  uint16_t cmd_id = 0x0001;
  pkt.append(reinterpret_cast<const char *>(&cmd_id), 2);
  pkt.append(static_cast<char>(0x00)); // argc = 0
  uint32_t crc = CRC32::calculate(
      reinterpret_cast<const uint8_t *>(pkt.constData()), pkt.size());
  pkt.append(reinterpret_cast<const char *>(&crc), 4);
  emit commandRequested(pkt);
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
    }
  }
}
