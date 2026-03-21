#include "CalibrationWidget.h"
#include "../core/crc.h"
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

  QString cardStyle =
      "QPushButton { background: #21252B; color: #ABB2BF; border: 2px solid "
      "#3E4452; border-radius: 8px; font-weight: bold; min-height: 80px; }"
      "QPushButton:checked { background: #2C313C; color: #61AFEF; "
      "border-color: #61AFEF; }"
      "QPushButton:hover:!checked { background: #2C313C; }";

  for (auto *btn : {m_accBtn, m_gyrBtn, m_magBtn}) {
    btn->setCheckable(true);
    btn->setStyleSheet(cardStyle);
    sensorLayout->addWidget(btn);
  }

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

  m_configGroup = new QGroupBox("Calibration Mode", this);
  m_configGroup->setStyleSheet(
      "QGroupBox { color: #ABB2BF; font-weight: bold; border: 1px solid "
      "#3E4452; margin-top: 10px; padding: 10px; } "
      "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top "
      "left; left: 10px; }");
  auto *configVBox = new QVBoxLayout(m_configGroup);
  m_biasOnlyRadio = new QRadioButton("Bias-Only (Zeroing)", this);
  m_fullCalibRadio =
      new QRadioButton("Full Calibration (Scale + Offset)", this);
  m_biasOnlyRadio->setChecked(true);
  m_fullCalibRadio->setEnabled(false); // Default is Gyroscope
  m_biasOnlyRadio->setStyleSheet("color: #ABB2BF;");
  m_fullCalibRadio->setStyleSheet("color: #ABB2BF;");
  configVBox->addWidget(m_biasOnlyRadio);
  configVBox->addWidget(m_fullCalibRadio);
  midLayout->addWidget(m_configGroup, 1);

  m_axisStatusArea = new QWidget(this);
  auto *axisGrid = new QGridLayout(m_axisStatusArea);
  QString axisStyle =
      "QLabel { background: #21252B; color: #5C6370; border: 1px solid "
      "#3E4452; border-radius: 4px; padding: 5px; font-weight: bold; }";

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
  m_instructionGroup = new QGroupBox("Instruction Panel", this);
  m_instructionGroup->setStyleSheet(
      "QGroupBox { color: #61AFEF; font-weight: bold; border: 2px solid "
      "#61AFEF; border-radius: 8px; margin-top: 15px; padding: 20px; } "
      "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top "
      "center; }");
  auto *instrVBox = new QVBoxLayout(m_instructionGroup);

  m_instructionText = new QLabel("SELECT A SENSOR AND PRESS START", this);
  m_instructionText->setAlignment(Qt::AlignCenter);
  m_instructionText->setWordWrap(true);
  m_instructionText->setStyleSheet(
      "font-size: 20px; font-weight: bold; color: #FFFFFF;");
  instrVBox->addWidget(m_instructionText);

  m_progressBar = new QProgressBar(this);
  m_progressBar->setFixedHeight(12);
  m_progressBar->setTextVisible(false);
  m_progressBar->setStyleSheet(
      "QProgressBar { background: #21252B; border: 1px solid #3E4452; "
      "border-radius: 6px; } "
      "QProgressBar::chunk { background: #98C379; border-radius: 5px; }");
  instrVBox->addWidget(m_progressBar);

  mainLayout->addWidget(m_instructionGroup);

  // --- Footer Controls ---
  auto *footerLayout = new QHBoxLayout();
  m_statusLabel = new QLabel("SYSTEM READY", this);
  m_statusLabel->setStyleSheet("color: #61AFEF; font-weight: bold;");
  footerLayout->addWidget(m_statusLabel);
  footerLayout->addStretch();

  m_cancelBtn = new QPushButton("CANCEL", this);
  m_cancelBtn->setFixedSize(120, 40);
  m_cancelBtn->setStyleSheet(
      "QPushButton { background: #E06C75; color: #21252B; font-weight: bold; "
      "border-radius: 4px; }");
  m_cancelBtn->setVisible(false);
  connect(m_cancelBtn, &QPushButton::clicked, this,
          &CalibrationWidget::onCancelClicked);
  footerLayout->addWidget(m_cancelBtn);

  m_startBtn = new QPushButton("START CALIBRATION", this);
  m_startBtn->setFixedSize(200, 40);
  m_startBtn->setStyleSheet(
      "QPushButton { background: #98C379; color: #21252B; font-weight: bold; "
      "border-radius: 4px; }"
      "QPushButton:hover { background: #B5E890; }"
      "QPushButton:disabled { background: #4B5263; color: #2C313C; }");
  connect(m_startBtn, &QPushButton::clicked, this,
          &CalibrationWidget::onStartClicked);
  footerLayout->addWidget(m_startBtn);

  mainLayout->addLayout(footerLayout);
  mainLayout->addStretch();
}

void CalibrationWidget::setProtocol(DroneProtocol *protocol) {
  m_protocol = protocol;
  connect(m_protocol, &DroneProtocol::statusReceived, this,
          &CalibrationWidget::onStatusReceived);
  connect(m_protocol, &DroneProtocol::calibrationUpdateReceived, this,
          [this](const CalibrationUpdate &update) {
            if (update.type == CalibUpdateType::Progress) {
              onProgressReceived(update.data);
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

  // Refinement: Gyro should only have Bias-Only option (keep board stable)
  if (id == 2) {
    m_fullCalibRadio->setEnabled(false);
    m_biasOnlyRadio->setChecked(true);
  } else {
    m_fullCalibRadio->setEnabled(true);
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

  // Clear axis statuses
  for (auto *lbl : m_axisMap) {
    lbl->setStyleSheet(
        "background: #21252B; color: #5C6370; border: 1px solid #3E4452; "
        "border-radius: 4px; padding: 5px; font-weight: bold;");
  }

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
          "background: #2D4A2D; color: #98C379; border: 2px solid #98C379; "
          "border-radius: 4px; padding: 5px; font-weight: bold;");
    }
  }
}

void CalibrationWidget::onInstructionReceived(int type) {
  CalibUpdateType instruction = static_cast<CalibUpdateType>(type);
  m_currentAxis = instruction;

  // Highlight target axis
  if (m_axisMap.contains(instruction)) {
    m_axisMap[instruction]->setStyleSheet(
        "background: #4A4A2D; color: #E5C07B; border: 2px solid #E5C07B; "
        "border-radius: 4px; padding: 5px; font-weight: bold;");
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
      "QGroupBox { color: #E5C07B; font-weight: bold; border: 2px solid "
      "#E5C07B; border-radius: 8px; margin-top: 15px; padding: 20px; } "
      "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top "
      "center; }");
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
          "QGroupBox { color: #98C379; font-weight: bold; border: 2px solid "
          "#98C379; border-radius: 8px; margin-top: 15px; padding: 20px; } "
          "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: "
          "top center; }");
      m_startBtn->setEnabled(true);
      m_cancelBtn->setVisible(false);
      m_sensorSelectArea->setEnabled(true);
      m_configGroup->setEnabled(true);
      m_progressBar->setValue(100);
    }
  }
}
