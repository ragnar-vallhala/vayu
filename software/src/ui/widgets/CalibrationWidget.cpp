#include "CalibrationWidget.h"
#include "crc.h"
#include <QDateTime>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QRegularExpression>

CalibrationWidget::CalibrationWidget(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(20, 20, 20, 20);
  layout->setSpacing(15);

  auto *backBtn = new QPushButton("← Back to Home", this);
  backBtn->setFixedWidth(150);
  connect(backBtn, &QPushButton::clicked, this,
          &CalibrationWidget::backToHomeRequested);
  layout->addWidget(backBtn);

  auto *group = new QGroupBox("IMU Calibration", this);
  auto *vbox = new QVBoxLayout(group);
  vbox->setContentsMargins(30, 30, 30, 30);
  vbox->setSpacing(20);

  auto *info = new QLabel("Ensure the drone is perfectly stationary and level "
                          "before starting calibration. "
                          "The process will take approximately 5 seconds.",
                          this);
  info->setWordWrap(true);
  info->setAlignment(Qt::AlignCenter);
  info->setStyleSheet("color: #ABB2BF; font-style: italic;");
  vbox->addWidget(info);

  m_progressBar = new QProgressBar(this);
  m_progressBar->setRange(0, 100);
  m_progressBar->setValue(0);
  m_progressBar->setFixedHeight(30);
  m_progressBar->setTextVisible(true);
  m_progressBar->setStyleSheet(
      "QProgressBar { border: 2px solid #3E4452; border-radius: 5px; "
      "text-align: center; background: #21252B; color: #FFFFFF; font-weight: "
      "bold; }"
      "QProgressBar::chunk { background-color: #98C379; }");
  vbox->addWidget(m_progressBar);

  m_statusLabel = new QLabel("READY", this);
  m_statusLabel->setAlignment(Qt::AlignCenter);
  m_statusLabel->setStyleSheet(
      "font-size: 16px; font-weight: bold; color: #61AFEF;");
  vbox->addWidget(m_statusLabel);

  m_startBtn = new QPushButton("START CALIBRATION", this);
  m_startBtn->setFixedHeight(50);
  m_startBtn->setStyleSheet(
      "QPushButton { background: #3A5F3A; color: #98C379; border: 1px solid "
      "#5A8F5A; font-size: 16px; font-weight: bold; border-radius: 6px; }"
      "QPushButton:hover { background: #4A7A4A; }"
      "QPushButton:disabled { background: #252525; color: #4B5263; "
      "border-color: #3A3A3A; }");
  connect(m_startBtn, &QPushButton::clicked, this,
          &CalibrationWidget::onStartClicked);
  vbox->addWidget(m_startBtn);

  m_cancelBtn = new QPushButton("CANCEL CALIBRATION", this);
  m_cancelBtn->setFixedHeight(50);
  m_cancelBtn->setVisible(false);
  m_cancelBtn->setStyleSheet(
      "QPushButton { background: #5F3A3A; color: #E06C75; border: 1px solid "
      "#8F5A5A; font-size: 16px; font-weight: bold; border-radius: 6px; }"
      "QPushButton:hover { background: #7A4A4A; }");
  connect(m_cancelBtn, &QPushButton::clicked, this,
          &CalibrationWidget::onCancelClicked);
  vbox->addWidget(m_cancelBtn);

  layout->addWidget(group);
  layout->addStretch();
}

void CalibrationWidget::setProtocol(DroneProtocol *protocol) {
  m_protocol = protocol;
  connect(m_protocol, &DroneProtocol::statusReceived, this,
          &CalibrationWidget::onStatusReceived);
}

void CalibrationWidget::onStartClicked() {
  if (!m_protocol)
    return;

  m_startBtn->setEnabled(false);
  m_cancelBtn->setVisible(true);
  m_progressBar->setValue(0);
  m_statusLabel->setText("INITIALIZING...");

  // Send CMD_CALIBRATE_GYR (0x0006)
  uint32_t now = static_cast<uint32_t>(QDateTime::currentMSecsSinceEpoch());
  uint8_t id = 42;

  QByteArray pkt;
  pkt.append(static_cast<char>(0x56)); // sync
  pkt.append(static_cast<char>(0x31)); // type 3 (command), ver 1
  pkt.append(static_cast<char>(0x03)); // length 3 (cmd_id[2] + argc[1])
  pkt.append(static_cast<char>(id));   // dev_id
  pkt.append(reinterpret_cast<const char *>(&now), 4);

  uint16_t cmd_id = 0x0006;
  pkt.append(reinterpret_cast<const char *>(&cmd_id), 2);
  pkt.append(static_cast<char>(0x00)); // argc = 0

  uint32_t crc = CRC32::calculate(
      reinterpret_cast<const uint8_t *>(pkt.constData()), pkt.size());
  pkt.append(reinterpret_cast<const char *>(&crc), 4);

  emit commandRequested(pkt);
}

void CalibrationWidget::onCancelClicked() {
  if (!m_protocol)
    return;

  // Send CMD_CANCEL_CALIBRATION (0x0009)
  uint32_t now = static_cast<uint32_t>(QDateTime::currentMSecsSinceEpoch());
  uint8_t id = 42;

  QByteArray pkt;
  pkt.append(static_cast<char>(0x56)); // sync
  pkt.append(static_cast<char>(0x31)); // type 3 (command), ver 1
  pkt.append(static_cast<char>(0x03)); // length 3 (cmd_id[2] + argc[1])
  pkt.append(static_cast<char>(id));   // dev_id
  pkt.append(reinterpret_cast<const char *>(&now), 4);

  uint16_t cmd_id = 0x0009;
  pkt.append(reinterpret_cast<const char *>(&cmd_id), 2);
  pkt.append(static_cast<char>(0x00)); // argc = 0

  uint32_t crc = CRC32::calculate(
      reinterpret_cast<const uint8_t *>(pkt.constData()), pkt.size());
  pkt.append(reinterpret_cast<const char *>(&crc), 4);

  emit commandRequested(pkt);

  // Reset UI
  m_startBtn->setEnabled(true);
  m_cancelBtn->setVisible(false);
  m_statusLabel->setText("READY");
  m_progressBar->setValue(0);
}

void CalibrationWidget::onStatusReceived(const QString &msg) {
  if (msg.contains("PROGRESS")) {
    m_statusLabel->setText(msg);
    // Extract progress % from "CALIBRATION PROGRESS: 50.00%"
    static QRegularExpression re("(-?\\d+\\.?\\d+)");
    QRegularExpressionMatch match = re.match(msg);
    if (match.hasMatch()) {
      float p = match.captured(1).toFloat();
      m_progressBar->setValue(static_cast<int>(p));
    }
  } else if (msg == "STANDBY") {
    if (m_progressBar->value() > 0) { // If we were calibrating
      m_progressBar->setValue(100);
      m_statusLabel->setText("CALIBRATION SUCCESSFUL");
      m_startBtn->setEnabled(true);
      m_cancelBtn->setVisible(false);
    }
  } else if (msg == "CALIBRATING") {
    m_statusLabel->setText("CALIBRATING...");
  }
}
