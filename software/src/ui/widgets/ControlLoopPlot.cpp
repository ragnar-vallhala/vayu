#include "ControlLoopPlot.h"
#include <QDoubleValidator>
#include <QGridLayout>
#include <QScrollArea>
#include <QSplitter>

ControlLoopPlot::ControlLoopPlot(QWidget *parent) : QWidget(parent) {
  auto *mainLayout = new QVBoxLayout(this);
  mainLayout->setSpacing(10);
  mainLayout->setContentsMargins(15, 15, 15, 15);

  auto *header = new QHBoxLayout();
  auto *backBtn = new QPushButton("← BACK", this);
  backBtn->setFixedSize(80, 30);
  backBtn->setStyleSheet(
      "QPushButton { background: #2C313A; color: #ABB2BF; border: 1px solid "
      "#3E4452; border-radius: 4px; font-weight: bold; }"
      "QPushButton:hover { background: #3E4452; }");
  connect(backBtn, &QPushButton::clicked, this,
          &ControlLoopPlot::backToHomeRequested);

  auto *title = new QLabel("CONTROL LOOP DASHBOARD", this);
  title->setStyleSheet("font-size: 18px; font-weight: bold; color: #61AFEF;");

  header->addWidget(backBtn);
  header->addSpacing(20);
  header->addWidget(title);
  header->addStretch();

  auto *scaleLabel = new QLabel("Gyro Scale:", this);
  scaleLabel->setStyleSheet("color: #ABB2BF; font-size: 12px;");
  auto *scaleEdit = new QLineEdit("1.0", this);
  scaleEdit->setFixedWidth(60);
  scaleEdit->setValidator(new QDoubleValidator(0.0, 1000.0, 4, this));
  scaleEdit->setStyleSheet(
      "QLineEdit { background: #2C313A; color: #ABB2BF; border: 1px solid "
      "#3E4452; border-radius: 4px; padding: 2px; }"
      "QLineEdit:focus { border-color: #61AFEF; }");
  connect(scaleEdit, &QLineEdit::textChanged, this,
          [this](const QString &text) { m_gyroScale = text.toFloat(); });

  header->addWidget(scaleLabel);
  header->addSpacing(5);
  header->addWidget(scaleEdit);

  mainLayout->addLayout(header);

  auto *scrollArea = new QScrollArea(this);
  scrollArea->setWidgetResizable(true);
  scrollArea->setStyleSheet(
      "QScrollArea { border: none; background: #21252B; }");

  auto *scrollContent = new QWidget();
  auto *scrollLayout = new QVBoxLayout(scrollContent);
  scrollLayout->setContentsMargins(0, 0, 0, 0);

  auto *vSplitter = new QSplitter(Qt::Vertical, scrollContent);
  vSplitter->setStyleSheet(
      "QSplitter::handle { background: #3E4452; height: 4px; }");

  auto *topHSplitter = new QSplitter(Qt::Horizontal, vSplitter);
  topHSplitter->setStyleSheet(
      "QSplitter::handle { background: #3E4452; width: 4px; }");

  auto *botHSplitter = new QSplitter(Qt::Horizontal, vSplitter);
  botHSplitter->setStyleSheet(
      "QSplitter::handle { background: #3E4452; width: 4px; }");

  scrollLayout->addWidget(vSplitter);

  auto createSection = [&](const QString &titleText,
                           QHBoxLayout **outStatsLayout,
                           RealTimeGraph **outGraph, int numTraces) {
    auto *section = new QWidget();
    section->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto *layout = new QVBoxLayout(section);
    layout->setContentsMargins(10, 10, 10, 10);

    auto *lbl = new QLabel(titleText, section);
    lbl->setStyleSheet("font-size: 14px; font-weight: bold; color: #ABB2BF;");
    layout->addWidget(lbl);

    *outStatsLayout = new QHBoxLayout();
    layout->addLayout(*outStatsLayout);

    *outGraph = new RealTimeGraph(section, numTraces);
    (*outGraph)->setMinimumHeight(150);
    (*outGraph)->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    (*outGraph)->setWindowSeconds(10);
    layout->addWidget(*outGraph);

    return section;
  };

  auto createValueLabel = [this](QHBoxLayout *parentLayout, const QString &name,
                                 const QString &color, QLabel **outLabel) {
    auto *box = new QVBoxLayout();
    auto *l = new QLabel(name, this);
    l->setStyleSheet(
        QString("color: %1; font-size: 11px; font-weight: bold;").arg(color));
    *outLabel = new QLabel("0.000", this);
    (*outLabel)->setStyleSheet(
        QString("color: %1; font-size: 18px; font-family: Monospace; "
                "font-weight: bold;")
            .arg(color));
    box->addWidget(l);
    box->addWidget(*outLabel);
    box->setAlignment(Qt::AlignLeft);
    parentLayout->addLayout(box);
  };

  QHBoxLayout *angleStats;
  auto *angleSec =
      createSection("OUTER LOOP: ANGLE ERROR", &angleStats, &m_angleGraph, 3);
  createValueLabel(angleStats, "ROLL ERR", "#E06C75", &m_rollAngleErrVal);
  createValueLabel(angleStats, "PITCH ERR", "#98C379", &m_pitchAngleErrVal);
  createValueLabel(angleStats, "YAW ERR", "#61AFEF", &m_yawAngleErrVal);
  createValueLabel(angleStats, "ROLL SP", "#D19A66", &m_rollAngleSpVal);
  createValueLabel(angleStats, "PITCH SP", "#C678DD", &m_pitchAngleSpVal);
  createValueLabel(angleStats, "YAW SP", "#56B6C2", &m_yawAngleSpVal);
  angleStats->addStretch();
  m_angleGraph->setColor(0, QColor("#E06C75"));
  m_angleGraph->setColor(1, QColor("#98C379"));
  m_angleGraph->setColor(2, QColor("#61AFEF"));
  topHSplitter->addWidget(angleSec);

  QHBoxLayout *rateStats;
  auto *rateSec = createSection("INNER LOOP: RATE ERROR & GYRO", &rateStats,
                                &m_rateGraph, 6);
  createValueLabel(rateStats, "ROLL RATE ERR", "#E06C75", &m_rollRateErrVal);
  createValueLabel(rateStats, "PITCH RATE ERR", "#98C379", &m_pitchRateErrVal);
  createValueLabel(rateStats, "YAW RATE ERR", "#61AFEF", &m_yawRateErrVal);
  createValueLabel(rateStats, "ROLL RATE SP", "#D19A66", &m_rollRateSpVal);
  createValueLabel(rateStats, "PITCH RATE SP", "#C678DD", &m_pitchRateSpVal);
  createValueLabel(rateStats, "YAW RATE SP", "#56B6C2", &m_yawRateSpVal);
  rateStats->addStretch();
  m_rateGraph->setColor(0, QColor("#E06C75"));
  m_rateGraph->setColor(1, QColor("#98C379"));
  m_rateGraph->setColor(2, QColor("#61AFEF"));
  m_rateGraph->setColor(3, QColor("#BE5046")); // gyro
  m_rateGraph->setColor(4, QColor("#7FB069"));
  m_rateGraph->setColor(5, QColor("#4078BF"));
  m_rateGraph->setPenStyle(3, Qt::DashLine);
  m_rateGraph->setPenStyle(4, Qt::DashLine);
  m_rateGraph->setPenStyle(5, Qt::DashLine);
  topHSplitter->addWidget(rateSec);

  vSplitter->addWidget(topHSplitter);

  QHBoxLayout *outStats;
  auto *outSec =
      createSection("CONTROLLER OUTPUTS", &outStats, &m_outputGraph, 4);
  createValueLabel(outStats, "ROLL OUT", "#E06C75", &m_rollOutVal);
  createValueLabel(outStats, "PITCH OUT", "#98C379", &m_pitchOutVal);
  createValueLabel(outStats, "YAW OUT", "#61AFEF", &m_yawOutVal);
  createValueLabel(outStats, "THRO. OUT", "#E5C07B", &m_throttleOutVal);
  outStats->addStretch();
  m_outputGraph->setColor(0, QColor("#E06C75"));
  m_outputGraph->setColor(1, QColor("#98C379"));
  m_outputGraph->setColor(2, QColor("#61AFEF"));
  m_outputGraph->setColor(3, QColor("#E5C07B"));
  botHSplitter->addWidget(outSec);

  QHBoxLayout *dtStats;
  auto *dtSec = createSection("LOOP TIME (dt)", &dtStats, &m_dtGraph, 1);
  createValueLabel(dtStats, "dt (sec)", "#98C379", &m_dtVal);
  dtStats->addStretch();
  m_dtGraph->setColor(0, QColor("#98C379"));
  m_dtGraph->setDynamicYAxis(true);
  botHSplitter->addWidget(dtSec);

  vSplitter->addWidget(botHSplitter);

  scrollContent->setStyleSheet("background: #21252B;");
  scrollArea->setWidget(scrollContent);
  mainLayout->addWidget(scrollArea, 1);

  setStyleSheet("background: #21252B; color: #ABB2BF;");
}

void ControlLoopPlot::setProtocol(DroneProtocol *protocol) {
  if (m_protocol) {
    disconnect(m_protocol, &DroneProtocol::controlLoopDataReceived, this,
               &ControlLoopPlot::onControlLoopDataReceived);
    disconnect(m_protocol, &DroneProtocol::imuReceived, this,
               &ControlLoopPlot::onImuReceived);
  }
  m_protocol = protocol;
  if (m_protocol) {
    connect(m_protocol, &DroneProtocol::controlLoopDataReceived, this,
            &ControlLoopPlot::onControlLoopDataReceived);
    connect(m_protocol, &DroneProtocol::imuReceived, this,
            &ControlLoopPlot::onImuReceived);
  }
}

void ControlLoopPlot::onControlLoopDataReceived(const ControlLoopData &data) {
  // Update DT
  m_dtVal->setText(QString::number(static_cast<double>(data.dt), 'f', 5));
  m_dtGraph->appendData(data.dt, 0);

  // Angles
  m_angleGraph->appendData(data.roll_angle_error, 0);
  m_angleGraph->appendData(data.pitch_angle_error, 1);
  m_angleGraph->appendData(data.yaw_angle_error, 2);
  m_rollAngleErrVal->setText(
      QString::number(static_cast<double>(data.roll_angle_error), 'f', 3));
  m_pitchAngleErrVal->setText(
      QString::number(static_cast<double>(data.pitch_angle_error), 'f', 3));
  m_yawAngleErrVal->setText(
      QString::number(static_cast<double>(data.yaw_angle_error), 'f', 3));
  m_rollAngleSpVal->setText(
      QString::number(static_cast<double>(data.roll_angle_setpoint), 'f', 3));
  m_pitchAngleSpVal->setText(
      QString::number(static_cast<double>(data.pitch_angle_setpoint), 'f', 3));
  m_yawAngleSpVal->setText(
      QString::number(static_cast<double>(data.yaw_angle_setpoint), 'f', 3));

  // Rates
  m_rateGraph->appendData(data.roll_rate_error, 0);
  m_rateGraph->appendData(data.pitch_rate_error, 1);
  m_rateGraph->appendData(data.yaw_rate_error, 2);
  m_rollRateErrVal->setText(
      QString::number(static_cast<double>(data.roll_rate_error), 'f', 3));
  m_pitchRateErrVal->setText(
      QString::number(static_cast<double>(data.pitch_rate_error), 'f', 3));
  m_yawRateErrVal->setText(
      QString::number(static_cast<double>(data.yaw_rate_error), 'f', 3));
  m_rollRateSpVal->setText(
      QString::number(static_cast<double>(data.roll_rate_setpoint), 'f', 3));
  m_pitchRateSpVal->setText(
      QString::number(static_cast<double>(data.pitch_rate_setpoint), 'f', 3));
  m_yawRateSpVal->setText(
      QString::number(static_cast<double>(data.yaw_rate_setpoint), 'f', 3));

  // Outputs
  m_outputGraph->appendData(data.roll_output, 0);
  m_outputGraph->appendData(data.pitch_output, 1);
  m_outputGraph->appendData(data.yaw_output, 2);
  m_outputGraph->appendData(data.throttle_output, 3);
  m_rollOutVal->setText(
      QString::number(static_cast<double>(data.roll_output), 'f', 3));
  m_pitchOutVal->setText(
      QString::number(static_cast<double>(data.pitch_output), 'f', 3));
  m_yawOutVal->setText(
      QString::number(static_cast<double>(data.yaw_output), 'f', 3));
  m_throttleOutVal->setText(
      QString::number(static_cast<double>(data.throttle_output), 'f', 3));
}

void ControlLoopPlot::onImuReceived(const ImuData &data) {
  m_rateGraph->appendData(data.gyr[0] * m_gyroScale, 3);
  m_rateGraph->appendData(data.gyr[1] * m_gyroScale, 4);
  m_rateGraph->appendData(data.gyr[2] * m_gyroScale, 5);
}
