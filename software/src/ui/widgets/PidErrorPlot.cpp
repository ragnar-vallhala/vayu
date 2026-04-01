#include "PidErrorPlot.h"
#include <QDoubleValidator>
#include <QHBoxLayout>
#include <QLineEdit>

PidErrorPlot::PidErrorPlot(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setSpacing(10);
  layout->setContentsMargins(15, 15, 15, 15);

  // Header with back button
  auto *header = new QHBoxLayout();
  auto *backBtn = new QPushButton("← BACK", this);
  backBtn->setFixedSize(80, 30);
  backBtn->setStyleSheet(
      "QPushButton { background: #2C313A; color: #ABB2BF; border: 1px solid "
      "#3E4452; border-radius: 4px; font-weight: bold; }"
      "QPushButton:hover { background: #3E4452; }");
  connect(backBtn, &QPushButton::clicked, this,
          &PidErrorPlot::backToHomeRequested);

  auto *title = new QLabel("PID ERROR ANALYSIS", this);
  title->setStyleSheet("font-size: 18px; font-weight: bold; color: #61AFEF;");

  header->addWidget(backBtn);
  header->addSpacing(20);
  header->addWidget(title);
  header->addStretch();

  // Gyro scale input
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

  layout->addLayout(header);

  // Stats row
  auto statsLayout = new QHBoxLayout();
  auto createValueLabel = [&](const QString &name, const QString &color) {
    auto *box = new QVBoxLayout();
    auto *l = new QLabel(name, this);
    l->setStyleSheet(
        QString("color: %1; font-size: 12px; font-weight: bold;").arg(color));
    auto *v = new QLabel("0.000", this);
    v->setStyleSheet(QString("color: %1; font-size: 24px; font-family: "
                             "Monospace; font-weight: bold;")
                         .arg(color));
    box->addWidget(l);
    box->addWidget(v);
    statsLayout->addLayout(box);
    return v;
  };

  m_rollVal = createValueLabel("ROLL ERROR", "#E06C75");
  m_pitchVal = createValueLabel("PITCH ERROR", "#98C379");
  m_yawVal = createValueLabel("YAW ERROR", "#61AFEF");
  layout->addLayout(statsLayout);

  // Graph
  m_graph = new RealTimeGraph(this, 6);
  m_graph->setColor(0, QColor("#E06C75")); // Roll Error
  m_graph->setColor(1, QColor("#98C379")); // Pitch Error
  m_graph->setColor(2, QColor("#61AFEF")); // Yaw Error
  m_graph->setColor(3, QColor("#BE5046")); // Scaled Roll Gyro (Darker Red)
  m_graph->setColor(4, QColor("#7FB069")); // Scaled Pitch Gyro (Darker Green)
  m_graph->setColor(5, QColor("#4078BF")); // Scaled Yaw Gyro (Darker Blue)
  m_graph->setPenStyle(3, Qt::DashLine);
  m_graph->setPenStyle(4, Qt::DashLine);
  m_graph->setPenStyle(5, Qt::DashLine);
  m_graph->setWindowSeconds(10);
  layout->addWidget(m_graph, 1);

  auto *hint = new QLabel("Real-time PID error tracking (deg)", this);
  hint->setStyleSheet("color: #5C6370; font-style: italic; font-size: 11px;");
  layout->addWidget(hint);

  setStyleSheet("background: #21252B; color: #ABB2BF;");
}

void PidErrorPlot::setProtocol(DroneProtocol *protocol) {
  if (m_protocol) {
    disconnect(m_protocol, &DroneProtocol::pidErrorReceived, this,
               &PidErrorPlot::onPidErrorReceived);
    disconnect(m_protocol, &DroneProtocol::imuReceived, this,
               &PidErrorPlot::onImuReceived);
  }
  m_protocol = protocol;
  if (m_protocol) {
    connect(m_protocol, &DroneProtocol::pidErrorReceived, this,
            &PidErrorPlot::onPidErrorReceived);
    connect(m_protocol, &DroneProtocol::imuReceived, this,
            &PidErrorPlot::onImuReceived);
  }
}

void PidErrorPlot::onPidErrorReceived(const PidErrorData &data) {
  m_graph->appendData(data.roll_error, 0);
  m_graph->appendData(data.pitch_error, 1);
  m_graph->appendData(data.yaw_error, 2);

  m_rollVal->setText(
      QString::number(static_cast<double>(data.roll_error), 'f', 3));
  m_pitchVal->setText(
      QString::number(static_cast<double>(data.pitch_error), 'f', 3));
  m_yawVal->setText(
      QString::number(static_cast<double>(data.yaw_error), 'f', 3));
}

void PidErrorPlot::onImuReceived(const ImuData &data) {
  m_graph->appendData(data.gyr[0] * m_gyroScale, 3);
  m_graph->appendData(data.gyr[1] * m_gyroScale, 4);
  m_graph->appendData(data.gyr[2] * m_gyroScale, 5);
}
