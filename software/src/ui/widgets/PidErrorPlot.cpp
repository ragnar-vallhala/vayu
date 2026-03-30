#include "PidErrorPlot.h"
#include <QHBoxLayout>

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
  m_graph = new RealTimeGraph(this, 3);
  m_graph->setColor(0, QColor("#E06C75")); // Roll
  m_graph->setColor(1, QColor("#98C379")); // Pitch
  m_graph->setColor(2, QColor("#61AFEF")); // Yaw
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
  }
  m_protocol = protocol;
  if (m_protocol) {
    connect(m_protocol, &DroneProtocol::pidErrorReceived, this,
            &PidErrorPlot::onPidErrorReceived);
  }
}

void PidErrorPlot::onPidErrorReceived(const PidErrorData &data) {
  m_graph->appendData(data.roll_error, 0);
  m_graph->appendData(data.pitch_error, 1);
  m_graph->appendData(data.yaw_error, 2);

  m_rollVal->setText(QString::number(data.roll_error, 'f', 3));
  m_pitchVal->setText(QString::number(data.pitch_error, 'f', 3));
  m_yawVal->setText(QString::number(data.yaw_error, 'f', 3));
}
