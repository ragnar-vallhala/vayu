#include "ImuPanel.h"
#include "SettingsWidget.h"

#include <QColor>
#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

// --------------- ImuAxisGroup -----------------------------------------------

ImuAxisGroup::ImuAxisGroup(const QString &title, const QString &unit,
                           QWidget *parent)
    : QGroupBox(title, parent) {
  auto *layout = new QHBoxLayout(this);
  layout->setSpacing(10);
  layout->setContentsMargins(8, 22, 8, 8); // Increased top margin for title

  // Left side: Labels
  auto *labelContainer = new QWidget(this);
  auto *labelLayout = new QVBoxLayout(labelContainer);
  labelLayout->setContentsMargins(0, 0, 0, 0);
  labelLayout->setSpacing(6);

  const char *axisNames[] = {"X", "Y", "Z"};
  const char *colors[] = {"#FF6B6B", "#4ECDC4", "#FFE66D"};

  bool isScalar = (unit == "°C");

  labelLayout->addStretch();
  for (int i = 0; i < (isScalar ? 1 : 3); ++i) {
    auto *row = new QHBoxLayout();

    auto *nameLabel =
        new QLabel(QString("<b>%1</b>").arg(axisNames[i]), labelContainer);
    nameLabel->setStyleSheet(
        QString("color: %1; font-size: 11px;").arg(colors[i]));
    nameLabel->setFixedWidth(15);
    if (isScalar)
      nameLabel->hide();

    m_labels[i] = new QLabel("—", labelContainer);
    m_labels[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    m_labels[i]->setStyleSheet("font-family: 'Monospace'; font-size: 13px; "
                               "color: #E8F0FE;");
    m_labels[i]->setMinimumWidth(80);

    row->addWidget(nameLabel);
    row->addWidget(m_labels[i]);
    labelLayout->addLayout(row);
  }

  if (!isScalar) {
    // Initialize hidden labels to avoid crashes if setValues is called with 3
    // values
    m_labels[1] = m_labels[1] ? m_labels[1] : new QLabel(this);
    m_labels[2] = m_labels[2] ? m_labels[2] : new QLabel(this);
  } else {
    // Hidden placeholders for scalar
    m_labels[1] = new QLabel(this);
    m_labels[1]->hide();
    m_labels[2] = new QLabel(this);
    m_labels[2]->hide();
  }

  labelLayout->addStretch();

  layout->addWidget(labelContainer);

  // Right side: Single Graph
  m_graph = new RealTimeGraph(this, isScalar ? 1 : 3);
  if (!isScalar) {
    for (int i = 0; i < 3; ++i) {
      m_graph->setColor(i, QColor(colors[i]));
    }
  } else {
    m_graph->setColor(0, QColor("#FFC66D")); // Temperature specific color
  }
  m_graph->setMinimumHeight(80);

  layout->addWidget(m_graph, 1);
}

void ImuAxisGroup::setValues(float x, float y, float z) {
  float vals[3] = {x, y, z};
  for (int i = 0; i < 3; ++i) {
    if (m_labels[i]->isHidden())
      continue;
    m_labels[i]->setText(QString::number(static_cast<double>(vals[i]), 'f', 3));
    if (i < m_graph->numSeries()) {
      m_graph->appendData(vals[i], i);
    }
  }
}

void ImuAxisGroup::setWindowSeconds(int seconds) {
  m_graph->setWindowSeconds(seconds);
}

void ImuAxisGroup::setDropoutRate(double rate) {
  m_graph->setDropoutRate(rate);
}

// --------------- ImuPanel ---------------------------------------------------

ImuPanel::ImuPanel(QWidget *parent) : QGroupBox("IMU — BMX160", parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setSpacing(6);
  layout->setContentsMargins(6, 12, 6, 6);

  m_acc = new ImuAxisGroup("Accelerometer (m/s²)", "m/s²", this);
  m_gyr = new ImuAxisGroup("Gyroscope (°/s)", "°/s", this);
  m_mag = new ImuAxisGroup("Magnetometer (µT)", "µT", this);
  m_temp = new ImuAxisGroup("Temperature (°C)", "°C", this);

  // Customizing temp graph to be a horizontal bar or just single line
  // Let's keep it as a line for now to be consistent with 4 horizontal graphs
  // But we can hide extra labels if we want.

  layout->addWidget(m_acc);
  layout->addWidget(m_gyr);
  layout->addWidget(m_mag);
  layout->addWidget(m_temp);
}

void ImuPanel::updateImu(const ImuData &data) {
  m_acc->setValues(data.acc[0], data.acc[1], data.acc[2]);
  m_gyr->setValues(data.gyr[0], data.gyr[1], data.gyr[2]);
  m_mag->setValues(data.mag[0], data.mag[1], data.mag[2]);
  m_temp->setValues(data.tempC, 0, 0); // Only X used for temp
}

void ImuPanel::setSensor(const QString &name) {
  setTitle(QString("IMU — %1").arg(name));
}

void ImuPanel::setGraphWindow(int seconds) {
  m_acc->setWindowSeconds(seconds);
  m_gyr->setWindowSeconds(seconds);
  m_mag->setWindowSeconds(seconds);
  m_temp->setWindowSeconds(seconds);
}

void ImuPanel::setGraphDropout(double rate) {
  m_acc->setDropoutRate(rate);
  m_gyr->setDropoutRate(rate);
  m_mag->setDropoutRate(rate);
  m_temp->setDropoutRate(rate);
}
