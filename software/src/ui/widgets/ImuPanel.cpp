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
  auto *layout = new QGridLayout(this);
  layout->setSpacing(8);

  const char *axisNames[] = {"X", "Y", "Z"};
  const char *colors[] = {"#FF6B6B", "#4ECDC4", "#FFE66D"};

  for (int i = 0; i < 3; ++i) {
    auto *rowLayout = new QVBoxLayout();
    rowLayout->setSpacing(2);

    auto *labelLayout = new QHBoxLayout();
    auto *nameLabel =
        new QLabel(QString("%1 [%2]").arg(axisNames[i], unit), this);
    nameLabel->setStyleSheet("color: #8EAEC8; font-size: 11px;");

    m_labels[i] = new QLabel("—", this);
    m_labels[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    m_labels[i]->setStyleSheet("font-family: 'Monospace'; font-size: 15px; "
                               "font-weight: bold; color: #E8F0FE;");
    m_labels[i]->setMinimumWidth(90);

    labelLayout->addWidget(nameLabel);
    labelLayout->addStretch();
    labelLayout->addWidget(m_labels[i]);

    m_graphs[i] = new RealTimeGraph(this);
    m_graphs[i]->setColor(QColor(colors[i]));
    m_graphs[i]->setFixedHeight(60);

    rowLayout->addLayout(labelLayout);
    rowLayout->addWidget(m_graphs[i]);

    layout->addLayout(rowLayout, i, 0);
  }
}

void ImuAxisGroup::setValues(float x, float y, float z) {
  float vals[3] = {x, y, z};
  for (int i = 0; i < 3; ++i) {
    m_labels[i]->setText(QString::number(static_cast<double>(vals[i]), 'f', 4));
    m_graphs[i]->appendData(vals[i]);
  }
}

void ImuAxisGroup::setWindowSeconds(int seconds) {
  for (int i = 0; i < 3; ++i) {
    m_graphs[i]->setWindowSeconds(seconds);
  }
}

void ImuAxisGroup::setDropoutRate(double rate) {
  for (int i = 0; i < 3; ++i) {
    m_graphs[i]->setDropoutRate(rate);
  }
}

// --------------- ImuPanel ---------------------------------------------------

ImuPanel::ImuPanel(QWidget *parent) : QGroupBox("IMU — BMX160", parent) {
  auto *layout = new QHBoxLayout(this);
  layout->setSpacing(10);

  m_acc = new ImuAxisGroup("Accelerometer", "m/s²", this);
  m_gyr = new ImuAxisGroup("Gyroscope", "°/s", this);
  m_mag = new ImuAxisGroup("Magnetometer", "µT", this);

  layout->addWidget(m_acc);
  layout->addWidget(m_gyr);
  layout->addWidget(m_mag);

  auto *tempBox = new QGroupBox("Temperature", this);
  auto *tl = new QVBoxLayout(tempBox);
  tl->setSpacing(8);

  m_temp = new QLabel("—", tempBox);
  m_temp->setAlignment(Qt::AlignCenter);
  m_temp->setStyleSheet("font-family: 'Monospace'; font-size: 14px; "
                        "font-weight: bold; color: #FFC66D;");

  m_tempGraph = new RealTimeGraph(tempBox);
  m_tempGraph->setMode(RealTimeGraph::Mode::HorizontalBar);
  m_tempGraph->setColor(QColor("#FFC66D"));
  m_tempGraph->setFixedHeight(250);

  tl->addStretch();
  tl->addWidget(m_temp);
  tl->addWidget(m_tempGraph);
  tl->addStretch();

  layout->addWidget(tempBox);
}

void ImuPanel::updateImu(const ImuData &data) {
  m_acc->setValues(data.acc[0], data.acc[1], data.acc[2]);
  m_gyr->setValues(data.gyr[0], data.gyr[1], data.gyr[2]);
  m_mag->setValues(data.mag[0], data.mag[1], data.mag[2]);
  m_temp->setText(
      QString("%1 °C").arg(static_cast<double>(data.tempC), 0, 'f', 2));
  m_tempGraph->appendData(data.tempC);
}

void ImuPanel::setSensor(const QString &name) {
  setTitle(QString("IMU — %1").arg(name));
}

void ImuPanel::setGraphWindow(int seconds) {
  m_acc->setWindowSeconds(seconds);
  m_gyr->setWindowSeconds(seconds);
  m_mag->setWindowSeconds(seconds);
  m_tempGraph->setWindowSeconds(seconds);
}

void ImuPanel::setGraphDropout(double rate) {
  m_acc->setDropoutRate(rate);
  m_gyr->setDropoutRate(rate);
  m_mag->setDropoutRate(rate);
  m_tempGraph->setDropoutRate(rate);
}
