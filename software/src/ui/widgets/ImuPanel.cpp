#include "ImuPanel.h"

#include <QFont>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

// --------------- ImuAxisGroup -----------------------------------------------

ImuAxisGroup::ImuAxisGroup(const QString &title, const QString &unit,
                           QWidget *parent)
    : QGroupBox(title, parent) {
  auto *layout = new QGridLayout(this);
  layout->setSpacing(4);

  const char *axisNames[] = {"X", "Y", "Z"};
  for (int i = 0; i < 3; ++i) {
    auto *nameLabel =
        new QLabel(QString("%1 [%2]").arg(axisNames[i], unit), this);
    nameLabel->setStyleSheet("color: #8EAEC8; font-size: 11px;");

    m_labels[i] = new QLabel("—", this);
    m_labels[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    m_labels[i]->setStyleSheet("font-family: 'Monospace'; font-size: 15px; "
                               "font-weight: bold; color: #E8F0FE;");
    m_labels[i]->setMinimumWidth(90);

    layout->addWidget(nameLabel, i, 0);
    layout->addWidget(m_labels[i], i, 1);
  }
}

void ImuAxisGroup::setValues(float x, float y, float z) {
  float vals[3] = {x, y, z};
  for (int i = 0; i < 3; ++i) {
    m_labels[i]->setText(QString::number(static_cast<double>(vals[i]), 'f', 4));
  }
}

// --------------- ImuPanel ---------------------------------------------------

ImuPanel::ImuPanel(QWidget *parent) : QGroupBox("IMU — BMX160", parent) {
  auto *layout = new QHBoxLayout(this);

  m_acc = new ImuAxisGroup("Accelerometer", "m/s²", this);
  m_gyr = new ImuAxisGroup("Gyroscope", "°/s", this);
  m_mag = new ImuAxisGroup("Magnetometer", "µT", this);

  layout->addWidget(m_acc);
  layout->addWidget(m_gyr);
  layout->addWidget(m_mag);

  auto *tempBox = new QGroupBox("Temperature", this);
  auto *tl = new QVBoxLayout(tempBox);
  m_temp = new QLabel("—", tempBox);
  m_temp->setAlignment(Qt::AlignCenter);
  m_temp->setStyleSheet("font-family: 'Monospace'; font-size: 14px; "
                        "font-weight: bold; color: #FFC66D;");
  tl->addWidget(m_temp);
  layout->addWidget(tempBox);
}

void ImuPanel::updateImu(const ImuData &data) {
  m_acc->setValues(data.acc[0], data.acc[1], data.acc[2]);
  m_gyr->setValues(data.gyr[0], data.gyr[1], data.gyr[2]);
  m_mag->setValues(data.mag[0], data.mag[1], data.mag[2]);
  m_temp->setText(
      QString("%1 °C").arg(static_cast<double>(data.tempC), 0, 'f', 2));
}

void ImuPanel::setSensor(const QString &name) {
  setTitle(QString("IMU — %1").arg(name));
}
