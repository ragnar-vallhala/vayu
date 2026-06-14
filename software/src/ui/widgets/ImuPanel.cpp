#include "ImuPanel.h"
#include "SettingsWidget.h"

#include "core/CsvExport.h"
#include "core/Notify.h"
#include "core/ui/Buttons.h"

#include <QColor>
#include <QDateTime>
#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
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
                               "color: #E8F0FE; font-weight: bold;");
    m_labels[i]->setFixedWidth(90);

    m_stdLabels[i] = new QLabel("± σ —", labelContainer);
    m_stdLabels[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    m_stdLabels[i]->setStyleSheet("font-family: 'Monospace'; font-size: 10px; "
                                  "color: #888888;");
    m_stdLabels[i]->setFixedWidth(90);

    row->addWidget(nameLabel);
    auto *valsVBox = new QVBoxLayout();
    valsVBox->setSpacing(0);
    valsVBox->addWidget(m_labels[i]);
    valsVBox->addWidget(m_stdLabels[i]);
    row->addLayout(valsVBox);
    labelLayout->addLayout(row);
  }

  if (!isScalar) {
    // Initialize hidden labels to avoid crashes if setValues is called with 3
    // values
    for (int i = 1; i < 3; ++i) {
      if (!m_labels[i])
        m_labels[i] = new QLabel(this);
      if (!m_stdLabels[i])
        m_stdLabels[i] = new QLabel(this);
    }
  } else {
    // Hidden placeholders for scalar
    for (int i = 1; i < 3; ++i) {
      m_labels[i] = new QLabel(this);
      m_labels[i]->hide();
      m_stdLabels[i] = new QLabel(this);
      m_stdLabels[i]->hide();
    }
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
  // Mockup parity: rolling-σ traces on a right axis + vehicle-state band.
  if (!isScalar) {
    m_graph->setSigmaAxis(true, m_sigmaMax);
    m_graph->setStateBandEnabled(true);
  }

  layout->addWidget(m_graph, 1);
}

void ImuAxisGroup::setValues(float x, float y, float z) {
  float vals[3] = {x, y, z};
  for (int i = 0; i < 3; ++i) {
    if (m_labels[i]->isHidden())
      continue;
    m_labels[i]->setText(QString::number(static_cast<double>(vals[i]), 'f', 3));

    m_stats[i].push(vals[i]);
    float sd = m_stats[i].stdDev();
    m_stdLabels[i]->setText(
        QString("± σ %1").arg(static_cast<double>(sd), 0, 'f', 3));

    if (i < m_graph->numSeries()) {
      m_graph->appendData(vals[i], i);
      // Plot σ on the right axis, growing the scale to fit (mockup stdMax).
      m_graph->appendSigma(sd, i);
      if (sd * 1.3f > m_sigmaMax) {
        m_sigmaMax = sd * 1.3f;
        m_graph->setSigmaAxis(true, m_sigmaMax);
      }
    }
  }
}

void ImuAxisGroup::setUnavailable() {
  for (int i = 0; i < 3; ++i) {
    if (m_labels[i] && !m_labels[i]->isHidden()) m_labels[i]->setText("-");
    if (m_stdLabels[i] && !m_stdLabels[i]->isHidden())
      m_stdLabels[i]->setText("± σ -");
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
  // Mockup parity: the panel is a left column (acc/gyr/mag graphs + a vehicle-
  // state legend + CSV export) beside a right-hand temp-gauge column carrying
  // the device-temp and battery vertical gauges. Temperature is shown as a
  // gauge here (not a time-series graph) to match the mockup.
  auto *outer = new QHBoxLayout(this);
  outer->setSpacing(8);
  outer->setContentsMargins(6, 12, 6, 6);

  auto *col = new QVBoxLayout();
  col->setSpacing(6);

  m_acc = new ImuAxisGroup("Accelerometer (m/s²)", "m/s²", this);
  m_gyr = new ImuAxisGroup("Gyroscope (°/s)", "°/s", this);
  m_mag = new ImuAxisGroup("Magnetometer (µT)", "µT", this);

  col->addWidget(m_acc);
  col->addWidget(m_gyr);
  col->addWidget(m_mag);

  // Vehicle-state legend (mockup .status-key): maps the graph state-band
  // colours to flight states.
  auto *key = new QWidget(this);
  auto *keyLayout = new QHBoxLayout(key);
  keyLayout->setContentsMargins(2, 0, 2, 0);
  keyLayout->setSpacing(10);
  auto *keyCap = new QLabel(tr("Vehicle state:"), key);
  keyCap->setStyleSheet("color: #8A92A6; font-size: 10px;");
  keyLayout->addWidget(keyCap);
  const struct {
    const char *label;
    const char *color;
  } states[] = {{"Init / Prearm", "#61AFEF"},
                {"Standby", "#98C379"},
                {"Armed", "#E06C75"},
                {"Failsafe", "#E0822E"}};
  for (const auto &s : states) {
    auto *swatch = new QLabel(key);
    swatch->setFixedSize(10, 10);
    swatch->setStyleSheet(QString("background:%1; border-radius:2px;").arg(s.color));
    auto *txt = new QLabel(tr(s.label), key);
    txt->setStyleSheet("color: #ABB2BF; font-size: 10px;");
    keyLayout->addWidget(swatch);
    keyLayout->addWidget(txt);
  }
  keyLayout->addStretch();
  col->addWidget(key);

  // Footer: CSV export button. Writes the three axis groups' current
  // ring-buffer contents to one file with a shared timestamp axis.
  auto *footer = new QHBoxLayout();
  footer->addStretch();
  auto *exportBtn = new ui::GhostButton(tr("Export CSV"), this);
  exportBtn->setToolTip(
      tr("Save the currently-buffered IMU traces (acc/gyr/mag) to a CSV file"));
  connect(exportBtn, &QPushButton::clicked, this, [this] {
    const QString stamp =
        QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss");
    const QString path = CsvExport::promptAndWriteCombined(
        this, QString("imu-%1.csv").arg(stamp),
        {
          {m_acc->graph(), {"acc_x", "acc_y", "acc_z"}},
          {m_gyr->graph(), {"gyr_x", "gyr_y", "gyr_z"}},
          {m_mag->graph(), {"mag_x", "mag_y", "mag_z"}},
        });
    if (!path.isEmpty()) {
      Notify::ok(this, tr("Wrote %1").arg(path));
    }
  });
  footer->addWidget(exportBtn);
  col->addLayout(footer);

  outer->addLayout(col, 1);

  // Right-hand vertical gauges (mockup .temp-gauge column).
  auto *gauges = new QHBoxLayout();
  gauges->setSpacing(8);
  m_tempGauge = new VGauge(tr("DEVICE\nTEMP"), 0, 80, "°C", VGauge::Temp, this);
  m_battGauge = new VGauge(tr("BATTERY"), 0, 100, "%", VGauge::Battery, this);
  // No battery telemetry source yet → the gauge reads "-" (set by VGauge's
  // ctor). setBattery() will light it up once a SYSTEM_STATUS feed exists.
  gauges->addWidget(m_tempGauge);
  gauges->addWidget(m_battGauge);
  auto *gaugeWrap = new QWidget(this);
  gaugeWrap->setLayout(gauges);
  gaugeWrap->setFixedWidth(116);
  outer->addWidget(gaugeWrap);
}

void ImuPanel::updateImu(const ImuData &data, bool available) {
  if (!available) {
    // No live telemetry: numeric labels + temp gauge read "-"; the graphs go
    // NA on their own once their traces age out of the window.
    m_acc->setUnavailable();
    m_gyr->setUnavailable();
    m_mag->setUnavailable();
    m_tempGauge->setUnavailable();
    return;
  }
  m_acc->setValues(data.acc[0], data.acc[1], data.acc[2]);
  m_gyr->setValues(data.gyr[0], data.gyr[1], data.gyr[2]);
  m_mag->setValues(data.mag[0], data.mag[1], data.mag[2]);
  m_tempGauge->setValue(data.tempC);
  // Advance the state band one cell per update (mockup .g-status cadence).
  m_acc->pushState(m_state);
  m_gyr->pushState(m_state);
  m_mag->pushState(m_state);
}

void ImuPanel::setSensor(const QString &name) {
  setTitle(QString("IMU — %1").arg(name));
}

void ImuPanel::setBattery(double pct) { m_battGauge->setValue(pct); }

void ImuPanel::setGraphWindow(int seconds) {
  m_acc->setWindowSeconds(seconds);
  m_gyr->setWindowSeconds(seconds);
  m_mag->setWindowSeconds(seconds);
}

void ImuPanel::setGraphDropout(double rate) {
  m_acc->setDropoutRate(rate);
  m_gyr->setDropoutRate(rate);
  m_mag->setDropoutRate(rate);
}
