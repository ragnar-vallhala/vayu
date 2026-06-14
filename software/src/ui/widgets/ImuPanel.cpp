#include "ImuPanel.h"

#include "core/CsvExport.h"
#include "core/Notify.h"
#include "core/ui/Buttons.h"

#include <QColor>
#include <QDateTime>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace {
// Per-graph spec: title, unit, fixed Y range and σ-axis max (mockup data-min/
// data-max/data-stdmax). Colours match the X/Y/Z trace palette.
const char *const kAxisColors[3] = {"#FF6B6B", "#4ECDC4", "#FFE66D"};

RealTimeGraph *makeAxisGraph(QWidget *parent, const QString &title,
                             const QString &unit, float sigmaMax) {
  auto *g = new RealTimeGraph(parent, 3);
  for (int i = 0; i < 3; ++i) g->setColor(i, QColor(kAxisColors[i]));
  // Auto-scale the Y axis so the trace never saturates against fixed rails; the
  // tick labels follow the live range.
  g->setDynamicYAxis(true);
  g->setSigmaAxis(true, sigmaMax);  // grows to fit (RealTimeGraph::appendSigma)
  g->setStateBandEnabled(true);
  g->setTitle(title, unit);
  g->setSeriesLabels({"X", "Y", "Z"});
  g->setMinimumHeight(120);
  return g;
}
}  // namespace

ImuPanel::ImuPanel(QWidget *parent) : QWidget(parent) {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(6, 6, 6, 6);
  root->setSpacing(6);

  // ---- Header: title + Export CSV (mockup IMU TELEMETRY · Export CSV) ----
  auto *headerRow = new QHBoxLayout();
  // Sensor-agnostic by default; setSensor() appends the part name if telemetry
  // ever reports it (don't hardcode a specific IMU).
  m_header = new QLabel(tr("IMU Telemetry"), this);
  m_header->setStyleSheet(
      "color: #61AFEF; font-weight: bold; font-size: 13px;");
  headerRow->addWidget(m_header);
  headerRow->addStretch();
  auto *exportBtn = new ui::GhostButton(tr("Export CSV"), this);
  exportBtn->setToolTip(
      tr("Save the currently-buffered IMU traces (acc/gyr/mag) to a CSV file"));
  connect(exportBtn, &QPushButton::clicked, this, [this] {
    const QString stamp =
        QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss");
    const QString path = CsvExport::promptAndWriteCombined(
        this, QString("imu-%1.csv").arg(stamp),
        {
          {m_accG, {"acc_x", "acc_y", "acc_z"}},
          {m_gyrG, {"gyr_x", "gyr_y", "gyr_z"}},
          {m_magG, {"mag_x", "mag_y", "mag_z"}},
        });
    if (!path.isEmpty()) Notify::ok(this, tr("Wrote %1").arg(path));
  });
  headerRow->addWidget(exportBtn);
  root->addLayout(headerRow);

  // ---- Body: 2×2 graph grid (left) + temp/battery gauges (right) ----
  auto *body = new QHBoxLayout();
  body->setSpacing(8);

  auto *leftCol = new QVBoxLayout();
  leftCol->setSpacing(6);

  auto *grid = new QGridLayout();
  grid->setSpacing(8);
  m_accG = makeAxisGraph(this, tr("Accel"), QStringLiteral("m/s²"), 8);
  m_gyrG = makeAxisGraph(this, tr("Gyro"), QStringLiteral("°/s"), 2);
  m_magG = makeAxisGraph(this, tr("Mag"), QStringLiteral("µT"), 24);

  // Baro Altitude: single trace, no telemetry source yet → paints NA.
  m_baroG = new RealTimeGraph(this, 1);
  m_baroG->setColor(0, QColor("#61AFEF"));
  m_baroG->setDynamicYAxis(true);
  m_baroG->setStateBandEnabled(true);
  m_baroG->setTitle(tr("Baro Altitude"), QStringLiteral("m"));
  m_baroG->setSeriesLabels({"ALT"});
  m_baroG->setMinimumHeight(120);

  grid->addWidget(m_accG, 0, 0);
  grid->addWidget(m_gyrG, 0, 1);
  grid->addWidget(m_magG, 1, 0);
  grid->addWidget(m_baroG, 1, 1);
  leftCol->addLayout(grid, 1);

  // Vehicle-state legend (mockup .status-key).
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
    swatch->setStyleSheet(
        QString("background:%1; border-radius:2px;").arg(s.color));
    auto *txt = new QLabel(tr(s.label), key);
    txt->setStyleSheet("color: #ABB2BF; font-size: 10px;");
    keyLayout->addWidget(swatch);
    keyLayout->addWidget(txt);
  }
  keyLayout->addStretch();
  leftCol->addWidget(key);

  body->addLayout(leftCol, 1);

  // Right-hand vertical gauges (mockup .temp-gauge column).
  auto *gauges = new QHBoxLayout();
  gauges->setSpacing(8);
  m_tempGauge = new VGauge(tr("DEVICE\nTEMP"), 0, 80, "°C", VGauge::Temp, this);
  m_battGauge = new VGauge(tr("BATTERY"), 0, 100, "%", VGauge::Battery, this);
  // No battery telemetry source yet → the gauge reads "-" (set by its ctor).
  gauges->addWidget(m_tempGauge);
  gauges->addWidget(m_battGauge);
  auto *gaugeWrap = new QWidget(this);
  gaugeWrap->setLayout(gauges);
  gaugeWrap->setFixedWidth(116);
  body->addWidget(gaugeWrap);

  root->addLayout(body, 1);
}

void ImuPanel::updateImu(const ImuData &data, bool available) {
  if (!available) {
    // No live telemetry: temp gauge reads "-"; the graphs go NA on their own
    // once their traces age out of the rolling window (no new samples fed).
    m_tempGauge->setUnavailable();
    return;
  }

  RealTimeGraph *graphs[3] = {m_accG, m_gyrG, m_magG};
  RollingStats *stats[3] = {m_accStats, m_gyrStats, m_magStats};
  const float *src[3] = {data.acc, data.gyr, data.mag};
  for (int g = 0; g < 3; ++g) {
    for (int i = 0; i < 3; ++i) {
      stats[g][i].push(src[g][i]);
      graphs[g]->appendData(src[g][i], i);
      graphs[g]->appendSigma(stats[g][i].stdDev(), i);
    }
    graphs[g]->pushState(m_state);
  }
  m_baroG->pushState(m_state);  // keep the band scrolling even with no baro data
  m_tempGauge->setValue(data.tempC);
}

void ImuPanel::setSensor(const QString &name) {
  if (m_header) m_header->setText(QString("IMU — %1").arg(name));
}

void ImuPanel::setBattery(double pct) { m_battGauge->setValue(pct); }

void ImuPanel::setGraphWindow(int seconds) {
  for (RealTimeGraph *g : {m_accG, m_gyrG, m_magG, m_baroG})
    g->setWindowSeconds(seconds);
}

void ImuPanel::setGraphDropout(double rate) {
  for (RealTimeGraph *g : {m_accG, m_gyrG, m_magG, m_baroG})
    g->setDropoutRate(rate);
}
