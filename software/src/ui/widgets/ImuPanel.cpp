#include "ImuPanel.h"

#include "core/ui/Icons.h"

#include <QColor>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
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
  g->setStateBandEnabled(false);    // off by default (Settings can enable it)
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

  // ---- Header: icon + title (Export CSV removed — whole-session capture is
  // handled system-wide via Settings → record-on-connect) ----
  auto *headerRow = new QHBoxLayout();
  auto *icon = new QLabel(this);
  icon->setPixmap(ui::svgPixmap(ui::Icon::Imu, QColor(0x61, 0xAF, 0xEF), 16));
  // Centre the pixmap inside the label. QLabel defaults to AlignLeft|AlignVCenter,
  // so on HiDPI/fractional scaling — where the label box rounds a pixel wider than
  // the 16px pixmap — the glyph would otherwise hug the left edge.
  icon->setAlignment(Qt::AlignCenter);
  headerRow->addWidget(icon);
  // Sensor-agnostic by default; setSensor() appends the part name if telemetry
  // ever reports it (don't hardcode a specific IMU).
  m_header = new QLabel(tr("IMU Telemetry"), this);
  m_header->setStyleSheet(
      "color: #61AFEF; font-weight: bold; font-size: 13px;");
  headerRow->addWidget(m_header);
  headerRow->addStretch();
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

  // Baro Altitude: two traces — MSL (height above sea level, the absolute
  // figure from the barometer) and AGL (height above the ground reference
  // captured on the ground; ~0 at rest, rises with climb). Fed by BARO telem.
  m_baroG = new RealTimeGraph(this, 2);
  m_baroG->setColor(0, QColor("#61AFEF"));  // MSL — blue (right axis)
  m_baroG->setColor(1, QColor("#98C379"));  // AGL — green (left axis)
  m_baroG->setDynamicYAxis(true);
  m_baroG->setSeriesAxis(0, true);  // MSL on the right axis (sea-level, ~485 m)
  m_baroG->setSeriesAxis(1, false); // AGL on the left axis (above-ground, ~0 m)
  m_baroG->setStateBandEnabled(false);  // off by default (Settings can enable it)
  m_baroG->setTitle(tr("Baro Altitude"), QStringLiteral("m"));
  m_baroG->setSeriesLabels({"MSL", "AGL"});
  m_baroG->setMinimumHeight(120);

  // Vertical Estimate: the VERT fused-vs-raw validation chart. Fused altitude
  // (orange) overlays the raw baro altitude (blue) on the left axis so estimator
  // lag/divergence is visible at a glance; fused climb rate (green) rides the
  // right axis (m/s). Fed by VERTICAL_STATE telem (setVerticalState).
  m_vertG = new RealTimeGraph(this, 3);
  m_vertG->setColor(0, QColor("#E0822E"));  // fused altitude — orange (left axis)
  m_vertG->setColor(1, QColor("#61AFEF"));  // raw baro altitude — blue (left axis)
  m_vertG->setColor(2, QColor("#98C379"));  // climb rate — green (right axis, m/s)
  m_vertG->setDynamicYAxis(true);
  m_vertG->setSeriesAxis(0, false);  // fused altitude on the left axis (m)
  m_vertG->setSeriesAxis(1, false);  // raw baro altitude on the left axis (m)
  m_vertG->setSeriesAxis(2, true);   // climb rate on the right axis (m/s)
  m_vertG->setStateBandEnabled(false);
  m_vertG->setTitle(tr("Vertical Estimate"), QStringLiteral("m"));
  m_vertG->setSeriesLabels({"Fused", "Baro", "Climb"});
  m_vertG->setMinimumHeight(120);

  grid->addWidget(m_accG, 0, 0);
  grid->addWidget(m_gyrG, 0, 1);
  grid->addWidget(m_magG, 1, 0);
  grid->addWidget(m_baroG, 1, 1);
  grid->addWidget(m_vertG, 2, 0, 1, 2);  // full-width fused-vs-raw chart
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
  m_baroG->pushState(m_state);  // keep the band scrolling; data fed separately
  m_vertG->pushState(m_state);  // ditto for the fused vertical chart
  m_tempGauge->setValue(data.tempC);
}

void ImuPanel::setBaroAltitude(float mslM, float aglM, bool available) {
  // BARO arrives independently of the IMU stream (separate NavLink message), so
  // it has its own slot. When stale, stop feeding — the traces age out to NA on
  // their own, matching the IMU graphs' behaviour. MSL = sea-level height; AGL =
  // height above the ground reference (computed by the caller).
  if (!available)
    return;
  m_baroG->appendData(mslM, 0);
  m_baroG->appendData(aglM, 1);
}

void ImuPanel::setVerticalState(float fusedAltM, float baroAltM,
                                float climbRateMs, bool available) {
  // VERTICAL_STATE arrives independently of the IMU stream (its own NavLink
  // message + freshness/seeded gate), so it has its own slot. When stale/unseeded
  // we stop feeding and the traces age out to NA, matching the other graphs.
  if (!available)
    return;
  m_vertG->appendData(fusedAltM, 0);
  m_vertG->appendData(baroAltM, 1);
  m_vertG->appendData(climbRateMs, 2);
}

void ImuPanel::setSensor(const QString &name) {
  if (m_header) m_header->setText(QString("IMU — %1").arg(name));
}

void ImuPanel::setBattery(double pct) { m_battGauge->setValue(pct); }

void ImuPanel::setGraphWindow(int seconds) {
  for (RealTimeGraph *g : {m_accG, m_gyrG, m_magG, m_baroG, m_vertG})
    g->setWindowSeconds(seconds);
}

void ImuPanel::setGraphDropout(double rate) {
  for (RealTimeGraph *g : {m_accG, m_gyrG, m_magG, m_baroG, m_vertG})
    g->setDropoutRate(rate);
}

void ImuPanel::setSigmaTraces(bool on) {
  // Only the 3-axis graphs feed σ; baro has no σ overlay.
  for (RealTimeGraph *g : {m_accG, m_gyrG, m_magG}) g->setSigmaEnabled(on);
}

void ImuPanel::setStateBand(bool on) {
  for (RealTimeGraph *g : {m_accG, m_gyrG, m_magG, m_baroG, m_vertG})
    g->setStateBandEnabled(on);
}

void ImuPanel::setTraceWidth(double w) {
  for (RealTimeGraph *g : {m_accG, m_gyrG, m_magG, m_baroG, m_vertG})
    g->setTraceWidth(w);
}

void ImuPanel::setAntialias(bool on) {
  for (RealTimeGraph *g : {m_accG, m_gyrG, m_magG, m_baroG, m_vertG})
    g->setAntialias(on);
}
