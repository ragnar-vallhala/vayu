#include "ControlLoopPlot.h"

#include "core/CsvExport.h"
#include "core/Notify.h"
#include "core/Theme.h"
#include "core/ui/Buttons.h"

#include <QDateTime>
#include <QDoubleValidator>
#include <QGridLayout>
#include <QScrollArea>
#include <QSplitter>
#include <cmath>

ControlLoopPlot::ControlLoopPlot(QWidget *parent) : QWidget(parent) {
  auto *mainLayout = new QVBoxLayout(this);
  mainLayout->setSpacing(10);
  mainLayout->setContentsMargins(15, 15, 15, 15);

  auto *header = new QHBoxLayout();
  auto *backBtn = new ui::BackButton(this);
  backBtn->setText(tr("← BACK"));
  backBtn->setFixedSize(80, 30);
  backBtn->setToolTip(tr("Return to home"));
  connect(backBtn, &QPushButton::clicked, this,
          &ControlLoopPlot::backToHomeRequested);

  auto *title = new QLabel("CONTROL LOOP DASHBOARD", this);
  title->setStyleSheet(
      QString("font-size: 18px; font-weight: bold; color: %1;")
          .arg(Theme::hex(Theme::kAccent)));

  header->addWidget(backBtn);
  header->addSpacing(20);
  header->addWidget(title);
  header->addStretch();

  auto *scaleLabel = new QLabel("Gyro Scale:", this);
  scaleLabel->setStyleSheet(
      QString("color: %1; font-size: 12px;").arg(Theme::hex(Theme::kTextMuted)));
  auto *scaleEdit = new QLineEdit("1.0", this);
  scaleEdit->setFixedWidth(60);
  scaleEdit->setValidator(new QDoubleValidator(0.0, 1000.0, 4, this));
  scaleEdit->setToolTip(tr("Multiplier applied to gyro current traces"));
  // QLineEdit chrome comes from the global QSS.
  connect(scaleEdit, &QLineEdit::textChanged, this,
          [this](const QString &text) { m_gyroScale = text.toFloat(); });

  header->addWidget(scaleLabel);
  header->addSpacing(5);
  header->addWidget(scaleEdit);
  header->addSpacing(20);

  // CSV export. Combines all four sub-graphs into one file along a
  // unified timestamp axis so they can be analysed together off-line.
  auto *exportBtn = new ui::GhostButton(tr("Export CSV"), this);
  exportBtn->setToolTip(
      tr("Save the currently-buffered angle / rate / output / dt traces"));
  connect(exportBtn, &QPushButton::clicked, this, [this] {
    const QString stamp =
        QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss");
    const QString path = CsvExport::promptAndWriteCombined(
        this, QString("controlloop-%1.csv").arg(stamp),
        {
          {m_angleGraph,  {"roll_angle_sp", "pitch_angle_sp", "yaw_angle_sp",
                           "roll_angle_curr", "pitch_angle_curr", "yaw_angle_curr"}},
          {m_rateGraph,   {"roll_rate_sp", "pitch_rate_sp", "yaw_rate_sp",
                           "roll_rate_curr", "pitch_rate_curr", "yaw_rate_curr"}},
          {m_outputGraph, {"roll_out", "pitch_out", "yaw_out", "throttle_out"}},
          {m_dtGraph,     {"outer_dt", "inner_dt"}},
          {m_estLatGraph, {"est_peak_us", "est_mean_us"}},
        });
    if (!path.isEmpty()) Notify::ok(this, tr("Wrote %1").arg(path));
  });
  header->addWidget(exportBtn);

  mainLayout->addLayout(header);

  auto *scrollArea = new QScrollArea(this);
  scrollArea->setWidgetResizable(true);
  scrollArea->setStyleSheet("QScrollArea { border: none; }");

  auto *scrollContent = new QWidget();
  auto *scrollLayout = new QVBoxLayout(scrollContent);
  scrollLayout->setContentsMargins(0, 0, 0, 0);

  auto *vSplitter = new QSplitter(Qt::Vertical, scrollContent);
  auto *topHSplitter = new QSplitter(Qt::Horizontal, vSplitter);
  auto *botHSplitter = new QSplitter(Qt::Horizontal, vSplitter);
  // Splitter handle styling comes from the global QSS.

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

  auto createValueWithStdLabel = [this](QHBoxLayout *parentLayout,
                                        const QString &name,
                                        const QString &color,
                                        QLabel **outValueLabel,
                                        QLabel **outStdLabel) {
    auto *box = new QVBoxLayout();
    auto *l = new QLabel(name, this);
    l->setStyleSheet(
        QString("color: %1; font-size: 11px; font-weight: bold;").arg(color));

    *outValueLabel = new QLabel("0.00000", this);
    (*outValueLabel)
        ->setStyleSheet(
            QString("color: %1; font-size: 18px; font-family: Monospace; "
                    "font-weight: bold;")
                .arg(color));

    *outStdLabel = new QLabel("± σ 0.000000", this);
    (*outStdLabel)
        ->setStyleSheet(
            "color: #ABB2BF; font-size: 10px; font-family: Monospace;");

    box->addWidget(l);
    box->addWidget(*outValueLabel);
    box->addWidget(*outStdLabel);
    box->setAlignment(Qt::AlignLeft);
    parentLayout->addLayout(box);
  };

  QHBoxLayout *angleStats;
  auto *angleSec = createSection("OUTER LOOP: ANGLE SP & CURRENT", &angleStats,
                                 &m_angleGraph, 6);
  createValueLabel(angleStats, "ROLL SP", "#D19A66", &m_rollAngleSpVal);
  createValueLabel(angleStats, "PITCH SP", "#C678DD", &m_pitchAngleSpVal);
  createValueLabel(angleStats, "YAW SP", "#56B6C2", &m_yawAngleSpVal);
  createValueLabel(angleStats, "ROLL CURR", "#E06C75", &m_rollAngleCurrVal);
  createValueLabel(angleStats, "PITCH CURR", "#98C379", &m_pitchAngleCurrVal);
  createValueLabel(angleStats, "YAW CURR", "#61AFEF", &m_yawAngleCurrVal);
  angleStats->addStretch();

  // SP colors (Dashed)
  m_angleGraph->setColor(0, QColor("#D19A66"));
  m_angleGraph->setColor(1, QColor("#C678DD"));
  m_angleGraph->setColor(2, QColor("#56B6C2"));
  m_angleGraph->setPenStyle(0, Qt::DashLine);
  m_angleGraph->setPenStyle(1, Qt::DashLine);
  m_angleGraph->setPenStyle(2, Qt::DashLine);
  // Current colors (Solid)
  m_angleGraph->setColor(3, QColor("#E06C75"));
  m_angleGraph->setColor(4, QColor("#98C379"));
  m_angleGraph->setColor(5, QColor("#61AFEF"));

  topHSplitter->addWidget(angleSec);

  QHBoxLayout *rateStats;
  auto *rateSec =
      createSection("INNER LOOP: RATE SP & GYRO", &rateStats, &m_rateGraph, 6);
  createValueLabel(rateStats, "ROLL RATE SP", "#D19A66", &m_rollRateSpVal);
  createValueLabel(rateStats, "PITCH RATE SP", "#C678DD", &m_pitchRateSpVal);
  createValueLabel(rateStats, "YAW RATE SP", "#56B6C2", &m_yawRateSpVal);
  createValueLabel(rateStats, "ROLL CURR", "#BE5046", &m_rollRateCurrVal);
  createValueLabel(rateStats, "PITCH CURR", "#7FB069", &m_pitchRateCurrVal);
  createValueLabel(rateStats, "YAW CURR", "#4078BF", &m_yawRateCurrVal);
  rateStats->addStretch();

  // SP colors (Dashed)
  m_rateGraph->setColor(0, QColor("#D19A66"));
  m_rateGraph->setColor(1, QColor("#C678DD"));
  m_rateGraph->setColor(2, QColor("#56B6C2"));
  m_rateGraph->setPenStyle(0, Qt::DashLine);
  m_rateGraph->setPenStyle(1, Qt::DashLine);
  m_rateGraph->setPenStyle(2, Qt::DashLine);
  // Current (Gyro) colors (Solid)
  m_rateGraph->setColor(3, QColor("#BE5046"));
  m_rateGraph->setColor(4, QColor("#7FB069"));
  m_rateGraph->setColor(5, QColor("#4078BF"));
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
  auto *dtSec = createSection("LOOP TIME (dt)", &dtStats, &m_dtGraph, 2);
  createValueWithStdLabel(dtStats, "Outer dt", "#98C379", &m_dtOuterVal,
                          &m_dtOuterStdVal);
  createValueWithStdLabel(dtStats, "Inner dt", "#61AFEF", &m_dtInnerVal,
                          &m_dtInnerStdVal);
  dtStats->addStretch();
  m_dtGraph->setColor(0, QColor("#98C379"));
  m_dtGraph->setColor(1, QColor("#61AFEF"));
  m_dtGraph->setDynamicYAxis(true);
  botHSplitter->addWidget(dtSec);

  // Estimator cost: per-update EKF/Mahony peak & mean (µs), from the firmware
  // ATTITUDE_CYCLE_PROBE (SYSTEM_ORIGIN_EST_PERF, ~1 Hz).
  QHBoxLayout *estStats;
  auto *estSec =
      createSection("EST LATENCY (us)", &estStats, &m_estLatGraph, 2);
  createValueLabel(estStats, "PEAK us", "#E06C75", &m_estPeakVal);
  createValueLabel(estStats, "MEAN us", "#98C379", &m_estMeanVal);
  createValueLabel(estStats, "CADENCE", "#56B6C2", &m_estCadenceVal);
  estStats->addStretch();
  m_estLatGraph->setColor(0, QColor("#E06C75")); // peak
  m_estLatGraph->setColor(1, QColor("#98C379")); // mean
  m_estLatGraph->setDynamicYAxis(true);
  botHSplitter->addWidget(estSec);

  vSplitter->addWidget(botHSplitter);

  // Background + text colour come from the global QSS (QMainWindow + QWidget).
  scrollArea->setWidget(scrollContent);
  mainLayout->addWidget(scrollArea, 1);
}

void ControlLoopPlot::setProtocol(DroneProtocol *protocol) {
  if (m_protocol) {
    disconnect(m_protocol, &DroneProtocol::controlLoopDataReceived, this,
               &ControlLoopPlot::onControlLoopDataReceived);
    disconnect(m_protocol, &DroneProtocol::imuReceived, this,
               &ControlLoopPlot::onImuReceived);
    disconnect(m_protocol, &DroneProtocol::estPerfReceived, this,
               &ControlLoopPlot::onEstPerfReceived);
  }
  m_protocol = protocol;
  if (m_protocol) {
    connect(m_protocol, &DroneProtocol::controlLoopDataReceived, this,
            &ControlLoopPlot::onControlLoopDataReceived);
    connect(m_protocol, &DroneProtocol::imuReceived, this,
            &ControlLoopPlot::onImuReceived);
    connect(m_protocol, &DroneProtocol::estPerfReceived, this,
            &ControlLoopPlot::onEstPerfReceived);
  }
}

void ControlLoopPlot::onControlLoopDataReceived(const ControlLoopData &data) {
  // Update DT
  m_dtOuterVal->setText(
      QString::number(static_cast<double>(data.outer_dt), 'f', 5));
  m_dtInnerVal->setText(
      QString::number(static_cast<double>(data.inner_dt), 'f', 5));
  m_dtGraph->appendData(data.outer_dt, 0);
  m_dtGraph->appendData(data.inner_dt, 1);

  // Helper for rolling STD
  auto updateStd = [this](QQueue<float> &history, float newVal, QLabel *label) {
    history.enqueue(newVal);
    if (history.size() > m_stdWindowSize)
      history.dequeue();
    if (history.size() < 2) {
      label->setText("0.00000");
      return;
    }
    double sum = 0;
    for (float v : history)
      sum += v;
    double mean = sum / history.size();
    double sqSum = 0;
    for (float v : history)
      sqSum += (v - mean) * (v - mean);
    double stdDev = std::sqrt(sqSum / history.size());
    label->setText(QString("± σ %1").arg(stdDev, 0, 'f', 6));
  };

  updateStd(m_outerDtHistory, data.outer_dt, m_dtOuterStdVal);
  updateStd(m_innerDtHistory, data.inner_dt, m_dtInnerStdVal);

  // Angles
  m_angleGraph->appendData(data.roll_angle_setpoint, 0);
  m_angleGraph->appendData(data.pitch_angle_setpoint, 1);
  m_angleGraph->appendData(data.yaw_angle_setpoint, 2);
  m_angleGraph->appendData(data.roll_angle_current, 3);
  m_angleGraph->appendData(data.pitch_angle_current, 4);
  m_angleGraph->appendData(data.yaw_angle_current, 5);

  m_rollAngleSpVal->setText(
      QString::number(static_cast<double>(data.roll_angle_setpoint), 'f', 3));
  m_pitchAngleSpVal->setText(
      QString::number(static_cast<double>(data.pitch_angle_setpoint), 'f', 3));
  m_yawAngleSpVal->setText(
      QString::number(static_cast<double>(data.yaw_angle_setpoint), 'f', 3));
  m_rollAngleCurrVal->setText(
      QString::number(static_cast<double>(data.roll_angle_current), 'f', 3));
  m_pitchAngleCurrVal->setText(
      QString::number(static_cast<double>(data.pitch_angle_current), 'f', 3));
  m_yawAngleCurrVal->setText(
      QString::number(static_cast<double>(data.yaw_angle_current), 'f', 3));

  // Rates
  m_rateGraph->appendData(data.roll_rate_setpoint, 0);
  m_rateGraph->appendData(data.pitch_rate_setpoint, 1);
  m_rateGraph->appendData(data.yaw_rate_setpoint, 2);

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

void ControlLoopPlot::onEstPerfReceived(const EstPerfData &data) {
  m_estLatGraph->appendData(data.peak_us, 0);
  m_estLatGraph->appendData(data.mean_us, 1);

  m_estPeakVal->setText(
      QString::number(static_cast<double>(data.peak_us), 'f', 1));
  m_estMeanVal->setText(
      QString::number(static_cast<double>(data.mean_us), 'f', 1));
  m_estCadenceVal->setText(QString("decim %1 @ %2 Hz")
                               .arg(static_cast<int>(data.decim))
                               .arg(static_cast<int>(data.rate_hz)));
}

void ControlLoopPlot::onImuReceived(const ImuData &data) {
  float rollRateCurr = data.gyr[0] * m_gyroScale;
  float pitchRateCurr = data.gyr[1] * m_gyroScale;
  float yawRateCurr = data.gyr[2] * m_gyroScale;

  m_rateGraph->appendData(rollRateCurr, 3);
  m_rateGraph->appendData(pitchRateCurr, 4);
  m_rateGraph->appendData(yawRateCurr, 5);

  m_rollRateCurrVal->setText(
      QString::number(static_cast<double>(rollRateCurr), 'f', 3));
  m_pitchRateCurrVal->setText(
      QString::number(static_cast<double>(pitchRateCurr), 'f', 3));
  m_yawRateCurrVal->setText(
      QString::number(static_cast<double>(yawRateCurr), 'f', 3));
}
