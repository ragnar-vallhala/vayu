#include "MotorStatusWidget.h"

#include "core/CsvExport.h"
#include "core/Notify.h"
#include "core/ui/Buttons.h"

#include <QDateTime>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QtMath>

namespace {
// Quad-X motor geometry (mockup MOTORS): unit corner positions, spin
// direction and per-motor colour. Order M1..M4.
struct MotorDef {
  int cornerX;   // -1 left, +1 right
  int cornerY;   // -1 up (front), +1 down (rear)
  bool cw;       // true = clockwise
  const char *color;
};
const MotorDef kMotors[4] = {
    {+1, -1, true, "#E06C75"},   // M1 front-right CW
    {+1, +1, false, "#98C379"},  // M2 rear-right CCW
    {-1, +1, true, "#61AFEF"},   // M3 rear-left CW
    {-1, -1, false, "#D19A66"},  // M4 front-left CCW
};
}  // namespace

// ----------------------------------------------------------------------------
// DroneViewWidget: Quad-X schematic with speed-arc rings, spinning props,
// floating M#·DIR labels and a large centre % per motor (mockup parity).
// ----------------------------------------------------------------------------
class DroneViewWidget : public QWidget {
public:
  explicit DroneViewWidget(QWidget *parent = nullptr) : QWidget(parent) {
    m_speeds = {0.0f, 0.0f, 0.0f, 0.0f};
    setMinimumSize(300, 300);
    // Drive the prop animation at ~30 fps; blade angle advances per motor in
    // proportion to throttle (and sign per spin direction).
    m_anim = new QTimer(this);
    connect(m_anim, &QTimer::timeout, this, [this] {
      for (int i = 0; i < 4; ++i) {
        const float spin = (kMotors[i].cw ? 1.0f : -1.0f) * m_speeds[i];
        m_blade[i] += spin * 28.0f;  // deg/frame at full throttle
      }
      update();
    });
    m_anim->start(33);
  }

  void setSpeeds(const QVector<float> &speeds) {
    m_speeds = speeds;
    m_hasData = true;  // real motor telemetry has arrived
    update();
  }

protected:
  void paintEvent(QPaintEvent *event) override {
    Q_UNUSED(event);
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const int w = width();
    const int h = height();
    const int cx = w / 2;
    const int cy = h / 2;
    // Leave margin for the floating M#·DIR labels above/below the corner rings.
    const int size = qMin(w, h) * 0.62;
    const int armLen = size / 2;

    p.save();
    p.translate(cx, cy);

    // Arms (X configuration).
    p.setPen(QPen(QColor(62, 68, 82), 12, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(-armLen, -armLen, armLen, armLen);
    p.drawLine(-armLen, armLen, armLen, -armLen);

    // Centre body + front indicator.
    p.setPen(QPen(QColor(97, 175, 239), 2));
    p.setBrush(QColor(33, 37, 43));
    p.drawRoundedRect(-40, -60, 80, 120, 15, 15);
    p.setBrush(QColor(224, 108, 117));
    p.setPen(Qt::NoPen);
    p.drawEllipse(-10, -50, 20, 20);

    for (int i = 0; i < 4; ++i)
      drawMotor(p, kMotors[i].cornerX * armLen, kMotors[i].cornerY * armLen, i);

    p.restore();
  }

private:
  void drawMotor(QPainter &p, int x, int y, int idx) {
    const int r = 38;
    const QColor col(kMotors[idx].color);
    const float frac = qBound(0.0f, m_speeds[idx], 1.0f);

    p.save();
    p.translate(x, y);

    // Outer ring track.
    p.setPen(QPen(QColor(58, 65, 80), 7));
    p.setBrush(QColor(26, 29, 39));
    p.drawEllipse(QPointF(0, 0), r, r);

    // Speed arc — starts at top (12 o'clock), grows clockwise with throttle.
    QRectF rect(-r, -r, 2 * r, 2 * r);
    p.setPen(QPen(col, 7, Qt::SolidLine, Qt::RoundCap));
    p.setBrush(Qt::NoBrush);
    p.drawArc(rect, 90 * 16, -static_cast<int>(frac * 360 * 16));

    // Spinning prop blades over the hub (motion-blurred via low opacity).
    p.save();
    p.rotate(m_blade[idx]);
    QColor blade = col;
    blade.setAlpha(frac > 0.01f ? 150 : 60);
    p.setPen(Qt::NoPen);
    p.setBrush(blade);
    for (int b = 0; b < 2; ++b) {
      p.drawEllipse(QPointF(0, 0), 21, 5);
      p.rotate(90);
    }
    p.restore();

    // Hub disc.
    p.setPen(QPen(col, 1.5));
    p.setBrush(QColor(44, 49, 58));
    p.drawEllipse(QPointF(0, 0), 12, 12);

    // Large centre % overlay ("-" until real motor telemetry arrives).
    p.setPen(col);
    QFont pf = p.font();
    pf.setBold(true);
    pf.setPixelSize(15);
    p.setFont(pf);
    p.drawText(QRectF(-r, -10, 2 * r, 20), Qt::AlignCenter,
               m_hasData ? QString::number(static_cast<int>(frac * 100))
                         : QStringLiteral("-"));

    // Floating M#·DIR label: above the ring for front motors, below for rear.
    const int lblY = (kMotors[idx].cornerY < 0) ? -(r + 16) : (r + 6);
    QFont lf = p.font();
    lf.setBold(true);
    lf.setPixelSize(11);
    p.setFont(lf);
    p.drawText(QRectF(-r, lblY, 2 * r, 14), Qt::AlignCenter,
               QString("M%1 · %2").arg(idx + 1).arg(kMotors[idx].cw ? "CW"
                                                                    : "CCW"));

    p.restore();
  }

  QVector<float> m_speeds;
  bool m_hasData = false;           // false → centre reads "-" (no telemetry)
  float m_blade[4] = {0, 0, 0, 0};  // accumulated blade angle (deg) per motor
  QTimer *m_anim = nullptr;
};

// ----------------------------------------------------------------------------
// MotorStatusWidget
// ----------------------------------------------------------------------------
namespace {
// One "k : v" row inside the Power group, styled like the mockup .field.
QLabel *addPowerField(QGridLayout *g, int row, const QString &key) {
  auto *k = new QLabel(key);
  k->setStyleSheet("color: #8A92A6; font-size: 11px; border: none;");
  auto *v = new QLabel("-");
  v->setStyleSheet("color: #E8F0FE; font-family: 'Monospace'; "
                   "font-weight: bold; font-size: 12px; border: none;");
  v->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  g->addWidget(k, row, 0);
  g->addWidget(v, row, 1);
  return v;
}
}  // namespace

MotorStatusWidget::MotorStatusWidget(QWidget *parent) : QWidget(parent) {
  m_speeds = {0.0f, 0.0f, 0.0f, 0.0f};

  auto *mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(15, 15, 15, 15);
  mainLayout->setSpacing(10);

  // Header
  auto *header = new QHBoxLayout();
  auto *backBtn = new ui::BackButton(this);
  backBtn->setText(tr("← BACK TO HOME"));
  backBtn->setFixedSize(140, 32);
  backBtn->setToolTip(tr("Return to home"));
  connect(backBtn, &QPushButton::clicked, this,
          &MotorStatusWidget::backToHomeRequested);

  auto *title = new QLabel("MOTORS & POWER", this);
  title->setStyleSheet("color: #ABB2BF; font-weight: bold; font-size: 14px;");

  auto *exportBtn = new ui::GhostButton(tr("Export CSV"), this);
  exportBtn->setToolTip(tr("Save the currently-buffered motor speed traces"));
  connect(exportBtn, &QPushButton::clicked, this, [this] {
    const QString stamp =
        QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss");
    const QString path = CsvExport::promptAndWriteCombined(
        this, QString("motors-%1.csv").arg(stamp),
        {{m_graph, {"m1_fr", "m2_rr", "m3_rl", "m4_fl"}}});
    if (!path.isEmpty()) Notify::ok(this, tr("Wrote %1").arg(path));
  });

  header->addWidget(backBtn);
  header->addStretch();
  header->addWidget(title);
  header->addStretch();
  header->addWidget(exportBtn);
  mainLayout->addLayout(header);

  // Content: left column (schematic over motor-speed graph) + right column
  // (per-motor cards over the Power group), matching the mockup layout.
  auto *content = new QHBoxLayout();
  content->setSpacing(20);

  // ---- Left column ----
  auto *leftCol = new QVBoxLayout();
  leftCol->setSpacing(10);

  auto *schemTitle = new QLabel("Motor Layout — Quad X", this);
  schemTitle->setStyleSheet(
      "color: #8A92A6; font-weight: bold; font-size: 11px;");
  leftCol->addWidget(schemTitle);

  m_droneView = new DroneViewWidget(this);
  leftCol->addWidget(m_droneView, 3);

  auto *graphTitle = new QLabel("Motor Speeds — 10s", this);
  graphTitle->setStyleSheet(
      "color: #8A92A6; font-weight: bold; font-size: 11px;");
  leftCol->addWidget(graphTitle);

  const char *colors[] = {"#E06C75", "#98C379", "#61AFEF", "#D19A66"};
  m_graph = new RealTimeGraph(this, 4);
  for (int i = 0; i < 4; ++i) m_graph->setColor(i, QColor(colors[i]));
  m_graph->setWindowSeconds(10);
  leftCol->addWidget(m_graph, 2);

  content->addLayout(leftCol, 1);

  // ---- Right column (fixed width, like the mockup's 340px panel) ----
  auto *rightWrap = new QWidget(this);
  rightWrap->setFixedWidth(320);
  auto *rightPanel = new QVBoxLayout(rightWrap);
  rightPanel->setContentsMargins(0, 0, 0, 0);
  rightPanel->setSpacing(10);

  // Per-Motor cards.
  auto *perMotorTitle = new QLabel("Per-Motor", rightWrap);
  perMotorTitle->setStyleSheet(
      "color: #8A92A6; font-weight: bold; font-size: 11px;");
  rightPanel->addWidget(perMotorTitle);

  for (int i = 0; i < 4; ++i) {
    auto *card = new QWidget(rightWrap);
    card->setStyleSheet(
        "background: #282C34; border-radius: 6px; border: 1px solid #3E4452;");
    auto *cardLayout = new QHBoxLayout(card);
    cardLayout->setContentsMargins(10, 5, 10, 5);

    auto *nameLabel = new QLabel(QString("M%1").arg(i + 1), card);
    nameLabel->setStyleSheet(
        QString("color: %1; font-weight: bold; font-size: 14px; border: none;")
            .arg(colors[i]));

    auto *dirLabel = new QLabel(kMotors[i].cw ? "CW" : "CCW", card);
    dirLabel->setStyleSheet("color: #5C6370; font-size: 10px; border: none;");

    m_valLabels[i] = new QLabel("-", card);
    m_valLabels[i]->setStyleSheet(
        "color: #E8F0FE; font-family: 'Monospace'; font-weight: bold; "
        "font-size: 16px; border: none;");
    m_valLabels[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    m_valLabels[i]->setFixedWidth(54);

    m_stdLabels[i] = new QLabel("± σ -", card);
    m_stdLabels[i]->setStyleSheet("color: #5C6370; font-family: 'Monospace'; "
                                  "font-size: 11px; border: none;");
    m_stdLabels[i]->setFixedWidth(90);

    cardLayout->addWidget(nameLabel);
    cardLayout->addWidget(dirLabel);
    cardLayout->addStretch();
    cardLayout->addWidget(m_valLabels[i]);
    cardLayout->addSpacing(8);
    cardLayout->addWidget(m_stdLabels[i]);

    rightPanel->addWidget(card);
  }

  // Power group.
  auto *powerTitle = new QLabel("Power", rightWrap);
  powerTitle->setStyleSheet(
      "color: #8A92A6; font-weight: bold; font-size: 11px;");
  rightPanel->addSpacing(4);
  rightPanel->addWidget(powerTitle);

  auto *powerBox = new QWidget(rightWrap);
  powerBox->setStyleSheet(
      "background: #282C34; border-radius: 6px; border: 1px solid #3E4452;");
  auto *powerGrid = new QGridLayout(powerBox);
  powerGrid->setContentsMargins(10, 8, 10, 8);
  powerGrid->setHorizontalSpacing(12);
  powerGrid->setVerticalSpacing(6);
  m_avgThrottle = addPowerField(powerGrid, 0, tr("Avg Throttle"));
  m_maxSpread = addPowerField(powerGrid, 1, tr("Max Spread"));
  m_battery = addPowerField(powerGrid, 2, tr("Battery"));
  m_current = addPowerField(powerGrid, 3, tr("Current"));
  m_escTemp = addPowerField(powerGrid, 4, tr("ESC Temp"));
  m_mahUsed = addPowerField(powerGrid, 5, tr("Used"));
  // Battery / current / ESC-temp / mAh have no telemetry source yet → they stay
  // "-" (set by addPowerField) so absent data isn't mistaken for a real value.
  rightPanel->addWidget(powerBox);

  rightPanel->addStretch();
  content->addWidget(rightWrap);

  mainLayout->addLayout(content, 1);
}

void MotorStatusWidget::setMotorSpeeds(const QVector<float> &speeds) {
  if (speeds.size() < 4) return;
  m_speeds = speeds;
  m_droneView->setSpeeds(speeds);

  float sum = 0.0f, mn = 1e9f, mx = -1e9f;
  for (int i = 0; i < 4; ++i) {
    m_stats[i].push(speeds[i]);
    m_valLabels[i]->setText(
        QString("%1%").arg(static_cast<int>(speeds[i] * 100)));
    m_stdLabels[i]->setText(QString("± σ %1").arg(
        static_cast<double>(m_stats[i].stdDev()), 0, 'f', 3));
    m_graph->appendData(speeds[i], i);
    sum += speeds[i];
    mn = qMin(mn, speeds[i]);
    mx = qMax(mx, speeds[i]);
  }

  // Derived power readouts from the live motor mix.
  m_avgThrottle->setText(QString("%1%").arg(static_cast<int>(sum / 4 * 100)));
  m_maxSpread->setText(QString("%1%").arg(static_cast<int>((mx - mn) * 100)));
}
