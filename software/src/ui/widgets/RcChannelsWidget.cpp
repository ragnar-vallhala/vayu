#include "RcChannelsWidget.h"

#include "core/Theme.h"
#include "core/ui/Buttons.h"

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <cmath>

namespace {
// Mockup rcDefs + per-type colours (#3a5f8f stick, #7fae5a thr, #5c6f99 aux).
struct ChDef {
  const char *name;
  const char *color;
};
const ChDef kChans[8] = {
    {"Roll (Aileron)", "#3a5f8f"}, {"Pitch (Elevator)", "#3a5f8f"},
    {"Throttle", "#7fae5a"},       {"Yaw (Rudder)", "#3a5f8f"},
    {"AUX1 — Mode", "#5c6f99"},    {"AUX2 — Arm", "#5c6f99"},
    {"AUX3", "#5c6f99"},           {"AUX4", "#5c6f99"},
};

// A "Link Health" field: key label + coloured value (mockup .field).
QWidget *field(const QString &k, const QString &v, const QString &color = {}) {
  auto *w = new QWidget;
  auto *l = new QVBoxLayout(w);
  l->setContentsMargins(0, 0, 0, 0);
  l->setSpacing(1);
  auto *kl = new QLabel(k);
  kl->setStyleSheet("font-size:10px; color:" +
                    QString(Theme::hex(Theme::kTextDim)) + ";");
  auto *vl = new QLabel(v);
  vl->setStyleSheet("font-size:12px; font-weight:600;" +
                    (color.isEmpty() ? QString() : "color:" + color + ";"));
  l->addWidget(kl);
  l->addWidget(vl);
  return w;
}
}  // namespace

RcChannelsWidget::RcChannelsWidget(QWidget *parent) : QWidget(parent) {
  setWindowTitle("RC Channels Monitor");
  setMinimumSize(400, 600);

  auto *mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(16, 12, 16, 12);
  mainLayout->setSpacing(10);

  // Back button.
  auto *topRow = new QHBoxLayout();
  auto *backBtn = new ui::BackButton(this);
  backBtn->setText(tr(" ←  Back"));
  backBtn->setFixedWidth(100);
  connect(backBtn, &QPushButton::clicked, this,
          &RcChannelsWidget::backToHomeRequested);
  topRow->addWidget(backBtn);
  topRow->addStretch();
  mainLayout->addLayout(topRow);

  // ---- rc-top: Sticks | Link Health | Switches ----
  auto *rcTop = new QHBoxLayout();
  rcTop->setSpacing(10);

  // Sticks box (dual gimbals).
  auto *sticks = new QGroupBox(tr("Sticks"), this);
  auto *sl = new QHBoxLayout(sticks);
  m_stickL = new StickGimbal(tr("Throttle / Yaw"), this);
  m_stickR = new StickGimbal(tr("Pitch / Roll"), this);
  sl->addWidget(m_stickL);
  sl->addWidget(m_stickR);
  rcTop->addWidget(sticks);

  // Link Health box — no link-quality telemetry source yet, so every field
  // reads "-" rather than a fabricated value (real RC data only drives the
  // channel bars below).
  auto *link = new QGroupBox(tr("Link Health"), this);
  auto *lv = new QVBoxLayout(link);
  auto *grid = new QGridLayout();
  grid->addWidget(field(tr("RSSI"), "-"), 0, 0);
  grid->addWidget(field(tr("Link Quality"), "-"), 0, 1);
  grid->addWidget(field(tr("Frame Rate"), "-"), 1, 0);
  grid->addWidget(field(tr("Protocol"), "-"), 1, 1);
  grid->addWidget(field(tr("Channels"), "-"), 2, 0);
  grid->addWidget(field(tr("Failsafe"), "-"), 2, 1);
  lv->addLayout(grid);
  auto *rssi = new QProgressBar(link);
  rssi->setRange(0, 100);
  rssi->setValue(0);
  rssi->setTextVisible(false);
  rssi->setFixedHeight(8);
  rssi->setStyleSheet(
      "QProgressBar{background:#13141B;border:1px solid #2A3347;border-radius:4px;}"
      "QProgressBar::chunk{border-radius:3px;background:qlineargradient(x1:0,y1:0,"
      "x2:1,y2:0,stop:0 #E06C75,stop:0.55 #D19A66,stop:1 #98C379);}");
  lv->addWidget(rssi);
  lv->addStretch();
  rcTop->addWidget(link, 1);

  // Switches box (AUX segmented switches).
  auto *sw = new QGroupBox(tr("Switches"), this);
  auto *swv = new QVBoxLayout(sw);
  m_aux[0] = new AuxSwitch("AUX1 · Mode", {"ANGLE", "HORIZON", "ACRO"}, this);
  m_aux[1] = new AuxSwitch("AUX2 · Arm", {"SAFE", "ARM"}, this);
  m_aux[2] = new AuxSwitch("AUX3 · Beeper", {"OFF", "ON"}, this);
  m_aux[3] = new AuxSwitch("AUX4 · Turtle", {"OFF", "ON"}, this);
  for (auto *a : m_aux) swv->addWidget(a);
  swv->addStretch();
  rcTop->addWidget(sw);

  mainLayout->addLayout(rcTop);

  // ---- Channels (named bars, 2-col grid) ----
  auto *chans = new QGroupBox(tr("Channels"), this);
  auto *cg = new QGridLayout(chans);
  cg->setHorizontalSpacing(28);
  cg->setVerticalSpacing(9);
  for (int i = 0; i < 8; ++i) {
    auto *name = new QLabel(kChans[i].name, this);
    name->setStyleSheet("font-size:11px;");
    auto *bar = new QProgressBar(this);
    bar->setRange(1000, 2000);
    bar->setValue(1500);
    bar->setTextVisible(false);
    bar->setFixedHeight(14);
    bar->setStyleSheet(
        QString("QProgressBar{background:#13141B;border:1px solid #2A3347;"
                "border-radius:2px;}QProgressBar::chunk{border-radius:1px;"
                "background:%1;}")
            .arg(kChans[i].color));
    auto *val = new QLabel("—", this);
    val->setFixedWidth(48);
    val->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    val->setStyleSheet("font-family:'DejaVu Sans Mono',monospace;font-size:11px;");
    auto *rowW = new QWidget(this);
    auto *rl = new QHBoxLayout(rowW);
    rl->setContentsMargins(0, 0, 0, 0);
    rl->addWidget(name);
    rl->addWidget(bar, 1);
    rl->addWidget(val);
    cg->addWidget(rowW, i / 2, i % 2);
    m_bars.append(bar);
    m_labels.append(val);
  }
  mainLayout->addWidget(chans);

  // ---- Channel History (8 traces, 10s; aux 4-7 dotted) ----
  auto *hist = new QGroupBox(tr("Channel History"), this);
  auto *hl = new QVBoxLayout(hist);
  m_history = new RealTimeGraph(this, 8);
  m_history->setWindowSeconds(10);
  m_history->setYRange(1000, 2000);
  m_history->setMinimumHeight(150);
  for (int i = 0; i < 8; ++i) {
    m_history->setColor(i, QColor(kChans[i].color));
    if (i >= 4)
      m_history->setPenStyle(i, Qt::DotLine);  // aux channels dotted
  }
  hl->addWidget(m_history);
  mainLayout->addWidget(hist, 1);
}

void RcChannelsWidget::updateChannels(const RcData &data) {
  auto norm = [](int us) -> float {
    if (us <= 0) return NAN;  // failsafe / no data -> N/A
    return std::clamp((us - 1500) / 500.0f, -1.0f, 1.0f);
  };
  const int roll = data.channels[0], pitch = data.channels[1];
  const int thr = data.channels[2], yaw = data.channels[3];

  m_stickR->setPos(norm(roll), norm(pitch));
  m_stickL->setPos(norm(yaw), norm(thr));
  for (int i = 0; i < 4; ++i)
    m_aux[i]->setFromChannel(data.channels[4 + i]);

  for (int i = 0; i < 8; ++i) {
    const int v = data.channels[i];
    if (v <= 0) {
      m_labels[i]->setText("N/A");
      m_bars[i]->setValue(1000);
    } else {
      m_bars[i]->setValue(std::clamp(v, 1000, 2000));
      m_labels[i]->setText(QString::number(v));
    }
    if (m_history)
      m_history->appendData(v > 0 ? std::clamp(v, 1000, 2000) : 1000, i);
  }
}
