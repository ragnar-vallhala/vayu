#include "ReplayBar.h"

#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>

#include "ReplaySource.h"

ReplayBar::ReplayBar(QWidget *parent) : QWidget(parent) {
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(8, 4, 8, 4);

  // ---- Transport row ----
  auto *row = new QHBoxLayout();
  m_toStart = new QPushButton("⏮", this);
  m_play = new QPushButton("▶", this);
  m_toEnd = new QPushButton("⏭", this);
  m_toStart->setToolTip(tr("Jump to crop start"));
  m_toEnd->setToolTip(tr("Jump to crop end"));
  m_play->setToolTip(tr("Play / Pause"));
  row->addWidget(m_toStart);
  row->addWidget(m_play);
  row->addWidget(m_toEnd);

  row->addSpacing(12);
  row->addWidget(new QLabel(tr("Speed:"), this));
  m_speed = new QComboBox(this);
  for (double x : {0.25, 0.5, 1.0, 2.0, 4.0})
    m_speed->addItem(QString::number(x) + "x", x);
  m_speed->setCurrentIndex(2);  // 1.0x
  row->addWidget(m_speed);

  m_loop = new QCheckBox(tr("Loop crop"), this);
  row->addWidget(m_loop);

  row->addStretch();
  m_time = new QLabel("0.00 / 0.00 s", this);
  row->addWidget(m_time);
  m_exit = new QPushButton(tr("Exit Replay"), this);
  row->addWidget(m_exit);
  v->addLayout(row);

  // ---- Playhead (within crop) ----
  m_playhead = new QSlider(Qt::Horizontal, this);
  m_playhead->setToolTip(tr("Playhead — drag to scrub within the crop"));
  v->addWidget(m_playhead);

  // ---- Crop overview (start / end over the full timeline) ----
  auto *cropRow = new QHBoxLayout();
  cropRow->addWidget(new QLabel(tr("Crop:"), this));
  m_cropStart = new QSlider(Qt::Horizontal, this);
  m_cropEnd = new QSlider(Qt::Horizontal, this);
  m_cropStart->setToolTip(tr("Crop in-point"));
  m_cropEnd->setToolTip(tr("Crop out-point"));
  cropRow->addWidget(m_cropStart);
  cropRow->addWidget(m_cropEnd);
  v->addLayout(cropRow);

  // ---- Wiring (acts only when a source is bound) ----
  connect(m_play, &QPushButton::clicked, this, [this] {
    if (!m_src) return;
    if (m_src->isPlaying()) m_src->pause();
    else m_src->play();
    m_play->setText(m_src->isPlaying() ? "⏸" : "▶");
  });
  connect(m_exit, &QPushButton::clicked, this, &ReplayBar::exitRequested);
  connect(m_toStart, &QPushButton::clicked, this, [this] {
    if (m_src) m_src->seek(m_src->rangeStart());
  });
  connect(m_toEnd, &QPushButton::clicked, this, [this] {
    if (m_src) m_src->seek(m_src->rangeEnd());
  });
  connect(m_speed, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this](int) {
            if (m_src) m_src->setSpeed(m_speed->currentData().toDouble());
          });
  connect(m_loop, &QCheckBox::toggled, this, [this](bool on) {
    if (m_src) m_src->setLoop(on);
  });
  connect(m_playhead, &QSlider::sliderPressed, this,
          [this] { m_scrubbing = true; });
  connect(m_playhead, &QSlider::sliderReleased, this,
          [this] { m_scrubbing = false; });
  connect(m_playhead, &QSlider::valueChanged, this, [this](int v) {
    if (m_src && m_scrubbing)
      m_src->seek(m_src->rangeStart() + v);
  });
  connect(m_cropStart, &QSlider::valueChanged, this, [this](int) {
    if (m_src)
      m_src->setRange(m_cropStart->value(), m_cropEnd->value());
  });
  connect(m_cropEnd, &QSlider::valueChanged, this, [this](int) {
    if (m_src)
      m_src->setRange(m_cropStart->value(), m_cropEnd->value());
  });
}

void ReplayBar::bind(ReplaySource *src) {
  if (m_src)
    m_src->disconnect(this);
  m_src = src;
  if (!m_src)
    return;
  connect(m_src, &ReplaySource::positionChanged, this, &ReplayBar::onPosition);
  connect(m_src, &ReplaySource::finished, this,
          [this] { m_play->setText("▶"); });
  syncFromSource();
}

void ReplayBar::syncFromSource() {
  const qint64 dur = m_src->durationUs();
  m_cropStart->setRange(0, int(dur));
  m_cropEnd->setRange(0, int(dur));
  m_cropStart->setValue(0);
  m_cropEnd->setValue(int(dur));
  m_playhead->setRange(0, int(m_src->rangeEnd() - m_src->rangeStart()));
  m_loop->setChecked(m_src->loop());
  onPosition(m_src->positionUs());
  m_play->setText(m_src->isPlaying() ? "⏸" : "▶");
}

void ReplayBar::onPosition(qint64 us) {
  const qint64 span = m_src->rangeEnd() - m_src->rangeStart();
  m_playhead->setRange(0, int(span));
  if (!m_scrubbing) {
    QSignalBlocker b(m_playhead);
    m_playhead->setValue(int(us - m_src->rangeStart()));
  }
  m_time->setText(fmt(us) + " / " + fmt(m_src->durationUs()) + " s");
}

QString ReplayBar::fmt(qint64 us) const {
  return QString::number(double(us) / 1'000'000.0, 'f', 2);
}
