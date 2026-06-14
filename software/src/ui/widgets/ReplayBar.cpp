#include "ReplayBar.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QPolygonF>
#include <QPushButton>
#include <QShortcut>
#include <QVBoxLayout>

#include "ReplaySource.h"

namespace {
// Palette lifted from the mockup (docs/ui-mockup/styles.css).
const QColor kAccent(97, 175, 239);
const QColor kWarn(209, 154, 102);
const QColor kBase(19, 20, 27);
const QColor kBorder(42, 51, 71);
const QColor kHandleEdge(207, 227, 255);
const QColor kCropBand(97, 175, 239, 46);  // rgba(accent, .18)

constexpr int kMargin = 7;             // keeps edge handles/knob fully on-widget
constexpr qint64 kStepUs = 5'000'000;  // ←/→ and step buttons jump 5 s

int lerpX(int left, int right, double f) {
  return left + int(qBound(0.0, f, 1.0) * (right - left));
}

// Transport glyphs painted as vector shapes rather than Unicode media chars,
// whose font metrics render off-centre in a button box. All shapes are drawn
// symmetrically inside the vertical band [top,bot] so they sit dead-centre.
enum class Glyph { ToStart, Back, Play, Pause, Fwd, ToEnd };

QIcon mediaIcon(Glyph g, const QColor &c) {
  constexpr int S = 64;
  constexpr qreal cy = S / 2.0, top = 18, bot = 46;  // 28 px tall, centred
  QPixmap pm(S, S);
  pm.fill(Qt::transparent);
  QPainter p(&pm);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.setPen(Qt::NoPen);
  p.setBrush(c);

  auto rtri = [&](qreal x) {  // right-pointing triangle, apex at x+15
    p.drawPolygon(QPolygonF{{x, top}, {x, bot}, {x + 15, cy}});
  };
  auto ltri = [&](qreal x) {  // left-pointing triangle, apex at x-15
    p.drawPolygon(QPolygonF{{x, top}, {x, bot}, {x - 15, cy}});
  };
  auto bar = [&](qreal x) { p.drawRoundedRect(QRectF(x, top, 6, bot - top), 1, 1); };

  switch (g) {
    case Glyph::Play:    rtri(24); break;
    case Glyph::Pause:   bar(24); bar(34); break;
    case Glyph::Back:    ltri(33); ltri(47); break;
    case Glyph::Fwd:     rtri(18); rtri(32); break;
    case Glyph::ToStart: bar(14); ltri(38); ltri(52); break;
    case Glyph::ToEnd:   rtri(12); rtri(26); bar(44); break;
  }
  return QIcon(pm);
}
}  // namespace

// ===========================================================================
// PlaybackScrub — row 1, zoomed to the crop window
// ===========================================================================
PlaybackScrub::PlaybackScrub(QWidget *parent) : QWidget(parent) {
  setCursor(Qt::PointingHandCursor);
}

void PlaybackScrub::setCrop(qint64 t0, qint64 t1) {
  m_t0 = t0;
  m_t1 = qMax(t0, t1);
  update();
}

void PlaybackScrub::setPosition(qint64 us) {
  m_pos = qBound(m_t0, us, m_t1);
  update();
}

int PlaybackScrub::xForPos() const {
  const qint64 span = m_t1 - m_t0;
  const double f = span > 0 ? double(m_pos - m_t0) / double(span) : 0.0;
  return lerpX(kMargin, width() - kMargin, f);
}

qint64 PlaybackScrub::usForX(int x) const {
  const int span = width() - 2 * kMargin;
  if (span <= 0) return m_t0;
  const double f = double(qBound(kMargin, x, width() - kMargin) - kMargin) /
                   double(span);
  return m_t0 + qint64(f * double(m_t1 - m_t0));
}

void PlaybackScrub::seekTo(int x) {
  m_pos = qBound(m_t0, usForX(x), m_t1);
  update();
  emit scrubbed(m_pos);
}

void PlaybackScrub::mousePressEvent(QMouseEvent *e) {
  if (e->button() != Qt::LeftButton) return;
  m_drag = true;
  emit scrubbingChanged(true);
  seekTo(int(e->position().x()));
}

void PlaybackScrub::mouseMoveEvent(QMouseEvent *e) {
  if (m_drag) seekTo(int(e->position().x()));
}

void PlaybackScrub::mouseReleaseEvent(QMouseEvent *) {
  if (m_drag) {
    m_drag = false;
    emit scrubbingChanged(false);
  }
}

void PlaybackScrub::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);

  const int cy = height() / 2, th = 9;
  const QRect track(kMargin, cy - th / 2, width() - 2 * kMargin, th);
  p.setPen(QPen(kBorder, 1));
  p.setBrush(kBase);
  p.drawRoundedRect(track, 3, 3);

  const int xk = xForPos();
  p.setPen(Qt::NoPen);
  p.setBrush(kWarn);
  p.drawRoundedRect(QRect(kMargin, cy - th / 2, qMax(0, xk - kMargin), th), 2, 2);

  // Round white knob at the playhead.
  p.setBrush(Qt::white);
  p.drawEllipse(QPoint(xk, cy), 6, 6);
}

// ===========================================================================
// CropScrub — row 2, full timeline with two handles
// ===========================================================================
CropScrub::CropScrub(QWidget *parent) : QWidget(parent) {
  setMouseTracking(true);
  setCursor(Qt::PointingHandCursor);
}

void CropScrub::setDuration(qint64 us) {
  m_dur = qMax<qint64>(0, us);
  update();
}

void CropScrub::setCrop(qint64 t0, qint64 t1) {
  m_t0 = qBound<qint64>(0, t0, m_dur);
  m_t1 = qBound<qint64>(m_t0, t1, m_dur);
  update();
}

void CropScrub::setPosition(qint64 us) {
  m_pos = qBound<qint64>(0, us, m_dur);
  update();
}

int CropScrub::xForUs(qint64 us) const {
  const double f = m_dur > 0 ? double(qBound<qint64>(0, us, m_dur)) / m_dur : 0.0;
  return lerpX(kMargin, width() - kMargin, f);
}

qint64 CropScrub::usForX(int x) const {
  const int span = width() - 2 * kMargin;
  if (m_dur <= 0 || span <= 0) return 0;
  const double f = double(qBound(kMargin, x, width() - kMargin) - kMargin) /
                   double(span);
  return qint64(f * m_dur);
}

CropScrub::Grab CropScrub::hitTest(int x) const {
  const int xs = xForUs(m_t0), xe = xForUs(m_t1);
  const int tol = 7, ds = qAbs(x - xs), de = qAbs(x - xe);
  if (ds <= tol && ds <= de) return Grab::Start;
  if (de <= tol) return Grab::End;
  return Grab::Seek;
}

void CropScrub::mousePressEvent(QMouseEvent *e) {
  if (e->button() != Qt::LeftButton || m_dur <= 0) return;
  m_grab = hitTest(int(e->position().x()));
  if (m_grab == Grab::Seek) emit scrubbingChanged(true);
  mouseMoveEvent(e);
}

void CropScrub::mouseMoveEvent(QMouseEvent *e) {
  const int x = int(e->position().x());
  if (m_grab == Grab::None) {  // hover feedback
    setCursor(hitTest(x) == Grab::Seek ? Qt::PointingHandCursor
                                       : Qt::SplitHCursor);
    return;
  }
  const qint64 t = usForX(x);
  const qint64 minGap = qMax<qint64>(1, m_dur / 1000);
  if (m_grab == Grab::Start) {
    m_t0 = qBound<qint64>(0, t, m_t1 - minGap);
    update();
    emit cropChanged(m_t0, m_t1);
  } else if (m_grab == Grab::End) {
    m_t1 = qBound<qint64>(m_t0 + minGap, t, m_dur);
    update();
    emit cropChanged(m_t0, m_t1);
  } else {  // Seek
    m_pos = qBound(m_t0, t, m_t1);
    update();
    emit scrubbed(m_pos);
  }
}

void CropScrub::mouseReleaseEvent(QMouseEvent *) {
  if (m_grab == Grab::Seek) emit scrubbingChanged(false);
  m_grab = Grab::None;
}

void CropScrub::leaveEvent(QEvent *) { unsetCursor(); }

void CropScrub::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);

  const int cy = height() / 2, th = 7;
  const int top = cy - th / 2;
  const QRect track(kMargin, top, width() - 2 * kMargin, th);
  p.setPen(QPen(kBorder, 1));
  p.setBrush(kBase);
  p.drawRoundedRect(track, 3, 3);
  if (m_dur <= 0) return;

  const int xs = xForUs(m_t0), xe = xForUs(m_t1), xp = xForUs(m_pos);

  // Crop band + its accent edges.
  p.setPen(Qt::NoPen);
  p.setBrush(kCropBand);
  p.drawRect(QRect(xs, top, qMax(1, xe - xs), th));
  p.setPen(QPen(kAccent, 1));
  p.drawLine(xs, top, xs, top + th);
  p.drawLine(xe, top, xe, top + th);

  // Playhead tick (thin white bar over the full timeline).
  p.setPen(Qt::NoPen);
  p.setBrush(Qt::white);
  p.drawRect(QRect(xp - 1, top - 2, 2, th + 4));

  // Crop handles.
  auto handle = [&](int x) {
    p.setPen(QPen(kHandleEdge, 1));
    p.setBrush(kAccent);
    p.drawRoundedRect(QRect(x - 3, cy - 9, 7, 18), 2, 2);
    p.setPen(Qt::NoPen);
  };
  handle(xs);
  handle(xe);
}

// ===========================================================================
// ReplayBar
// ===========================================================================
ReplayBar::ReplayBar(QWidget *parent) : QWidget(parent) {
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(8, 4, 8, 4);
  v->setSpacing(5);

  auto timeLabel = [this] {
    auto *l = new QLabel("0:00", this);
    l->setMinimumWidth(46);
    l->setAlignment(Qt::AlignCenter);
    return l;
  };

  // ---- Row 1: play cluster + zoomed playback scrubber ----
  auto *r1 = new QHBoxLayout();
  m_toStart = new QPushButton(this);
  m_back = new QPushButton(this);
  m_play = new QPushButton(this);
  m_fwd = new QPushButton(this);
  m_toEnd = new QPushButton(this);
  m_play->setObjectName("rpPlay");  // stable handle for tests (icon, no text)
  m_toStart->setToolTip(tr("Skip to crop start"));
  m_back->setToolTip(tr("Step back 5 s (←)"));
  m_play->setToolTip(tr("Play / Pause (Space)"));
  m_fwd->setToolTip(tr("Step forward 5 s (→)"));
  m_toEnd->setToolTip(tr("Skip to crop end"));

  // Vector glyphs (font-independent, vertically centred). Play uses the accent.
  const QColor glyph(0xAB, 0xB2, 0xBF);  // matches the toolbar's muted text
  m_playIcon = mediaIcon(Glyph::Play, kAccent);
  m_pauseIcon = mediaIcon(Glyph::Pause, kAccent);
  m_toStart->setIcon(mediaIcon(Glyph::ToStart, glyph));
  m_back->setIcon(mediaIcon(Glyph::Back, glyph));
  m_play->setIcon(m_playIcon);
  m_fwd->setIcon(mediaIcon(Glyph::Fwd, glyph));
  m_toEnd->setIcon(mediaIcon(Glyph::ToEnd, glyph));
  for (auto *b : {m_toStart, m_back, m_play, m_fwd, m_toEnd}) {
    b->setIconSize(QSize(15, 15));
    b->setFocusPolicy(Qt::NoFocus);
    r1->addWidget(b);
  }

  m_curTime = timeLabel();
  r1->addWidget(m_curTime);
  m_play_scrub = new PlaybackScrub(this);
  r1->addWidget(m_play_scrub, 1);
  m_cropEndTime = timeLabel();
  r1->addWidget(m_cropEndTime);

  r1->addSpacing(8);
  r1->addWidget(new QLabel(tr("Speed:"), this));
  m_speed = new QComboBox(this);
  for (double x : {0.25, 0.5, 1.0, 2.0, 4.0})
    m_speed->addItem(QString::number(x) + "x", x);
  m_speed->setCurrentIndex(2);  // 1.0x
  r1->addWidget(m_speed);

  m_mode = new QComboBox(this);
  m_mode->addItem(tr("Play once"), false);
  m_mode->addItem(tr("Loop"), true);
  m_mode->setToolTip(tr("Play through once, or loop the crop region"));
  r1->addWidget(m_mode);

  m_exit = new QPushButton(tr("Exit Replay"), this);
  r1->addWidget(m_exit);
  v->addLayout(r1);

  // ---- Row 2: full-timeline crop overview ----
  auto *r2 = new QHBoxLayout();
  m_fileLabel = new QLabel(this);
  m_fileLabel->setStyleSheet(QStringLiteral("color:#D19A66;"));
  r2->addWidget(m_fileLabel);
  r2->addWidget(new QLabel(tr("Crop:"), this));
  m_cropStartTime = timeLabel();
  r2->addWidget(m_cropStartTime);
  m_crop_scrub = new CropScrub(this);
  r2->addWidget(m_crop_scrub, 1);
  m_totalTime = timeLabel();
  r2->addWidget(m_totalTime);
  v->addLayout(r2);

  // ---- Keyboard shortcuts (whole replay window) ----
  m_scPlay = new QShortcut(QKeySequence(Qt::Key_Space), this);
  m_scBack = new QShortcut(QKeySequence(Qt::Key_Left), this);
  m_scFwd = new QShortcut(QKeySequence(Qt::Key_Right), this);
  for (auto *s : {m_scPlay, m_scBack, m_scFwd}) s->setContext(Qt::WindowShortcut);
  connect(m_scPlay, &QShortcut::activated, this, &ReplayBar::togglePlay);
  connect(m_scBack, &QShortcut::activated, this, [this] { step(-kStepUs); });
  connect(m_scFwd, &QShortcut::activated, this, [this] { step(kStepUs); });

  // ---- Wiring (acts only when a source is bound) ----
  connect(m_play, &QPushButton::clicked, this, &ReplayBar::togglePlay);
  connect(m_back, &QPushButton::clicked, this, [this] { step(-kStepUs); });
  connect(m_fwd, &QPushButton::clicked, this, [this] { step(kStepUs); });
  connect(m_toStart, &QPushButton::clicked, this, [this] {
    if (m_src) m_src->seek(m_src->rangeStart());
  });
  connect(m_toEnd, &QPushButton::clicked, this, [this] {
    if (m_src) m_src->seek(m_src->rangeEnd());
  });
  connect(m_exit, &QPushButton::clicked, this, &ReplayBar::exitRequested);
  connect(m_speed, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this](int) {
            if (m_src) m_src->setSpeed(m_speed->currentData().toDouble());
          });
  connect(m_mode, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this](int) {
            if (m_src) m_src->setLoop(m_mode->currentData().toBool());
          });

  // Playback knob (row 1) — fine scrub within the crop.
  connect(m_play_scrub, &PlaybackScrub::scrubbed, this, [this](qint64 us) {
    if (m_src) m_src->seek(us);
  });
  connect(m_play_scrub, &PlaybackScrub::scrubbingChanged, this,
          [this](bool on) { m_scrubbing = on; });

  // Crop overview (row 2) — handles re-crop, track clicks seek.
  connect(m_crop_scrub, &CropScrub::cropChanged, this,
          [this](qint64 a, qint64 b) {
            if (m_src) m_src->setRange(a, b);
            m_play_scrub->setCrop(a, b);
            updateTimeLabels();
          });
  connect(m_crop_scrub, &CropScrub::scrubbed, this, [this](qint64 us) {
    if (m_src) m_src->seek(us);
  });
  connect(m_crop_scrub, &CropScrub::scrubbingChanged, this,
          [this](bool on) { m_scrubbing = on; });
}

void ReplayBar::setLogName(const QString &name) {
  m_fileLabel->setText(name);
}

void ReplayBar::bind(ReplaySource *src) {
  if (m_src) m_src->disconnect(this);
  m_src = src;
  const bool on = (m_src != nullptr);
  m_scPlay->setEnabled(on);
  m_scBack->setEnabled(on);
  m_scFwd->setEnabled(on);
  if (!m_src) return;
  connect(m_src, &ReplaySource::positionChanged, this, &ReplayBar::onPosition);
  connect(m_src, &ReplaySource::finished, this, [this] { updatePlayIcon(); });
  syncFromSource();
}

void ReplayBar::syncFromSource() {
  const qint64 t0 = m_src->rangeStart(), t1 = m_src->rangeEnd();
  m_crop_scrub->setDuration(m_src->durationUs());
  m_crop_scrub->setCrop(t0, t1);
  m_play_scrub->setCrop(t0, t1);
  m_mode->setCurrentIndex(m_src->loop() ? 1 : 0);
  onPosition(m_src->positionUs());
  updatePlayIcon();
}

void ReplayBar::togglePlay() {
  if (!m_src) return;
  if (m_src->isPlaying()) m_src->pause();
  else m_src->play();
  updatePlayIcon();
}

void ReplayBar::step(qint64 deltaUs) {
  if (m_src) m_src->seek(m_src->positionUs() + deltaUs);
}

void ReplayBar::updatePlayIcon() {
  m_play->setIcon(m_src && m_src->isPlaying() ? m_pauseIcon : m_playIcon);
}

void ReplayBar::updateTimeLabels() {
  if (!m_src) return;
  m_curTime->setText(fmt(m_src->positionUs()));
  m_cropStartTime->setText(fmt(m_src->rangeStart()));
  m_cropEndTime->setText(fmt(m_src->rangeEnd()));
  m_totalTime->setText(fmt(m_src->durationUs()));
}

void ReplayBar::onPosition(qint64 us) {
  if (!m_scrubbing) {
    m_play_scrub->setPosition(us);
    m_crop_scrub->setPosition(us);
  }
  updateTimeLabels();
}

QString ReplayBar::fmt(qint64 us) {
  const qint64 total = qMax<qint64>(0, us) / 1'000'000;
  const qint64 h = total / 3600, m = (total % 3600) / 60, s = total % 60;
  if (h > 0)
    return QString("%1:%2:%3")
        .arg(h)
        .arg(m, 2, 10, QChar('0'))
        .arg(s, 2, 10, QChar('0'));
  return QString("%1:%2").arg(m).arg(s, 2, 10, QChar('0'));
}
