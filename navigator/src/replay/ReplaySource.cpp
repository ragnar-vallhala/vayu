#include "ReplaySource.h"

#include <algorithm>

#include <QTimer>

#include "RecordSink.h" // RecordReader

namespace {
constexpr int kTickMs = 20; // 50 Hz playback clock
}

ReplaySource::ReplaySource(QObject *parent) : ITelemetrySource(parent) {
  m_timer = new QTimer(this);
  m_timer->setInterval(kTickMs);
  connect(m_timer, &QTimer::timeout, this, &ReplaySource::onTick);
}

ReplaySource::~ReplaySource() = default;

bool ReplaySource::open(const QString &path) {
  RecordReader reader;
  if (!reader.open(path))
    return false;
  m_header = reader.header();

  QList<F> frames;
  RecordFormat::Frame f;
  while (reader.next(f))
    frames.append({qint64(f.tUs), f.bytes});
  if (frames.isEmpty())
    return false;

  // Normalise so the first frame sits at t=0; only inter-frame deltas matter.
  const qint64 base = frames.first().tUs;
  for (F &fr : frames)
    fr.tUs -= base;
  m_frames = std::move(frames);
  m_duration = m_frames.last().tUs;

  m_r0 = 0;
  m_r1 = m_duration;
  m_speed = 1.0;
  seek(0);
  return true;
}

void ReplaySource::setRange(qint64 t0, qint64 t1) {
  t0 = std::clamp<qint64>(t0, 0, m_duration);
  t1 = std::clamp<qint64>(t1, 0, m_duration);
  if (t1 < t0)
    std::swap(t0, t1);
  m_r0 = t0;
  m_r1 = t1;
  if (m_pos < m_r0 || m_pos > m_r1)
    seek(m_r0);
}

void ReplaySource::setSpeed(double x) { m_speed = x > 0.0 ? x : m_speed; }

void ReplaySource::resetCursor() {
  // First frame at or after the playhead.
  m_cursor =
      int(std::lower_bound(m_frames.begin(), m_frames.end(), m_pos,
                           [](const F &fr, qint64 v) { return fr.tUs < v; }) -
          m_frames.begin());
}

void ReplaySource::seek(qint64 us) {
  m_pos = std::clamp<qint64>(us, m_r0, m_r1);
  resetCursor();
  emit positionChanged(m_pos);
}

int ReplaySource::advanceTo(qint64 targetUs) {
  if (m_frames.isEmpty())
    return 0;
  const qint64 end = std::min(targetUs, m_r1);
  int emitted = 0;
  while (m_cursor < m_frames.size() && m_frames[m_cursor].tUs <= end) {
    emit bytesReceived(m_frames[m_cursor].bytes);
    ++m_cursor;
    ++emitted;
  }
  m_pos = std::max(m_pos, end);
  emit positionChanged(m_pos);

  if (m_pos >= m_r1) {
    if (m_loop) {
      seek(m_r0); // wrap; the next tick resumes emitting from the crop start
    } else {
      pause();
      emit finished();
    }
  }
  return emitted;
}

void ReplaySource::play() {
  if (m_frames.isEmpty() || m_timer->isActive())
    return;
  if (m_pos >= m_r1)
    seek(m_r0); // restart from the crop start if parked at the end
  m_wall.restart();
  m_timer->start();
}

void ReplaySource::pause() { m_timer->stop(); }

bool ReplaySource::isPlaying() const { return m_timer->isActive(); }

void ReplaySource::onTick() {
  const qint64 wallUs = m_wall.nsecsElapsed() / 1000;
  m_wall.restart();
  advanceTo(m_pos + qint64(wallUs * m_speed));
}
