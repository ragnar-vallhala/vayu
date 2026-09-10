#pragma once

#include <QElapsedTimer>
#include <QList>
#include <QString>

#include "ITelemetrySource.h"
#include "RecordFormat.h"

class QTimer;

// Replays a recorded .bin (FR-UI-19) as an ITelemetrySource so the whole GCS
// can be driven from a log exactly as if live. Frame timestamps are normalised
// to a 0-based playhead; a QTimer-driven clock — decoupled from wall-clock and
// scaled by speed — emits frames as their virtual time falls due. A crop range
// can loop a sub-span. The time-advance core (advanceTo) is pure and emits
// frames deterministically, so playback is unit-testable without real timers.
class ReplaySource : public ITelemetrySource {
  Q_OBJECT

public:
  explicit ReplaySource(QObject *parent = nullptr);
  ~ReplaySource() override;

  bool open(const QString &path); // loads frames + header; seeks to start
  bool isOpen() const { return !m_frames.isEmpty(); }
  RecordFormat::Header header() const { return m_header; }

  qint64 durationUs() const { return m_duration; }
  qint64 positionUs() const { return m_pos; }
  int frameCount() const { return int(m_frames.size()); }

  // Crop range within [0, duration]; loop wraps within it. Defaults to the
  // whole log. Re-seeks to t0 if the playhead falls outside the new range.
  void setRange(qint64 t0, qint64 t1);
  qint64 rangeStart() const { return m_r0; }
  qint64 rangeEnd() const { return m_r1; }

  void setLoop(bool on) { m_loop = on; }
  bool loop() const { return m_loop; }

  void setSpeed(double x); // clamped > 0
  double speed() const { return m_speed; }

  void play();
  void pause();
  bool isPlaying() const;

  void seek(qint64 us); // clamped into the crop range

  // Emit every frame whose timestamp is due at virtual time `targetUs` (capped
  // at the crop end), advance the playhead, and wrap (loop) or stop (finished)
  // at the end. Returns the number of frames emitted. Pure w.r.t. wall-clock.
  int advanceTo(qint64 targetUs);

signals:
  void positionChanged(qint64 us);
  void finished(); // reached the crop end with loop off

private:
  void onTick();
  void resetCursor(); // point the cursor at the first frame >= m_pos

  struct F {
    qint64 tUs;
    QByteArray bytes;
  };
  QList<F> m_frames; // normalised to 0-based, ascending tUs
  RecordFormat::Header m_header;
  qint64 m_duration = 0;
  qint64 m_pos = 0;
  qint64 m_r0 = 0;
  qint64 m_r1 = 0;
  int m_cursor = 0;
  double m_speed = 1.0;
  bool m_loop = false;
  QTimer *m_timer = nullptr;
  QElapsedTimer m_wall;
};
