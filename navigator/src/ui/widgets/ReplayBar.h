#pragma once

#include <QIcon>
#include <QWidget>

class ReplaySource;
class QComboBox;
class QLabel;
class QPushButton;
class QShortcut;

// Row 1 scrubber — the playback bar, ZOOMED so its full width maps to the crop
// window [t0,t1] (fine scrubbing). A warn-coloured fill runs to a round knob at
// the playhead. Click or drag anywhere to seek within the crop.
class PlaybackScrub : public QWidget {
  Q_OBJECT

public:
  explicit PlaybackScrub(QWidget *parent = nullptr);
  void setCrop(qint64 t0, qint64 t1);
  void setPosition(qint64 us);

  QSize sizeHint() const override { return {200, 18}; }
  QSize minimumSizeHint() const override { return {120, 18}; }

signals:
  void scrubbed(qint64 us);
  void scrubbingChanged(bool active);

protected:
  void paintEvent(QPaintEvent *) override;
  void mousePressEvent(QMouseEvent *) override;
  void mouseMoveEvent(QMouseEvent *) override;
  void mouseReleaseEvent(QMouseEvent *) override;

private:
  int xForPos() const;
  qint64 usForX(int x) const;
  void seekTo(int x);

  qint64 m_t0 = 0, m_t1 = 0, m_pos = 0;
  bool m_drag = false;
};

// Row 2 scrubber — the crop overview spanning the FULL timeline. One track with
// two draggable handles (in/out points) bracketing a highlighted crop band, and
// a thin playhead tick. Dragging a handle re-crops; clicking the track seeks the
// playhead (clamped into the crop).
class CropScrub : public QWidget {
  Q_OBJECT

public:
  explicit CropScrub(QWidget *parent = nullptr);
  void setDuration(qint64 us);
  void setCrop(qint64 t0, qint64 t1);
  void setPosition(qint64 us);

  QSize sizeHint() const override { return {200, 20}; }
  QSize minimumSizeHint() const override { return {120, 20}; }

signals:
  void cropChanged(qint64 t0, qint64 t1);
  void scrubbed(qint64 us);
  void scrubbingChanged(bool active);

protected:
  void paintEvent(QPaintEvent *) override;
  void mousePressEvent(QMouseEvent *) override;
  void mouseMoveEvent(QMouseEvent *) override;
  void mouseReleaseEvent(QMouseEvent *) override;
  void leaveEvent(QEvent *) override;

private:
  enum class Grab { None, Start, End, Seek };
  int xForUs(qint64 us) const;
  qint64 usForX(int x) const;
  Grab hitTest(int x) const;

  qint64 m_dur = 0, m_t0 = 0, m_t1 = 0, m_pos = 0;
  Grab m_grab = Grab::None;
};

// Transport bar for whole-GCS replay (FR-UI-19), hosted by MainWindow and shown
// only in replay. Two stacked rows matching the mockup: a play-cluster + zoomed
// playback scrubber (←/Space/→ keyboard) over a full-timeline crop overview.
// Drives a bound ReplaySource and mirrors its positionChanged back into the UI.
class ReplayBar : public QWidget {
  Q_OBJECT

public:
  explicit ReplayBar(QWidget *parent = nullptr);

  void bind(ReplaySource *src);         // nullptr unbinds
  void setLogName(const QString &name); // shown in the overview row

signals:
  void exitRequested();

private:
  void syncFromSource();
  void onPosition(qint64 us);
  void togglePlay();
  void step(qint64 deltaUs);
  void updatePlayIcon();
  void updateTimeLabels();
  static QString fmt(qint64 us); // HH:MM:SS, HH stripped under 60 min

  ReplaySource *m_src = nullptr;
  QPushButton *m_toStart = nullptr;
  QPushButton *m_back = nullptr;
  QPushButton *m_play = nullptr;
  QPushButton *m_fwd = nullptr;
  QPushButton *m_toEnd = nullptr;
  QPushButton *m_exit = nullptr;
  QComboBox *m_speed = nullptr;
  QComboBox *m_mode = nullptr; // Play once / Loop
  PlaybackScrub *m_play_scrub = nullptr;
  CropScrub *m_crop_scrub = nullptr;
  QLabel *m_curTime = nullptr;
  QLabel *m_cropEndTime = nullptr;
  QLabel *m_cropStartTime = nullptr;
  QLabel *m_totalTime = nullptr;
  QLabel *m_fileLabel = nullptr;
  QShortcut *m_scPlay = nullptr;
  QShortcut *m_scBack = nullptr;
  QShortcut *m_scFwd = nullptr;
  QIcon m_playIcon;  // ▶ — shown while paused
  QIcon m_pauseIcon; // ⏸ — shown while playing
  bool m_scrubbing = false;
};
