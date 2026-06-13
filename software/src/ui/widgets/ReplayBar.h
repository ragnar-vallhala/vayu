#pragma once

#include <QWidget>

class ReplaySource;
class QComboBox;
class QLabel;
class QPushButton;
class QSlider;
class QCheckBox;

// Transport bar for whole-GCS replay (FR-UI-19), hosted by MainWindow and shown
// only in replay. Two scrubbers — a crop overview (start/end over the full
// timeline → setRange) and a playhead (within the crop → seek) — plus
// play/pause, a speed selector, a loop toggle, step-to-edge, and Exit. Drives a
// bound ReplaySource directly and mirrors its positionChanged back into the UI.
//
// The mockup's draggable dual-handle crop is approximated here with start/end
// sliders; the polished handle drag is a later visual enhancement.
class ReplayBar : public QWidget {
  Q_OBJECT

public:
  explicit ReplayBar(QWidget *parent = nullptr);

  // Bind (or unbind with nullptr) the source this bar controls.
  void bind(ReplaySource *src);

signals:
  void exitRequested();

private:
  void syncFromSource();
  void onPosition(qint64 us);
  QString fmt(qint64 us) const;

  ReplaySource *m_src = nullptr;
  QPushButton *m_play = nullptr;
  QPushButton *m_exit = nullptr;
  QPushButton *m_toStart = nullptr;
  QPushButton *m_toEnd = nullptr;
  QComboBox *m_speed = nullptr;
  QCheckBox *m_loop = nullptr;
  QSlider *m_playhead = nullptr;
  QSlider *m_cropStart = nullptr;
  QSlider *m_cropEnd = nullptr;
  QLabel *m_time = nullptr;
  bool m_scrubbing = false;
};
