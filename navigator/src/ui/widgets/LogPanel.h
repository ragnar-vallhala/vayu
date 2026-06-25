#pragma once

#include <QCheckBox>
#include <QElapsedTimer>
#include <QGroupBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QString>

#include <deque>

class LogPanel : public QGroupBox {
  Q_OBJECT

public:
  explicit LogPanel(QWidget *parent = nullptr);

  // Timestamp prefix style (Settings ▸ Logging ▸ Timestamps).
  enum TimestampMode { Local = 0, Utc = 1, Elapsed = 2 };

public slots:
  void appendLog(const QString &msg);
  void clearLog();
  // Freeze/unfreeze the scrollback view (mockup Pause); disk logging continues.
  void setPaused(bool paused);

  // ---- Settings ▸ Logging & Recording --------------------------------------
  void setMaxLines(int maxLines);    // ring-buffer cap
  void setTimestampMode(int mode);   // Local / UTC / Elapsed (T+)
  // Dump the full session buffer (unfiltered) to a text file. Returns true on
  // success; used by MainWindow's "export on disconnect".
  bool exportToFile(const QString &path) const;

private:
  // One buffered line: the wall-clock + monotonic instants it was logged at (so
  // the timestamp can be re-rendered in any mode) and the raw message text
  // (without the timestamp prefix).
  struct Entry {
    qint64 wallMs;
    qint64 elapsedMs;
    QString msg;
  };

  QString stampFor(const Entry &e) const;      // timestamp prefix in the cur mode
  QString format(const Entry &e) const;        // "[stamp] msg"
  void rebuildView();                          // re-render m_text from m_entries

  QPlainTextEdit *m_text;
  QCheckBox *m_autoScroll;
  QPushButton *m_pauseBtn = nullptr;
  bool m_paused = false;

  std::deque<Entry> m_entries;  // full ring buffer (capped at m_maxLines)
  int m_maxLines = 2000;
  int m_tsMode = Local;
  QElapsedTimer m_clock;  // for Elapsed (T+) timestamps; started at construction
};
