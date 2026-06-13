#pragma once

#include <QCheckBox>
#include <QGroupBox>
#include <QPlainTextEdit>
#include <QPushButton>

class LogPanel : public QGroupBox {
  Q_OBJECT

public:
  explicit LogPanel(QWidget *parent = nullptr);

public slots:
  void appendLog(const QString &msg);
  void clearLog();
  // Freeze/unfreeze the scrollback view (mockup Pause); disk logging continues.
  void setPaused(bool paused);
  // Save the current scrollback to a text file (mockup Export).
  void exportLog();

private:
  QPlainTextEdit *m_text;
  QCheckBox *m_autoScroll;
  QPushButton *m_pauseBtn = nullptr;
  bool m_paused = false;
  int m_lineCount = 0;
  static constexpr int MAX_LINES = 2000;
};
