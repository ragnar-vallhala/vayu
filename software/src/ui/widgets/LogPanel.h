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

private:
  QPlainTextEdit *m_text;
  QCheckBox *m_autoScroll;
  int m_lineCount = 0;
  static constexpr int MAX_LINES = 2000;
};
