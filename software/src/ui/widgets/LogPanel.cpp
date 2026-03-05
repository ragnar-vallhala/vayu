#include "LogPanel.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QScrollBar>
#include <QVBoxLayout>

LogPanel::LogPanel(QWidget *parent) : QGroupBox("Log", parent) {
  auto *root = new QVBoxLayout(this);

  m_text = new QPlainTextEdit(this);
  m_text->setReadOnly(true);
  m_text->setMaximumBlockCount(MAX_LINES);
  m_text->setLineWrapMode(QPlainTextEdit::NoWrap);
  m_text->setStyleSheet("background: #0D1117; color: #A3BE8C; font-family: "
                        "'Monospace'; font-size: 12px;"
                        "border: 1px solid #2A3347;");

  auto *bar = new QHBoxLayout;
  m_autoScroll = new QCheckBox("Auto-scroll", this);
  m_autoScroll->setChecked(true);
  auto *clearBtn = new QPushButton("Clear", this);
  clearBtn->setFixedWidth(70);
  connect(clearBtn, &QPushButton::clicked, this, &LogPanel::clearLog);

  bar->addWidget(m_autoScroll);
  bar->addStretch();
  bar->addWidget(clearBtn);

  root->addLayout(bar);
  root->addWidget(m_text);
}

void LogPanel::appendLog(const QString &msg) {
  const QString ts = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
  m_text->appendPlainText(QString("[%1] %2").arg(ts, msg));
  ++m_lineCount;

  if (m_autoScroll->isChecked()) {
    m_text->verticalScrollBar()->setValue(
        m_text->verticalScrollBar()->maximum());
  }
}

void LogPanel::clearLog() {
  m_text->clear();
  m_lineCount = 0;
}
