#include "LogPanel.h"

#include "core/Logger.h"
#include "core/ui/Buttons.h"

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
  // Editor chrome handled by global QSS.

  auto *bar = new QHBoxLayout;
  m_autoScroll = new QCheckBox("Auto-scroll", this);
  m_autoScroll->setChecked(true);
  m_autoScroll->setToolTip(
      tr("Pin the view to the newest line"));
  auto *clearBtn = new ui::GhostButton(tr("Clear"), this);
  clearBtn->setFixedWidth(70);
  clearBtn->setToolTip(tr("Empty the scrollback (persistent log on disk is kept)"));
  connect(clearBtn, &QPushButton::clicked, this, &LogPanel::clearLog);

  bar->addWidget(m_autoScroll);
  bar->addStretch();
  bar->addWidget(clearBtn);

  root->addLayout(bar);
  root->addWidget(m_text);
}

void LogPanel::appendLog(const QString &msg) {
  const QString ts = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
  const QString line = QString("[%1] %2").arg(ts, msg);
  m_text->appendPlainText(line);
  ++m_lineCount;

  // Mirror to the persistent on-disk log. Logger handles its own
  // file rotation + thread-safety; the call is cheap if logging is
  // disabled. We send the un-timestamped msg because Logger prepends
  // its own UTC ISO timestamp.
  Logger::log(msg);

  if (m_autoScroll->isChecked()) {
    m_text->verticalScrollBar()->setValue(
        m_text->verticalScrollBar()->maximum());
  }
}

void LogPanel::clearLog() {
  m_text->clear();
  m_lineCount = 0;
}
