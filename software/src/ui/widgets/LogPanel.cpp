#include "LogPanel.h"

#include "core/Logger.h"
#include "core/Notify.h"
#include "core/ui/Buttons.h"

#include <QDateTime>
#include <QFile>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QScrollBar>
#include <QTextStream>
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
  m_pauseBtn = new ui::GhostButton(tr("Pause"), this);
  m_pauseBtn->setCheckable(true);
  m_pauseBtn->setFixedWidth(70);
  m_pauseBtn->setToolTip(tr("Freeze the scrollback view (disk log keeps writing)"));
  connect(m_pauseBtn, &QPushButton::toggled, this, &LogPanel::setPaused);

  auto *clearBtn = new ui::GhostButton(tr("Clear"), this);
  clearBtn->setFixedWidth(70);
  clearBtn->setToolTip(tr("Empty the scrollback (persistent log on disk is kept)"));
  connect(clearBtn, &QPushButton::clicked, this, &LogPanel::clearLog);

  auto *exportBtn = new ui::GhostButton(tr("Export"), this);
  exportBtn->setFixedWidth(70);
  exportBtn->setToolTip(tr("Save the current scrollback to a text file"));
  connect(exportBtn, &QPushButton::clicked, this, &LogPanel::exportLog);

  bar->addWidget(m_autoScroll);
  bar->addStretch();
  bar->addWidget(m_pauseBtn);
  bar->addWidget(clearBtn);
  bar->addWidget(exportBtn);

  root->addLayout(bar);
  root->addWidget(m_text);
}

void LogPanel::appendLog(const QString &msg) {
  // Always mirror to the persistent on-disk log. Logger handles its own
  // file rotation + thread-safety; the call is cheap if logging is disabled.
  // We send the un-timestamped msg because Logger prepends its own UTC
  // ISO timestamp. This runs even while the view is paused.
  Logger::log(msg);

  // Paused: freeze the scrollback view (mockup Pause) — disk logging above
  // still captures everything.
  if (m_paused) return;

  const QString ts = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
  const QString line = QString("[%1] %2").arg(ts, msg);
  m_text->appendPlainText(line);
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

void LogPanel::setPaused(bool paused) {
  m_paused = paused;
  if (m_pauseBtn) m_pauseBtn->setText(paused ? tr("Resume") : tr("Pause"));
}

void LogPanel::exportLog() {
  const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss");
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Export Log"), QString("navigator-log-%1.txt").arg(stamp),
      tr("Text files (*.txt)"));
  if (path.isEmpty()) return;
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
    Notify::error(this, tr("Could not write %1").arg(path));
    return;
  }
  QTextStream(&f) << m_text->toPlainText();
  f.close();
  Notify::ok(this, tr("Wrote %1").arg(path));
}
