#include "LogPanel.h"

#include "core/Logger.h"
#include "core/ui/Buttons.h"

#include <QDateTime>
#include <QFile>
#include <QHBoxLayout>
#include <QScrollBar>
#include <QTextStream>
#include <QVBoxLayout>

#include <algorithm>

LogPanel::LogPanel(QWidget *parent) : QGroupBox("Log", parent) {
  auto *root = new QVBoxLayout(this);

  m_text = new QPlainTextEdit(this);
  m_text->setReadOnly(true);
  m_text->setMaximumBlockCount(m_maxLines);
  m_text->setLineWrapMode(QPlainTextEdit::NoWrap);
  // Editor chrome handled by global QSS.

  m_clock.start();  // reference for Elapsed (T+) timestamps

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

  // (No "Export" here: it opened a modal file dialog that could wedge the UI,
  // and whole-session recording is handled system-wide via Settings → record.)

  bar->addWidget(m_autoScroll);
  bar->addStretch();
  bar->addWidget(m_pauseBtn);
  bar->addWidget(clearBtn);

  root->addLayout(bar);
  root->addWidget(m_text);
}

QString LogPanel::stampFor(const Entry &e) const {
  switch (m_tsMode) {
    case Utc:
      return QDateTime::fromMSecsSinceEpoch(e.wallMs, Qt::UTC)
          .toString(QStringLiteral("hh:mm:ss.zzz"));
    case Elapsed: {
      const qint64 ms = std::max<qint64>(0, e.elapsedMs);
      const qint64 s = ms / 1000;
      return QStringLiteral("T+%1:%2:%3.%4")
          .arg(s / 3600, 2, 10, QChar('0'))
          .arg((s / 60) % 60, 2, 10, QChar('0'))
          .arg(s % 60, 2, 10, QChar('0'))
          .arg(ms % 1000, 3, 10, QChar('0'));
    }
    case Local:
    default:
      return QDateTime::fromMSecsSinceEpoch(e.wallMs)
          .toString(QStringLiteral("hh:mm:ss.zzz"));
  }
}

QString LogPanel::format(const Entry &e) const {
  return QStringLiteral("[%1] %2").arg(stampFor(e), e.msg);
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

  Entry e{QDateTime::currentMSecsSinceEpoch(),
          m_clock.isValid() ? m_clock.elapsed() : 0, msg};
  m_entries.push_back(e);
  while (static_cast<int>(m_entries.size()) > m_maxLines) m_entries.pop_front();

  m_text->appendPlainText(format(e));
  if (m_autoScroll->isChecked())
    m_text->verticalScrollBar()->setValue(
        m_text->verticalScrollBar()->maximum());
}

void LogPanel::rebuildView() {
  QString all;
  for (const auto &e : m_entries) {
    all += format(e);
    all += QLatin1Char('\n');
  }
  if (all.endsWith(QLatin1Char('\n'))) all.chop(1);
  m_text->setPlainText(all);
  if (m_autoScroll->isChecked())
    m_text->verticalScrollBar()->setValue(
        m_text->verticalScrollBar()->maximum());
}

void LogPanel::clearLog() {
  m_entries.clear();
  m_text->clear();
}

void LogPanel::setPaused(bool paused) {
  m_paused = paused;
  if (m_pauseBtn) m_pauseBtn->setText(paused ? tr("Resume") : tr("Pause"));
}

void LogPanel::setMaxLines(int maxLines) {
  m_maxLines = std::max(1, maxLines);
  m_text->setMaximumBlockCount(m_maxLines);
  while (static_cast<int>(m_entries.size()) > m_maxLines) m_entries.pop_front();
  rebuildView();
}

void LogPanel::setTimestampMode(int mode) {
  m_tsMode = mode;
  rebuildView();  // re-render all buffered lines in the new mode
}

bool LogPanel::exportToFile(const QString &path) const {
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    return false;
  QTextStream out(&f);
  // The full session buffer, unfiltered — the on-screen level filter is a view
  // concern; an exported session log should be complete.
  for (const auto &e : m_entries) out << format(e) << '\n';
  return true;
}
