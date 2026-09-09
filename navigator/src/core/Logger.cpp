#include "Logger.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QMutex>
#include <QMutexLocker>
#include <QStandardPaths>
#include <QTextStream>

namespace Logger {

namespace {

QMutex g_mtx;
Config g_cfg;
QString g_filePath;
QFile g_file;
QTextStream g_out;
bool g_initialized = false;

QString defaultLogDir() {
  const QString base =
      QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
  return base.isEmpty() ? QDir::tempPath() + "/vayu-gcs/logs" : base + "/logs";
}

QString dateStampedFile(const QString &dir) {
  return QDir(dir).absoluteFilePath(
      "navigator-" + QDateTime::currentDateTimeUtc().toString("yyyy-MM-dd") +
      ".log");
}

// Drop the oldest files until we're under maxFiles. Matches navigator-*.log.
void enforceMaxFiles(const QString &dir, int maxFiles) {
  if (maxFiles <= 0)
    return;
  QDir d(dir);
  d.setNameFilters({"navigator-*.log*"});
  d.setSorting(QDir::Time); // newest first
  const QFileInfoList entries = d.entryInfoList(QDir::Files);
  for (int i = maxFiles; i < entries.size(); ++i) {
    QFile::remove(entries[i].absoluteFilePath());
  }
}

// Roll the current file to a numbered sibling and reopen fresh.
// File is *already* closed by the caller; we just rename + reopen.
void rotateLocked() {
  if (g_filePath.isEmpty())
    return;
  // Suffix: .1, .2, ... The current file becomes .1; existing .N
  // shifts to .N+1.  Cheap because we're not high-volume.
  for (int i = g_cfg.maxFiles; i > 1; --i) {
    const QString from = QString("%1.%2").arg(g_filePath).arg(i - 1);
    const QString to = QString("%1.%2").arg(g_filePath).arg(i);
    if (QFile::exists(to))
      QFile::remove(to);
    if (QFile::exists(from))
      QFile::rename(from, to);
  }
  if (QFile::exists(g_filePath)) {
    QFile::rename(g_filePath, g_filePath + ".1");
  }
  // Reopen.
  g_file.setFileName(g_filePath);
  g_file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
  g_out.setDevice(&g_file);
}

} // namespace

void init(const Config &cfg) {
  QMutexLocker lk(&g_mtx);
  g_cfg = cfg;
  if (g_cfg.logDir.isEmpty())
    g_cfg.logDir = defaultLogDir();
  QDir().mkpath(g_cfg.logDir);
  g_filePath = dateStampedFile(g_cfg.logDir);

  g_file.setFileName(g_filePath);
  if (!g_file.open(QIODevice::WriteOnly | QIODevice::Append |
                   QIODevice::Text)) {
    // Best-effort: if we can't open, leave g_initialized=false so log()
    // becomes a no-op. We don't have a working logger to report this
    // through, so this is intentionally silent.
    return;
  }
  g_out.setDevice(&g_file);
  g_initialized = true;
  enforceMaxFiles(g_cfg.logDir, g_cfg.maxFiles);

  // Banner so the log has a clear "navigator started" marker.
  g_out << QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)
        << " [GCS] navigator session start\n";
  g_out.flush();
}

void shutdown() {
  QMutexLocker lk(&g_mtx);
  if (!g_initialized)
    return;
  g_out << QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)
        << " [GCS] navigator session end\n";
  g_out.flush();
  g_file.close();
  g_initialized = false;
}

void log(const QString &line) {
  if (!g_cfg.enabled)
    return;
  QMutexLocker lk(&g_mtx);
  if (!g_initialized)
    return;

  // Date-rollover check: if the file's calendar day no longer matches
  // today, rotate by reopening the new dated file. Cheap (timestamp
  // compare) so we can do it per call.
  const QString today = dateStampedFile(g_cfg.logDir);
  if (today != g_filePath) {
    g_file.close();
    g_filePath = today;
    g_file.setFileName(g_filePath);
    g_file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
    g_out.setDevice(&g_file);
  }

  // Size-based rotation. Cheap stat after each write; the GCS log is
  // not high-throughput.
  if (g_file.size() > g_cfg.maxBytes) {
    g_file.close();
    rotateLocked();
    enforceMaxFiles(g_cfg.logDir, g_cfg.maxFiles);
  }

  g_out << QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs) << ' '
        << line;
  if (!line.endsWith('\n'))
    g_out << '\n';
  g_out.flush();
}

QString currentFile() {
  QMutexLocker lk(&g_mtx);
  return g_filePath;
}

} // namespace Logger
