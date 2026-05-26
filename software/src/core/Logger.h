#pragma once

#include <QString>

// Persistent text logger for the Navigator GCS (FR-LOG-03).
//
// What this is:
//   - One append-only text file per UTC date under <logDir>/.
//   - Rotation happens when the file crosses `maxBytes`; oldest is
//     dropped once we exceed `maxFiles` total in the directory.
//   - LogPanel::appendLog tees here, so the on-screen scrollback and
//     the persistent file see the same lines.
//
// What this is NOT:
//   - A binary telemetry log. The per-run SITL UART2 bytes already
//     get their own .bin under logs/ via SimulatorWidget; that's a
//     separate thing.
//   - A structured logger. Lines are written verbatim; no levels, no
//     fields. Most callers prefix manually ("[GCS] ...", "[ERROR] ...").
//
// Lifecycle: init() once in main() after QApplication is constructed
// (so QStandardPaths resolves correctly). shutdown() is optional —
// the static QFile flushes on process exit.
namespace Logger {

struct Config {
  QString logDir;            // "" → AppDataLocation/logs
  qint64  maxBytes  = 5 * 1024 * 1024;  // rotate at ~5 MB
  int     maxFiles  = 10;               // keep last 10 files
  bool    enabled   = true;
};

void init(const Config& cfg = {});
void shutdown();

// Append a line. A newline is added if the input doesn't end with one.
// Thread-safe; the underlying QFile is guarded by a QMutex so the GUI
// thread and the SITL UART thread can both call this.
void log(const QString& line);

// Where the active file lives (post-init). Useful for the status bar
// or a "show log file" menu action.
QString currentFile();

}  // namespace Logger
