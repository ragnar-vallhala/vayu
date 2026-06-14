#pragma once

#include <QString>

class QMainWindow;
class QStatusBar;
class QWidget;

// Lightweight toast wrapper. Routes short, actionable feedback to the
// main window's status bar with a typed severity (kind drives colour
// + default timeout). Does not replace the log panel — anything that
// belongs in the scrollback should also be appended there. notify() is
// for "did the action just happen?" feedback the user shouldn't have
// to scroll to find.
//
// Usage:
//   Notify::ok(this, "Calibration started");
//   Notify::warn(this, "Sim falling behind real-time");
//   Notify::error(this, "CRC mismatch on last 50 packets");
//
// The first argument is any QWidget; we walk up to the parent QMainWindow
// to find its status bar. If there isn't one (test harness, dialog-only
// context), notify silently does nothing.
namespace Notify {

enum class Kind {
  Info,   // muted accent
  Ok,     // green
  Warn,   // amber
  Error,  // red
};

// Global on/off (Settings ▸ Alerts ▸ Toast notifications). When disabled,
// send() is a no-op. Defaults to enabled.
void setEnabled(bool on);

// Send a toast. timeout_ms=0 means default per-kind (3000 info/ok,
// 5000 warn, 7000 error). Pass an explicit ms to override.
void send(QWidget* anchor, Kind kind, const QString& text,
          int timeout_ms = 0);

// Sugar.
inline void info (QWidget* a, const QString& t, int ms = 0) { send(a, Kind::Info,  t, ms); }
inline void ok   (QWidget* a, const QString& t, int ms = 0) { send(a, Kind::Ok,    t, ms); }
inline void warn (QWidget* a, const QString& t, int ms = 0) { send(a, Kind::Warn,  t, ms); }
inline void error(QWidget* a, const QString& t, int ms = 0) { send(a, Kind::Error, t, ms); }

}  // namespace Notify
