#include "Notify.h"

#include "Theme.h"
#include "ToastOverlay.h"

#include <QMainWindow>
#include <QWidget>

namespace Notify {

namespace {
// Global toast switch (Settings ▸ Alerts). Toasts are status-bar feedback, so
// suppressing them is purely cosmetic — the log panel still records events.
bool g_enabled = true;

// Walk up the parent chain to the QMainWindow that hosts the toast overlay.
// Returns nullptr if we never hit one — caller handles silently in that case.
QWidget *findHost(QWidget *anchor) {
  for (QWidget *w = anchor; w; w = w->parentWidget()) {
    if (auto *mw = qobject_cast<QMainWindow *>(w))
      return mw;
  }
  return nullptr;
}

int defaultTimeout(Kind k) {
  switch (k) {
  case Kind::Info:
    return 3000;
  case Kind::Ok:
    return 3000;
  case Kind::Warn:
    return 5000;
  case Kind::Error:
    return 7000;
  }
  return 3000;
}

QColor color(Kind k) {
  switch (k) {
  case Kind::Info:
    return Theme::kAccent;
  case Kind::Ok:
    return Theme::kOk;
  case Kind::Warn:
    return Theme::kWarn;
  case Kind::Error:
    return Theme::kDanger;
  }
  return Theme::kAccent;
}

const char *prefix(Kind k) {
  switch (k) {
  case Kind::Info:
    return "ℹ ";
  case Kind::Ok:
    return "✓ ";
  case Kind::Warn:
    return "⚠ ";
  case Kind::Error:
    return "✕ ";
  }
  return "";
}

} // namespace

void setEnabled(bool on) { g_enabled = on; }

void send(QWidget *anchor, Kind kind, const QString &text, int timeout_ms) {
  if (!g_enabled)
    return;
  QWidget *host = findHost(anchor);
  if (!host)
    return;
  const int t = timeout_ms > 0 ? timeout_ms : defaultTimeout(kind);
  ToastOverlay::forHost(host)->addToast(color(kind), QString(prefix(kind)),
                                        text, t);
}

} // namespace Notify
