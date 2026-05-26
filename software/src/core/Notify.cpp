#include "Notify.h"

#include "Theme.h"

#include <QMainWindow>
#include <QStatusBar>
#include <QWidget>

namespace Notify {

namespace {

// Walk up the parent chain to find the QMainWindow that owns the
// status bar. Returns nullptr if we never hit one — caller handles
// silently in that case.
QStatusBar* findStatusBar(QWidget* anchor) {
  for (QWidget* w = anchor; w; w = w->parentWidget()) {
    if (auto* mw = qobject_cast<QMainWindow*>(w)) return mw->statusBar();
  }
  return nullptr;
}

int defaultTimeout(Kind k) {
  switch (k) {
    case Kind::Info:  return 3000;
    case Kind::Ok:    return 3000;
    case Kind::Warn:  return 5000;
    case Kind::Error: return 7000;
  }
  return 3000;
}

QColor color(Kind k) {
  switch (k) {
    case Kind::Info:  return Theme::kAccent;
    case Kind::Ok:    return Theme::kOk;
    case Kind::Warn:  return Theme::kWarn;
    case Kind::Error: return Theme::kDanger;
  }
  return Theme::kAccent;
}

const char* prefix(Kind k) {
  switch (k) {
    case Kind::Info:  return "ℹ ";
    case Kind::Ok:    return "✓ ";
    case Kind::Warn:  return "⚠ ";
    case Kind::Error: return "✕ ";
  }
  return "";
}

}  // namespace

void send(QWidget* anchor, Kind kind, const QString& text, int timeout_ms) {
  QStatusBar* sb = findStatusBar(anchor);
  if (!sb) return;

  // Repaint the status bar with the kind's accent for the duration
  // of the message. QStatusBar applies its own stylesheet over the
  // global one for ::item style; we re-style just the message area
  // through the background of the bar itself. messageChanged() will
  // restore the default when the timeout expires.
  sb->setStyleSheet(
      QString("QStatusBar { background: %1; color: %2; }")
          .arg(Theme::hex(Theme::kBg), color(kind).name(QColor::HexRgb).toUpper()));
  sb->showMessage(QString(prefix(kind)) + text,
                  timeout_ms > 0 ? timeout_ms : defaultTimeout(kind));

  // Reset to default once the toast clears. Connect a single-shot
  // lambda; Qt::SingleShotConnection ensures we don't pile up handlers.
  QObject::connect(
      sb, &QStatusBar::messageChanged, sb,
      [sb](const QString& msg) {
        if (msg.isEmpty()) {
          sb->setStyleSheet(QString());
        }
      },
      Qt::SingleShotConnection);
}

}  // namespace Notify
