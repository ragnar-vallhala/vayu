#pragma once

#include <QColor>
#include <QString>
#include <QVector>
#include <QWidget>

// ToastOverlay — the in-app toast backend behind Notify.
//
// A transparent, click-through child widget that covers its host window and
// floats short "toast" cards in the bottom-right corner. Each card fades in,
// lives for a timeout, fades out, and the stack re-flows. One overlay per host
// (lazily created via forHost); Notify drives it. No Q_OBJECT needed — it has
// no signals/slots; per-card animations/timers are their own QObjects.
class QResizeEvent;

class ToastOverlay : public QWidget {
public:
  explicit ToastOverlay(QWidget *host);

  // Push a new toast. accent tints the card border; iconText is a short glyph
  // prefix; text is the message; timeoutMs is the visible lifetime.
  void addToast(const QColor &accent, const QString &iconText,
                const QString &text, int timeoutMs);

  // Get-or-create the overlay attached to a host window.
  static ToastOverlay *forHost(QWidget *host);

protected:
  bool eventFilter(QObject *obj, QEvent *ev) override;

private:
  void relayout();

  QWidget *m_host;
  QVector<QWidget *> m_toasts; // newest first
  static constexpr int kMaxToasts = 5;
};
