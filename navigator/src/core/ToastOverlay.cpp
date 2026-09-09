#include "ToastOverlay.h"

#include <QEvent>
#include <QFrame>
#include <QGraphicsOpacityEffect>
#include <QHBoxLayout>
#include <QLabel>
#include <QPropertyAnimation>
#include <QTimer>

#include <algorithm>

ToastOverlay::ToastOverlay(QWidget *host) : QWidget(host), m_host(host) {
  // Click-through, no background of its own — only the toast cards paint.
  setAttribute(Qt::WA_TransparentForMouseEvents);
  setAttribute(Qt::WA_NoSystemBackground);
  setGeometry(host->rect());
  host->installEventFilter(this);
  // Start hidden; relayout() shows the overlay only while toasts are present, so
  // an empty overlay never sits over (and risks intercepting) the UI underneath.
  hide();
}

ToastOverlay *ToastOverlay::forHost(QWidget *host) {
  // dynamic_cast over the child list — no Q_OBJECT/qobject_cast needed.
  for (QObject *c : host->children())
    if (auto *o = dynamic_cast<ToastOverlay *>(c))
      return o;
  return new ToastOverlay(host); // parented to host → lives with it
}

void ToastOverlay::addToast(const QColor &accent, const QString &iconText,
                            const QString &text, int timeoutMs) {
  // Coalesce: if an identical toast is still alive, bump its count and restart
  // its dismiss timer instead of stacking a duplicate — bursts of the same
  // message (e.g. repeated blocked tx) collapse into one "… ×N" card.
  for (QWidget *c : m_toasts) {
    if (c->property("toastMsg").toString() == text &&
        !c->property("toastDying").toBool()) {
      const int n = c->property("toastCount").toInt() + 1;
      c->setProperty("toastCount", n);
      if (auto *lbl = c->findChild<QLabel *>())
        lbl->setText(iconText + text + QStringLiteral("  ×%1").arg(n));
      if (auto *t = c->findChild<QTimer *>())
        t->start(); // restart countdown
      c->adjustSize();
      relayout();
      return;
    }
  }

  auto *card = new QFrame(this);
  card->setObjectName(QStringLiteral("Toast"));
  card->setAttribute(Qt::WA_TransparentForMouseEvents);
  card->setStyleSheet(
      QStringLiteral(
          "QFrame#Toast { background: rgba(28,32,40,236); border:1px solid %1; "
          "border-left:4px solid %1; border-radius:7px; }"
          "QLabel { color:#E6E9EF; background:transparent; font-size:12px; }")
          .arg(accent.name()));

  auto *lay = new QHBoxLayout(card);
  lay->setContentsMargins(12, 9, 14, 9);
  auto *lbl = new QLabel(iconText + text, card);
  lbl->setWordWrap(true);
  // The whole card subtree must be click-through, or a toast overlapping a
  // control (e.g. Exit Replay) steals the click — the card alone isn't enough,
  // the label is the deepest hit-test target.
  lbl->setAttribute(Qt::WA_TransparentForMouseEvents);
  lay->addWidget(lbl);

  card->setProperty("toastMsg", text);
  card->setProperty("toastCount", 1);
  card->setFixedWidth(320);
  card->adjustSize();

  auto *fx = new QGraphicsOpacityEffect(card);
  fx->setOpacity(0.0);
  card->setGraphicsEffect(fx);

  m_toasts.prepend(card); // newest first
  // Cap the stack: evict the oldest immediately if we're over the limit.
  while (m_toasts.size() > kMaxToasts) {
    QWidget *old = m_toasts.takeLast();
    old->deleteLater();
  }

  card->show();
  relayout();

  auto *in = new QPropertyAnimation(fx, "opacity", card);
  in->setDuration(160);
  in->setStartValue(0.0);
  in->setEndValue(1.0);
  in->start(QAbstractAnimation::DeleteWhenStopped);

  // Auto-dismiss: a restartable single-shot timer (coalescing restarts it). On
  // fire, mark the card dying, fade out, then drop + re-flow. The timer is a
  // child of the card, so a destroyed overlay/card cancels it.
  auto *timer = new QTimer(card);
  timer->setSingleShot(true);
  timer->setInterval(timeoutMs);
  QObject::connect(timer, &QTimer::timeout, card, [this, card, fx] {
    card->setProperty("toastDying", true);
    auto *out = new QPropertyAnimation(fx, "opacity", card);
    out->setDuration(220);
    out->setStartValue(fx->opacity());
    out->setEndValue(0.0);
    QObject::connect(out, &QPropertyAnimation::finished, card, [this, card] {
      m_toasts.removeAll(card);
      card->deleteLater();
      relayout();
    });
    out->start(QAbstractAnimation::DeleteWhenStopped);
  });
  timer->start();
}

void ToastOverlay::relayout() {
  // No toasts → get the overlay out of the way entirely.
  if (m_toasts.isEmpty()) {
    hide();
    return;
  }
  // The overlay is sized to JUST bound the toast stack and parked in the
  // top-right (below the toolbar) — never covering the whole window, so it can
  // never sit over the bottom replay controls / Exit Replay regardless of how
  // reliably WA_TransparentForMouseEvents passes clicks through on this platform.
  const int rightMargin = 16;
  const int topMargin = 56; // clear the menu/toolbar
  const int gap = 8;

  int w = 0, h = 0;
  for (QWidget *c : m_toasts) {
    c->adjustSize();
    w = std::max(w, c->width());
    h += c->height() + gap;
  }
  if (h > 0)
    h -= gap;

  const int x = std::max(0, m_host->width() - w - rightMargin);
  setGeometry(x, topMargin, w, h);

  int y = 0; // newest first → topmost in the stack
  for (QWidget *c : m_toasts) {
    c->move(w - c->width(), y);
    y += c->height() + gap;
  }
  show();
  raise();
}

bool ToastOverlay::eventFilter(QObject *obj, QEvent *ev) {
  if (obj == m_host &&
      (ev->type() == QEvent::Resize || ev->type() == QEvent::Move ||
       ev->type() == QEvent::Show))
    relayout();
  return QWidget::eventFilter(obj, ev);
}
