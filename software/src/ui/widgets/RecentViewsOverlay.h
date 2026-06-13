#pragma once

#include <QFrame>
#include <QList>
#include <QPair>
#include <QString>

class QListWidget;

// Firefox/VS Code-style recent-views switcher overlay (FR-UX-21): a centred
// frameless popup listing the MRU views. Driven by a Ctrl+Tab command —
// startCycle() shows it pre-advanced to the previous view; while Ctrl is held,
// the cycle key advances (Shift reverses); releasing Ctrl commits the
// selection (activated), Esc cancels. A quick Ctrl+Tab tap therefore toggles to
// the previous view.
class RecentViewsOverlay : public QFrame {
  Q_OBJECT

public:
  explicit RecentViewsOverlay(QWidget *parent = nullptr);

  // Most-recent-first (viewId, label) pairs, e.g. from ViewHistory::mru().
  void setItems(const QList<QPair<int, QString>> &items);

  // Show centred over the parent, grab the keyboard, and select `startIndex`
  // (1 = the toggle-to-previous target). No-op if fewer than 2 items.
  void startCycle(int startIndex = 1);

signals:
  void activated(int viewId);  // committed selection
  void cancelled();

protected:
  void keyPressEvent(QKeyEvent *event) override;
  void keyReleaseEvent(QKeyEvent *event) override;

private:
  void advance(int delta);
  void commit();
  void dismiss();

  QListWidget *m_list = nullptr;
  QList<QPair<int, QString>> m_items;
};
