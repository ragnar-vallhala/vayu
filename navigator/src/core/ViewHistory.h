#pragma once

#include <QList>
#include <QObject>

// Most-recently-used stack of view ids for the recent-views switcher
// (FR-UX-21). visit() is called on every page change; the front is the current
// view, the next entry is the toggle-to-previous target, and the first `depth`
// entries feed the hold-to-cycle overlay. Pure logic — unit-tested headless.
class ViewHistory : public QObject {
  Q_OBJECT

public:
  explicit ViewHistory(QObject *parent = nullptr);

  // Configurable cycle depth (mockup: Settings ▸ Units & Display). Clamped to
  // [kMinDepth, kMaxDepth]; shrinking truncates the kept history.
  static constexpr int kMinDepth = 2;
  static constexpr int kMaxDepth = 9;
  void setDepth(int n);
  int depth() const { return m_depth; }

  // Record a navigation to `viewId`. Re-visiting the current view is a no-op;
  // otherwise the id moves/inserts at the front and the list is capped to
  // depth.
  void visit(int viewId);

  QList<int> mru() const { return m_mru; } // front = most recent
  int count() const { return int(m_mru.size()); }

  // The view a quick toggle should jump to (second entry), or -1 if there is
  // no distinct previous view yet.
  int previous() const;

private:
  QList<int> m_mru; // front = most recent
  int m_depth = 5;
};
