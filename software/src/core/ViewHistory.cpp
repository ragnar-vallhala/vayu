#include "ViewHistory.h"

#include <algorithm>

ViewHistory::ViewHistory(QObject *parent) : QObject(parent) {}

void ViewHistory::setDepth(int n) {
  m_depth = std::clamp(n, kMinDepth, kMaxDepth);
  while (m_mru.size() > m_depth)
    m_mru.removeLast();
}

void ViewHistory::visit(int viewId) {
  if (!m_mru.isEmpty() && m_mru.front() == viewId)
    return;  // re-visiting the current view collapses
  m_mru.removeAll(viewId);
  m_mru.prepend(viewId);
  while (m_mru.size() > m_depth)
    m_mru.removeLast();
}

int ViewHistory::previous() const {
  return m_mru.size() >= 2 ? m_mru.at(1) : -1;
}
