#include "RecentViewsOverlay.h"

#include <QApplication>
#include <QKeyEvent>
#include <QLabel>
#include <QListWidget>
#include <QVBoxLayout>

namespace {
constexpr int kIdRole = Qt::UserRole;
}

RecentViewsOverlay::RecentViewsOverlay(QWidget *parent)
    : QFrame(parent, Qt::Popup) {
  setFrameShape(QFrame::StyledPanel);
  setObjectName("RecentViewsOverlay");
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(12, 12, 12, 12);
  auto *title = new QLabel(tr("Recent views"), this);
  QFont f = title->font();
  f.setBold(true);
  title->setFont(f);
  v->addWidget(title);
  m_list = new QListWidget(this);
  m_list->setFocusPolicy(Qt::NoFocus);  // keystrokes are handled here
  v->addWidget(m_list);
  resize(320, 240);
}

void RecentViewsOverlay::setItems(const QList<QPair<int, QString>> &items) {
  m_items = items;
  m_list->clear();
  for (const auto &it : items) {
    auto *row = new QListWidgetItem(it.second, m_list);
    row->setData(kIdRole, it.first);
  }
}

void RecentViewsOverlay::startCycle(int startIndex) {
  if (m_items.size() < 2)
    return;  // nothing meaningful to switch between
  if (QWidget *p = parentWidget())
    move(p->geometry().center() - QPoint(width() / 2, height() / 2));
  const int idx = qBound(0, startIndex, int(m_items.size()) - 1);
  m_list->setCurrentRow(idx);
  show();
  raise();
  grabKeyboard();
}

void RecentViewsOverlay::advance(int delta) {
  const int n = m_list->count();
  if (n == 0)
    return;
  int row = (m_list->currentRow() + delta % n + n) % n;
  m_list->setCurrentRow(row);
}

void RecentViewsOverlay::keyPressEvent(QKeyEvent *event) {
  switch (event->key()) {
    case Qt::Key_Tab:
    case Qt::Key_Backtab:
    case Qt::Key_QuoteLeft:  // backtick — the mockup's binding
    case Qt::Key_Down:
    case Qt::Key_Up: {
      const bool reverse = (event->key() == Qt::Key_Backtab) ||
                           (event->key() == Qt::Key_Up) ||
                           (event->modifiers() & Qt::ShiftModifier);
      advance(reverse ? -1 : +1);
      event->accept();
      return;
    }
    case Qt::Key_Escape:
      dismiss();
      emit cancelled();
      event->accept();
      return;
    case Qt::Key_Return:
    case Qt::Key_Enter:
      commit();
      event->accept();
      return;
    default:
      break;
  }
  QFrame::keyPressEvent(event);
}

void RecentViewsOverlay::keyReleaseEvent(QKeyEvent *event) {
  // Releasing Ctrl commits the highlighted view (the hold-to-cycle gesture).
  if (event->key() == Qt::Key_Control ||
      !(QApplication::keyboardModifiers() & Qt::ControlModifier)) {
    commit();
    event->accept();
    return;
  }
  QFrame::keyReleaseEvent(event);
}

void RecentViewsOverlay::commit() {
  QListWidgetItem *item = m_list->currentItem();
  const int id = item ? item->data(kIdRole).toInt() : -1;
  dismiss();
  if (id >= 0)
    emit activated(id);
}

void RecentViewsOverlay::dismiss() {
  releaseKeyboard();
  hide();
}
