#include <QtTest>

#include "ViewHistory.h"

// Phase-2C MRU logic (FR-UX-21): visit ordering, current-collapse, toggle
// target, depth cap, and depth clamp/truncate.
class TstViewHistory : public QObject {
  Q_OBJECT

private slots:
  void visitOrdersMostRecentFirst();
  void revisitMovesToFront();
  void revisitingCurrentIsNoOp();
  void previousIsSecondEntry();
  void depthCapsHistory();
  void setDepthClampsAndTruncates();
};

void TstViewHistory::visitOrdersMostRecentFirst() {
  ViewHistory h;
  h.visit(1);
  h.visit(2);
  h.visit(3);
  QCOMPARE(h.mru(), (QList<int>{3, 2, 1}));
}

void TstViewHistory::revisitMovesToFront() {
  ViewHistory h;
  h.visit(1);
  h.visit(2);
  h.visit(3);
  h.visit(1); // re-visit an older view
  QCOMPARE(h.mru(), (QList<int>{1, 3, 2}));
}

void TstViewHistory::revisitingCurrentIsNoOp() {
  ViewHistory h;
  h.visit(5);
  h.visit(5);
  h.visit(5);
  QCOMPARE(h.mru(), (QList<int>{5}));
  QCOMPARE(h.previous(), -1);
}

void TstViewHistory::previousIsSecondEntry() {
  ViewHistory h;
  QCOMPARE(h.previous(), -1); // empty
  h.visit(1);
  QCOMPARE(h.previous(), -1); // only one view
  h.visit(2);
  QCOMPARE(h.previous(), 1); // toggle target is the prior view
  h.visit(3);
  QCOMPARE(h.previous(), 2);
}

void TstViewHistory::depthCapsHistory() {
  ViewHistory h;
  h.setDepth(3);
  for (int i = 1; i <= 6; ++i)
    h.visit(i);
  QCOMPARE(h.mru(), (QList<int>{6, 5, 4})); // only the 3 most recent kept
}

void TstViewHistory::setDepthClampsAndTruncates() {
  ViewHistory h;
  QCOMPARE(h.depth(), 5); // default
  h.setDepth(99);
  QCOMPARE(h.depth(), ViewHistory::kMaxDepth);
  h.setDepth(0);
  QCOMPARE(h.depth(), ViewHistory::kMinDepth);

  for (int i = 1; i <= 5; ++i)
    h.visit(i); // mru = {5,4,3,2,1}, depth currently 2
  QCOMPARE(h.mru().size(), ViewHistory::kMinDepth);
  QCOMPARE(h.mru(), (QList<int>{5, 4}));
}

QTEST_MAIN(TstViewHistory)
#include "tst_view_history.moc"
