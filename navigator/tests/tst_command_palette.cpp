#include <QLineEdit>
#include <QListWidget>
#include <QtTest>

#include "CommandPalette.h"
#include "CommandRegistry.h"

// Phase-2B: the pure fuzzy matcher (ranking is unit-tested without a GUI) plus
// a smoke test that the popup filters the registry as you type.
class TstCommandPalette : public QObject {
  Q_OBJECT

private slots:
  void fuzzyMatchesSubsequence();
  void fuzzyRejectsNonSubsequence();
  void fuzzyRanksContiguousWordStartHigher();
  void emptyPatternMatchesEverything();
  void popupFiltersAsTyped();
};

void TstCommandPalette::fuzzyMatchesSubsequence() {
  int s = 0;
  QVERIFY(CommandPalette::fuzzyMatch("hs", "Home Screen", s)); // word starts
  QVERIFY(CommandPalette::fuzzyMatch("clr", "Clear Log", s));
  QVERIFY(CommandPalette::fuzzyMatch("pa", "Packet Analyzer", s));
}

void TstCommandPalette::fuzzyRejectsNonSubsequence() {
  int s = 0;
  QVERIFY(!CommandPalette::fuzzyMatch("zzz", "Home Screen", s));
  QVERIFY(!CommandPalette::fuzzyMatch("hx", "Home", s)); // no 'x' after 'h'
}

void TstCommandPalette::fuzzyRanksContiguousWordStartHigher() {
  int sClear = 0, sControl = 0;
  QVERIFY(CommandPalette::fuzzyMatch("cl", "Clear Log", sClear));
  QVERIFY(CommandPalette::fuzzyMatch("cl", "Control Loop", sControl));
  // "Clear" has c+l contiguous at the start; "Control" spreads c..l out.
  QVERIFY(sClear > sControl);
}

void TstCommandPalette::emptyPatternMatchesEverything() {
  int s = 123;
  QVERIFY(CommandPalette::fuzzyMatch("", "anything", s));
  QCOMPARE(s, 0);
}

void TstCommandPalette::popupFiltersAsTyped() {
  CommandRegistry reg;
  reg.add("view.home", "Home", "View", QKeySequence("Ctrl+H"),
          CmdContext::Always, {});
  reg.add("view.pa", "Packet Analyzer", "View", QKeySequence("Ctrl+P"),
          CmdContext::Always, {});
  reg.add("log.clear", "Clear Log", "Log", QKeySequence("Ctrl+L"),
          CmdContext::Always, {});

  CommandPalette pal(&reg);
  auto *list = pal.findChild<QListWidget *>();
  auto *input = pal.findChild<QLineEdit *>();
  QVERIFY(list && input);
  QCOMPARE(list->count(), 3); // empty query lists all

  input->setText("clear");
  QCOMPARE(list->count(), 1);
  QVERIFY(list->item(0)->text().startsWith("Clear Log"));

  input->setText("");
  QCOMPARE(list->count(), 3);
}

QTEST_MAIN(TstCommandPalette)
#include "tst_command_palette.moc"
