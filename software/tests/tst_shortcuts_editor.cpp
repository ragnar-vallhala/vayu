#include <QLineEdit>
#include <QTableWidget>
#include <QtTest>

#include "CommandRegistry.h"
#include "ShortcutsEditorDialog.h"
#include "ShortcutsManager.h"

// Phase-2A GUI smoke: the editor lists every command and its search box hides
// non-matching rows. Reaches into the dialog via findChild rather than adding
// test-only accessors. The override/conflict logic itself is covered by
// tst_shortcuts_manager.
class TstShortcutsEditor : public QObject {
  Q_OBJECT

private:
  CommandRegistry *reg = nullptr;
  ShortcutsManager *mgr = nullptr;

  void build() {
    reg = new CommandRegistry(this);
    reg->add("view.home", "Home", "View", QKeySequence("Ctrl+H"),
             CmdContext::Always, {});
    reg->add("view.pa", "Packet Analyzer", "View", QKeySequence("Ctrl+P"),
             CmdContext::Always, {});
    reg->add("log.clear", "Clear Log", "Log", QKeySequence("Ctrl+L"),
             CmdContext::Always, {});
    mgr = new ShortcutsManager(reg, this);
  }

private slots:
  void init() { build(); }
  void cleanup() {
    delete mgr;
    delete reg;
    mgr = nullptr;
    reg = nullptr;
  }

  void listsEveryCommand();
  void searchHidesNonMatchingRows();
};

void TstShortcutsEditor::listsEveryCommand() {
  ShortcutsEditorDialog dlg(reg, mgr);
  auto *table = dlg.findChild<QTableWidget *>();
  QVERIFY(table);
  QCOMPARE(table->rowCount(), 3);
}

void TstShortcutsEditor::searchHidesNonMatchingRows() {
  ShortcutsEditorDialog dlg(reg, mgr);
  auto *table = dlg.findChild<QTableWidget *>();
  auto *search = dlg.findChild<QLineEdit *>();
  QVERIFY(table && search);

  search->setText("Home");
  int visible = 0;
  for (int r = 0; r < table->rowCount(); ++r)
    if (!table->isRowHidden(r))
      ++visible;
  QCOMPARE(visible, 1);

  search->clear();
  visible = 0;
  for (int r = 0; r < table->rowCount(); ++r)
    if (!table->isRowHidden(r))
      ++visible;
  QCOMPARE(visible, 3);
}

QTEST_MAIN(TstShortcutsEditor)
#include "tst_shortcuts_editor.moc"
