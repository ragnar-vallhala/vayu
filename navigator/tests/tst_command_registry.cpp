#include <QAction>
#include <QtTest>

#include "CommandRegistry.h"

// Unit tests for the Phase-1A command registry (FR-UX-19): registration,
// id lookup, registration-order preservation, duplicate guarding, trigger
// dispatch, and default-shortcut application.
class TstCommandRegistry : public QObject {
  Q_OBJECT

private slots:
  void registersAndLooksUpById();
  void allPreservesRegistrationOrder();
  void duplicateIdIsIgnored();
  void triggerInvokesCallback();
  void defaultShortcutApplied();
  void unknownIdReturnsNull();
  void displayTitleStripsMnemonics();
};

void TstCommandRegistry::displayTitleStripsMnemonics() {
  QCOMPARE(commandDisplayTitle("&Home Screen"), QStringLiteral("Home Screen"));
  QCOMPARE(commandDisplayTitle("Si&mulator"), QStringLiteral("Simulator"));
  QCOMPARE(commandDisplayTitle("E&xit"), QStringLiteral("Exit"));
  QCOMPARE(commandDisplayTitle("No mnemonic"), QStringLiteral("No mnemonic"));
  // "&&" is a literal ampersand and must survive.
  QCOMPARE(commandDisplayTitle("Search && Replace"),
           QStringLiteral("Search & Replace"));
}

void TstCommandRegistry::registersAndLooksUpById() {
  CommandRegistry reg;
  QAction *a = reg.add("view.home", "Home", "View", QKeySequence("Ctrl+H"),
                       CmdContext::Always, {});
  QVERIFY(a != nullptr);
  QCOMPARE(reg.action("view.home"), a);
  const Command *c = reg.command("view.home");
  QVERIFY(c != nullptr);
  QCOMPARE(c->title, QStringLiteral("Home"));
  QCOMPARE(c->category, QStringLiteral("View"));
  QVERIFY(c->when == CmdContext::Always);
  QCOMPARE(c->action, a);
}

void TstCommandRegistry::allPreservesRegistrationOrder() {
  CommandRegistry reg;
  reg.add("a", "A", "C", {}, CmdContext::Always, {});
  reg.add("b", "B", "C", {}, CmdContext::Always, {});
  reg.add("c", "C", "C", {}, CmdContext::Always, {});
  QCOMPARE(reg.ids(), (QStringList{"a", "b", "c"}));
  const QList<Command> all = reg.all();
  QCOMPARE(all.size(), 3);
  QCOMPARE(all.first().id, QStringLiteral("a"));
  QCOMPARE(all.last().id, QStringLiteral("c"));
}

void TstCommandRegistry::duplicateIdIsIgnored() {
  CommandRegistry reg;
  QAction *a1 = reg.add("dup", "First", "C", {}, CmdContext::Always, {});
  QAction *a2 = reg.add("dup", "Second", "C", {}, CmdContext::Always, {});
  QCOMPARE(a1, a2); // re-registration returns the original action
  QCOMPARE(reg.all().size(), 1);
  QCOMPARE(reg.command("dup")->title, QStringLiteral("First")); // unchanged
}

void TstCommandRegistry::triggerInvokesCallback() {
  CommandRegistry reg;
  int calls = 0;
  QAction *a =
      reg.add("act", "Act", "C", {}, CmdContext::Always, [&] { ++calls; });
  a->trigger();
  a->trigger();
  QCOMPARE(calls, 2);
}

void TstCommandRegistry::defaultShortcutApplied() {
  CommandRegistry reg;
  QAction *a =
      reg.add("s", "S", "C", QKeySequence("Ctrl+K"), CmdContext::Always, {});
  QCOMPARE(a->shortcut(), QKeySequence("Ctrl+K"));
  // An empty default leaves the action with no shortcut.
  QAction *b = reg.add("b", "B", "C", QKeySequence(), CmdContext::Always, {});
  QVERIFY(b->shortcut().isEmpty());
}

void TstCommandRegistry::unknownIdReturnsNull() {
  CommandRegistry reg;
  QVERIFY(reg.action("missing") == nullptr);
  QVERIFY(reg.command("missing") == nullptr);
}

QTEST_MAIN(TstCommandRegistry)
#include "tst_command_registry.moc"
