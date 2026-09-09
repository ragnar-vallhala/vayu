#include <QAction>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtTest>

#include "CommandRegistry.h"
#include "ShortcutsManager.h"

// Phase-2A: overrides layer on registry defaults, apply to the QAction,
// resolve conflicts (last-bound wins), reset, and persist round-trip.
class TstShortcutsManager : public QObject {
  Q_OBJECT

private:
  CommandRegistry *reg = nullptr;

  void buildRegistry() {
    reg = new CommandRegistry(this);
    reg->add("view.home", "Home", "View", QKeySequence("Ctrl+H"),
             CmdContext::Always, {});
    reg->add("view.pa", "Packet Analyzer", "View", QKeySequence("Ctrl+P"),
             CmdContext::Always, {});
    reg->add("log.clear", "Clear Log", "Log", QKeySequence("Ctrl+L"),
             CmdContext::Always, {});
  }

private slots:
  void init() { buildRegistry(); }
  void cleanup() {
    delete reg;
    reg = nullptr;
  }

  void effectiveDefaultsToRegistry();
  void overrideAppliesToAction();
  void conflictIsDetected();
  void lastBoundWinsDisplacesPrevious();
  void clearRestoresDefault();
  void resetAllDropsOverrides();
  void persistRoundTrip();
};

void TstShortcutsManager::effectiveDefaultsToRegistry() {
  ShortcutsManager m(reg);
  QCOMPARE(m.effective("view.home"), QKeySequence("Ctrl+H"));
  QVERIFY(!m.hasOverride("view.home"));
}

void TstShortcutsManager::overrideAppliesToAction() {
  ShortcutsManager m(reg);
  m.setOverride("view.home", QKeySequence("Ctrl+Shift+H"));
  QCOMPARE(m.effective("view.home"), QKeySequence("Ctrl+Shift+H"));
  QVERIFY(m.hasOverride("view.home"));
  QCOMPARE(reg->action("view.home")->shortcut(), QKeySequence("Ctrl+Shift+H"));
}

void TstShortcutsManager::conflictIsDetected() {
  ShortcutsManager m(reg);
  // Ctrl+L is held by log.clear; binding it elsewhere conflicts with that id.
  QCOMPARE(m.conflict(QKeySequence("Ctrl+L"), "view.pa"), QString("log.clear"));
  // No conflict when excluding the holder itself, or for a free chord.
  QCOMPARE(m.conflict(QKeySequence("Ctrl+L"), "log.clear"), QString());
  QCOMPARE(m.conflict(QKeySequence("Ctrl+J"), "view.pa"), QString());
}

void TstShortcutsManager::lastBoundWinsDisplacesPrevious() {
  ShortcutsManager m(reg);
  QSignalSpy spy(&m, &ShortcutsManager::displaced);
  m.setOverride("view.pa", QKeySequence("Ctrl+L")); // steal from log.clear
  QCOMPARE(m.effective("view.pa"), QKeySequence("Ctrl+L"));
  QVERIFY(m.effective("log.clear").isEmpty()); // displaced -> unbound
  QCOMPARE(reg->action("log.clear")->shortcut(), QKeySequence());
  QCOMPARE(spy.count(), 1);
  QCOMPARE(spy.at(0).at(0).toString(), QString("log.clear"));
  QCOMPARE(spy.at(0).at(1).toString(), QString("view.pa"));
}

void TstShortcutsManager::clearRestoresDefault() {
  ShortcutsManager m(reg);
  m.setOverride("view.home", QKeySequence("Ctrl+Shift+H"));
  m.clearOverride("view.home");
  QVERIFY(!m.hasOverride("view.home"));
  QCOMPARE(m.effective("view.home"), QKeySequence("Ctrl+H"));
  QCOMPARE(reg->action("view.home")->shortcut(), QKeySequence("Ctrl+H"));
}

void TstShortcutsManager::resetAllDropsOverrides() {
  ShortcutsManager m(reg);
  m.setOverride("view.home", QKeySequence("Ctrl+Shift+H"));
  m.setOverride("view.pa", QKeySequence("Ctrl+Shift+P"));
  m.resetAll();
  QVERIFY(!m.hasOverride("view.home"));
  QVERIFY(!m.hasOverride("view.pa"));
  QCOMPARE(m.effective("view.home"), QKeySequence("Ctrl+H"));
  QCOMPARE(m.effective("view.pa"), QKeySequence("Ctrl+P"));
}

void TstShortcutsManager::persistRoundTrip() {
  QTemporaryDir dir;
  const QString path = dir.filePath("shortcuts.ini");
  {
    ShortcutsManager m(reg);
    m.setOverride("view.home", QKeySequence("Ctrl+Shift+H"));
    m.setOverride("log.clear", QKeySequence()); // explicit unbind
    m.save(path);
  }
  // Fresh registry + manager loading the same file reproduces the overrides.
  CommandRegistry reg2;
  reg2.add("view.home", "Home", "View", QKeySequence("Ctrl+H"),
           CmdContext::Always, {});
  reg2.add("log.clear", "Clear Log", "Log", QKeySequence("Ctrl+L"),
           CmdContext::Always, {});
  ShortcutsManager m2(&reg2);
  m2.load(path);
  QVERIFY(m2.hasOverride("view.home"));
  QCOMPARE(m2.effective("view.home"), QKeySequence("Ctrl+Shift+H"));
  QVERIFY(m2.hasOverride("log.clear"));
  QVERIFY(m2.effective("log.clear").isEmpty());
  QCOMPARE(reg2.action("view.home")->shortcut(), QKeySequence("Ctrl+Shift+H"));
}

QTEST_MAIN(TstShortcutsManager)
#include "tst_shortcuts_manager.moc"
