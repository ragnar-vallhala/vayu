#pragma once

#include <QHash>
#include <QList>
#include <QObject>
#include <QStringList>

#include <functional>

#include "Command.h"

// Central, single source of truth for "what commands exist". Menus, the
// toolbar, the (Phase-2) shortcut editor / palette, and the recent-views
// switcher all read from here. Owns every Command's QAction.
//
// Usage (Phase 1A): create one per MainWindow, then register each command
// and place its QAction wherever it belongs — a QMenu for menu items, or
// QWidget::addAction() on the window for shortcut-only commands:
//
//   QAction *a = cmds->add("view.home", "Home Screen", "View",
//                          QKeySequence("Ctrl+H"), CmdContext::Always,
//                          [this]{ showHome(); });
//   fileMenu->addAction(a);          // menu item + its shortcut
//   ... or window->addAction(a);     // hidden shortcut, no menu entry
class CommandRegistry : public QObject {
  Q_OBJECT

public:
  explicit CommandRegistry(QObject *parent = nullptr);

  // Create, configure, and register a command's QAction (parented to this
  // registry). `onTrigger` is invoked on QAction::triggered. Does NOT add
  // the action to any widget — the caller decides menu vs window-only
  // placement so a command never registers its shortcut twice (which Qt
  // would flag as ambiguous). Returns the QAction for that placement.
  // A duplicate id is a logic error: the existing action is returned and a
  // warning is logged.
  QAction *add(const QString &id, const QString &title, const QString &category,
               const QKeySequence &defaultSeq, CmdContext when,
               std::function<void()> onTrigger);

  QAction *action(const QString &id) const; // nullptr if unknown
  const Command *command(const QString &id) const;
  QList<Command> all() const; // registration order
  QStringList ids() const;    // registration order

private:
  QHash<QString, Command> m_byId;
  QStringList m_order; // preserves registration order for all()/ids()
};
