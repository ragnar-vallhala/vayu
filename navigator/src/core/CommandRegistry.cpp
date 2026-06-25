#include "CommandRegistry.h"

#include <QDebug>

CommandRegistry::CommandRegistry(QObject *parent) : QObject(parent) {}

QAction *CommandRegistry::add(const QString &id, const QString &title,
                              const QString &category,
                              const QKeySequence &defaultSeq, CmdContext when,
                              std::function<void()> onTrigger) {
  if (auto it = m_byId.constFind(id); it != m_byId.constEnd()) {
    qWarning() << "CommandRegistry: duplicate command id" << id
               << "- ignoring re-registration";
    return it->action;
  }

  auto *act = new QAction(title, this);
  act->setObjectName(id);  // lets the editor/palette key off the id
  if (!defaultSeq.isEmpty())
    act->setShortcut(defaultSeq);
  if (onTrigger)
    QObject::connect(act, &QAction::triggered, this,
                     [fn = std::move(onTrigger)](bool) { fn(); });

  m_byId.insert(id, Command{id, title, category, defaultSeq, when, act});
  m_order.append(id);
  return act;
}

QAction *CommandRegistry::action(const QString &id) const {
  auto it = m_byId.constFind(id);
  return it == m_byId.constEnd() ? nullptr : it->action;
}

const Command *CommandRegistry::command(const QString &id) const {
  auto it = m_byId.constFind(id);
  return it == m_byId.constEnd() ? nullptr : &it.value();
}

QList<Command> CommandRegistry::all() const {
  QList<Command> out;
  out.reserve(m_order.size());
  for (const QString &id : m_order)
    out.append(m_byId.value(id));
  return out;
}

QStringList CommandRegistry::ids() const { return m_order; }
