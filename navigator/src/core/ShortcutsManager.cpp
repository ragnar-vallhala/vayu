#include "ShortcutsManager.h"

#include <QAction>
#include <QSettings>
#include <QStandardPaths>

#include "CommandRegistry.h"

ShortcutsManager::ShortcutsManager(CommandRegistry *registry, QObject *parent)
    : QObject(parent), m_registry(registry) {}

QKeySequence ShortcutsManager::effective(const QString &id) const {
  auto it = m_overrides.constFind(id);
  if (it != m_overrides.constEnd())
    return it.value();
  const Command *c = m_registry ? m_registry->command(id) : nullptr;
  return c ? c->defaultSeq : QKeySequence();
}

bool ShortcutsManager::hasOverride(const QString &id) const {
  return m_overrides.contains(id);
}

QString ShortcutsManager::conflict(const QKeySequence &seq,
                                   const QString &exceptId) const {
  if (seq.isEmpty() || !m_registry)
    return QString();
  for (const QString &id : m_registry->ids()) {
    if (id == exceptId)
      continue;
    if (effective(id) == seq)
      return id;
  }
  return QString();
}

void ShortcutsManager::setOverride(const QString &id, const QKeySequence &seq) {
  if (!seq.isEmpty()) {
    // Last-bound wins: unbind whoever currently holds this chord.
    const QString other = conflict(seq, id);
    if (!other.isEmpty()) {
      m_overrides[other] = QKeySequence(); // explicit unbind
      applyTo(other);
      emit displaced(other, id);
      emit changed(other);
    }
  }
  m_overrides[id] = seq;
  applyTo(id);
  emit changed(id);
}

void ShortcutsManager::clearOverride(const QString &id) {
  if (m_overrides.remove(id) > 0) {
    applyTo(id); // re-applies the registry default
    emit changed(id);
  }
}

void ShortcutsManager::resetAll() {
  const QList<QString> ids = m_overrides.keys();
  m_overrides.clear();
  for (const QString &id : ids) {
    applyTo(id);
    emit changed(id);
  }
}

void ShortcutsManager::applyTo(const QString &id) {
  if (!m_registry)
    return;
  if (QAction *a = m_registry->action(id))
    a->setShortcut(effective(id));
}

void ShortcutsManager::applyAll() {
  if (!m_registry)
    return;
  for (const QString &id : m_registry->ids())
    applyTo(id);
}

QString ShortcutsManager::defaultPath() {
  const QString dir =
      QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
  return dir + "/shortcuts.ini";
}

void ShortcutsManager::load(const QString &iniPath) {
  m_overrides.clear();
  QSettings s(iniPath, QSettings::IniFormat);
  s.beginGroup("shortcuts");
  for (const QString &id : s.childKeys())
    m_overrides.insert(id, QKeySequence(s.value(id).toString()));
  s.endGroup();
  applyAll();
}

void ShortcutsManager::save(const QString &iniPath) const {
  QSettings s(iniPath, QSettings::IniFormat);
  s.beginGroup("shortcuts");
  s.remove(""); // clear stale keys, then rewrite the current overrides
  for (auto it = m_overrides.constBegin(); it != m_overrides.constEnd(); ++it)
    s.setValue(it.key(), it.value().toString());
  s.endGroup();
}

void ShortcutsManager::load() { load(defaultPath()); }
void ShortcutsManager::save() const { save(defaultPath()); }
