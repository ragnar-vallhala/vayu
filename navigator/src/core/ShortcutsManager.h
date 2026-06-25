#pragma once

#include <QHash>
#include <QKeySequence>
#include <QObject>
#include <QString>

class CommandRegistry;

// Owns per-command keybinding overrides on top of the CommandRegistry's
// factory defaults (FR-UX-19). Applies the effective binding to each command's
// QAction (so menu + toolbar + shortcut all update at once), resolves
// conflicts (last-bound wins — the previous holder is unbound), and persists
// overrides to an INI store separate from GcsSettings (which is a fixed blob,
// not a key-value store).
//
// Override semantics: an id PRESENT in the override map has an explicit binding
// — possibly an empty QKeySequence, meaning "explicitly unbound". An id ABSENT
// falls back to the registry default. clearOverride() removes the key (back to
// default); setOverride(id, {}) records an explicit unbind.
class ShortcutsManager : public QObject {
  Q_OBJECT

public:
  explicit ShortcutsManager(CommandRegistry *registry,
                            QObject *parent = nullptr);

  // The binding in effect for a command: its override if present, else the
  // registry default.
  QKeySequence effective(const QString &id) const;
  bool hasOverride(const QString &id) const;

  // Set/clear a command's override and apply it to the QAction immediately.
  // On a conflict, last-bound wins: the command that previously held `seq` is
  // unbound (its override becomes an explicit empty), and displaced() fires.
  void setOverride(const QString &id, const QKeySequence &seq);
  void clearOverride(const QString &id);  // -> back to default
  void resetAll();                        // drop every override

  // The id currently bound to `seq` (other than `exceptId`), or empty string.
  QString conflict(const QKeySequence &seq, const QString &exceptId) const;

  // Re-apply every effective binding onto the registry's QActions.
  void applyAll();

  // Persistence. Paths are explicit so tests can use a temp file; the no-arg
  // overloads use defaultPath().
  void load(const QString &iniPath);
  void save(const QString &iniPath) const;
  void load();
  void save() const;
  static QString defaultPath();

signals:
  void changed(const QString &id);
  // Emitted when setting `id`'s binding displaced `displacedId`.
  void displaced(const QString &displacedId, const QString &id);

private:
  void applyTo(const QString &id);

  CommandRegistry *m_registry = nullptr;
  QHash<QString, QKeySequence> m_overrides;
};
