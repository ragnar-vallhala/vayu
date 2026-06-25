#pragma once

#include <QDialog>

class CommandRegistry;
class ShortcutsManager;
class QLineEdit;
class QTableWidget;

// VS Code-style keyboard-shortcuts editor (FR-UX-19): a searchable table of
// every registered command with its effective binding and a "modified" marker
// for overridden rows. Double-click (or the row context menu) rebinds via a
// QKeySequenceEdit capture; conflicts are resolved last-bound-wins by
// ShortcutsManager and surfaced as a toast. Per-row reset/unbind and a global
// Reset All. All edits go through ShortcutsManager, which persists them.
class ShortcutsEditorDialog : public QDialog {
  Q_OBJECT

public:
  ShortcutsEditorDialog(CommandRegistry *registry, ShortcutsManager *manager,
                        QWidget *parent = nullptr);

private:
  void rebuild();
  void applyFilter(const QString &text);
  void rebindRow(int row);
  void resetRow(int row);
  void unbindRow(int row);
  void showRowMenu(const QPoint &pos);
  QString idForRow(int row) const;

  CommandRegistry *m_registry = nullptr;
  ShortcutsManager *m_manager = nullptr;
  QLineEdit *m_search = nullptr;
  QTableWidget *m_table = nullptr;
};
