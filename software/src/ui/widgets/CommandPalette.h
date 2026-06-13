#pragma once

#include <QDialog>

class CommandRegistry;
class QLineEdit;
class QListWidget;

// Fuzzy command runner (Ctrl+Shift+P / FR-UX-20): a centred popup with a search
// box over CommandRegistry::all(). Typing fuzzy-filters and ranks; Up/Down move
// the selection, Enter triggers the selected command's QAction, Esc dismisses.
// Disabled commands (their `when`-context off) are shown greyed and not run.
class CommandPalette : public QDialog {
  Q_OBJECT

public:
  CommandPalette(CommandRegistry *registry, QWidget *parent = nullptr);

  // Case-insensitive subsequence match of `pattern` in `text`. Returns false
  // if `pattern` is not a subsequence; otherwise true and `score` rewards
  // start-of-word and contiguous matches (higher = better). Static + pure so
  // the ranking is unit-testable without a GUI.
  static bool fuzzyMatch(const QString &pattern, const QString &text,
                         int &score);

protected:
  bool eventFilter(QObject *obj, QEvent *event) override;

private:
  void rebuild(const QString &query);
  void runSelected();

  CommandRegistry *m_registry = nullptr;
  QLineEdit *m_input = nullptr;
  QListWidget *m_list = nullptr;
};
