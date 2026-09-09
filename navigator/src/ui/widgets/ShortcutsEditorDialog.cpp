#include "ShortcutsEditorDialog.h"

#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QKeySequenceEdit>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

#include "CommandRegistry.h"
#include "Notify.h"
#include "ShortcutsManager.h"

namespace {

// Columns. The id lives in COL_COMMAND's Qt::UserRole.
enum { COL_COMMAND = 0, COL_CATEGORY, COL_SHORTCUT, COL_COUNT };
constexpr int kIdRole = Qt::UserRole;

// Modal capture of a single key chord, seeded with the current binding.
// Returns true on accept; `out` holds the captured (possibly empty) sequence.
bool captureSequence(QWidget *parent, const QString &title,
                     const QKeySequence &seed, QKeySequence *out) {
  QDialog dlg(parent);
  dlg.setWindowTitle(title);
  auto *v = new QVBoxLayout(&dlg);
  v->addWidget(new QLabel(QObject::tr("Press the new shortcut, then OK. "
                                      "Use Clear to unbind."),
                          &dlg));
  auto *edit = new QKeySequenceEdit(seed, &dlg);
  v->addWidget(edit);
  auto *box = new QDialogButtonBox(
      QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
  auto *clearBtn =
      box->addButton(QObject::tr("Clear"), QDialogButtonBox::ResetRole);
  QObject::connect(clearBtn, &QPushButton::clicked, edit,
                   &QKeySequenceEdit::clear);
  QObject::connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
  QObject::connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
  v->addWidget(box);
  if (dlg.exec() != QDialog::Accepted)
    return false;
  // Keep only the first chord — chorded sequences are out of scope (v1).
  const QKeySequence seq = edit->keySequence();
  *out = seq.isEmpty() ? QKeySequence() : QKeySequence(seq[0]);
  return true;
}

} // namespace

ShortcutsEditorDialog::ShortcutsEditorDialog(CommandRegistry *registry,
                                             ShortcutsManager *manager,
                                             QWidget *parent)
    : QDialog(parent), m_registry(registry), m_manager(manager) {
  setWindowTitle(tr("Keyboard Shortcuts"));
  resize(620, 460);

  auto *v = new QVBoxLayout(this);

  m_search = new QLineEdit(this);
  m_search->setPlaceholderText(tr("Search by command, category, or key…"));
  m_search->setClearButtonEnabled(true);
  connect(m_search, &QLineEdit::textChanged, this,
          &ShortcutsEditorDialog::applyFilter);
  v->addWidget(m_search);

  m_table = new QTableWidget(0, COL_COUNT, this);
  m_table->setHorizontalHeaderLabels(
      {tr("Command"), tr("Category"), tr("Shortcut")});
  m_table->horizontalHeader()->setSectionResizeMode(COL_COMMAND,
                                                    QHeaderView::Stretch);
  m_table->horizontalHeader()->setSectionResizeMode(
      COL_CATEGORY, QHeaderView::ResizeToContents);
  m_table->horizontalHeader()->setSectionResizeMode(
      COL_SHORTCUT, QHeaderView::ResizeToContents);
  m_table->verticalHeader()->setVisible(false);
  m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
  m_table->setSelectionMode(QAbstractItemView::SingleSelection);
  m_table->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(m_table, &QTableWidget::customContextMenuRequested, this,
          &ShortcutsEditorDialog::showRowMenu);
  connect(m_table, &QTableWidget::cellDoubleClicked, this,
          [this](int row, int) { rebindRow(row); });
  v->addWidget(m_table);

  auto *buttons = new QDialogButtonBox(this);
  auto *resetAll =
      buttons->addButton(tr("Reset All"), QDialogButtonBox::ResetRole);
  connect(resetAll, &QPushButton::clicked, this, [this] {
    m_manager->resetAll();
    m_manager->save();
    rebuild();
    applyFilter(m_search->text());
  });
  buttons->addButton(QDialogButtonBox::Close);
  connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  v->addWidget(buttons);

  rebuild();
}

QString ShortcutsEditorDialog::idForRow(int row) const {
  QTableWidgetItem *it = m_table->item(row, COL_COMMAND);
  return it ? it->data(kIdRole).toString() : QString();
}

void ShortcutsEditorDialog::rebuild() {
  m_table->setRowCount(0);
  for (const Command &c : m_registry->all()) {
    const int row = m_table->rowCount();
    m_table->insertRow(row);

    auto *cmd = new QTableWidgetItem(commandDisplayTitle(c.title));
    cmd->setData(kIdRole, c.id);
    m_table->setItem(row, COL_COMMAND, cmd);
    m_table->setItem(row, COL_CATEGORY, new QTableWidgetItem(c.category));

    const QKeySequence seq = m_manager->effective(c.id);
    auto *keyItem =
        new QTableWidgetItem(seq.toString(QKeySequence::NativeText));
    // Mark overridden rows so the user sees what diverges from defaults.
    if (m_manager->hasOverride(c.id)) {
      QFont f = keyItem->font();
      f.setBold(true);
      keyItem->setFont(f);
      keyItem->setText(keyItem->text() + tr("  (modified)"));
    }
    m_table->setItem(row, COL_SHORTCUT, keyItem);
  }
}

void ShortcutsEditorDialog::applyFilter(const QString &text) {
  const QString needle = text.trimmed();
  for (int row = 0; row < m_table->rowCount(); ++row) {
    bool match = needle.isEmpty();
    for (int col = 0; col < COL_COUNT && !match; ++col) {
      QTableWidgetItem *it = m_table->item(row, col);
      if (it && it->text().contains(needle, Qt::CaseInsensitive))
        match = true;
    }
    m_table->setRowHidden(row, !match);
  }
}

void ShortcutsEditorDialog::rebindRow(int row) {
  const QString id = idForRow(row);
  if (id.isEmpty())
    return;
  QKeySequence seq;
  if (!captureSequence(
          this, tr("Rebind: %1").arg(m_table->item(row, COL_COMMAND)->text()),
          m_manager->effective(id), &seq))
    return;

  if (!seq.isEmpty()) {
    const QString other = m_manager->conflict(seq, id);
    if (!other.isEmpty()) {
      const Command *oc = m_registry->command(other);
      Notify::warn(this, tr("%1 was unbound — it had %2")
                             .arg(oc ? oc->title : other,
                                  seq.toString(QKeySequence::NativeText)));
    }
  }
  m_manager->setOverride(id, seq);
  m_manager->save();
  rebuild();
  applyFilter(m_search->text());
}

void ShortcutsEditorDialog::resetRow(int row) {
  const QString id = idForRow(row);
  if (id.isEmpty())
    return;
  m_manager->clearOverride(id);
  m_manager->save();
  rebuild();
  applyFilter(m_search->text());
}

void ShortcutsEditorDialog::unbindRow(int row) {
  const QString id = idForRow(row);
  if (id.isEmpty())
    return;
  m_manager->setOverride(id, QKeySequence());
  m_manager->save();
  rebuild();
  applyFilter(m_search->text());
}

void ShortcutsEditorDialog::showRowMenu(const QPoint &pos) {
  const int row = m_table->rowAt(pos.y());
  if (row < 0)
    return;
  QMenu menu(this);
  QAction *rebind = menu.addAction(tr("Rebind…"));
  QAction *reset = menu.addAction(tr("Reset to default"));
  QAction *unbind = menu.addAction(tr("Unbind"));
  QAction *picked = menu.exec(m_table->viewport()->mapToGlobal(pos));
  if (picked == rebind)
    rebindRow(row);
  else if (picked == reset)
    resetRow(row);
  else if (picked == unbind)
    unbindRow(row);
}
