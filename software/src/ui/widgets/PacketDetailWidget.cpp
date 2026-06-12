#include "PacketDetailWidget.h"

#include <QFontDatabase>
#include <QHeaderView>
#include <QScrollArea>
#include <QSplitter>
#include <QVBoxLayout>

namespace {
constexpr int kOffRole = Qt::UserRole;
constexpr int kLenRole = Qt::UserRole + 1;
}

PacketDetailWidget::PacketDetailWidget(QWidget *parent) : QWidget(parent) {
  setupUi();
}

void PacketDetailWidget::setupUi() {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  auto *split = new QSplitter(Qt::Horizontal, this);

  m_tree = new QTreeWidget(this);
  m_tree->setColumnCount(2);
  m_tree->setHeaderLabels({"Field", "Value"});
  m_tree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  m_tree->header()->setSectionResizeMode(1, QHeaderView::Stretch);
  m_tree->setStyleSheet(
      "QTreeWidget { background: #2A3347; border: 1px solid #3E4452; color: "
      "#ABB2BF; } QHeaderView::section { background: #3E4452; color: #E8F0FE; "
      "padding: 4px; }");

  m_hex = new HexView(this);
  auto *hexScroll = new QScrollArea(this);
  hexScroll->setWidget(m_hex);
  hexScroll->setWidgetResizable(true);
  hexScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  hexScroll->setStyleSheet("QScrollArea { border: 1px solid #3E4452; }");

  split->addWidget(m_tree);
  split->addWidget(hexScroll);
  split->setStretchFactor(0, 3);
  split->setStretchFactor(1, 2);
  layout->addWidget(split);

  // Field selection -> highlight its bytes in the hex pane.
  connect(m_tree, &QTreeWidget::currentItemChanged, this,
          [this](QTreeWidgetItem *item, QTreeWidgetItem *) {
            if (!item) {
              m_hex->setHighlight(-1, 0);
              return;
            }
            const int off = item->data(0, kOffRole).toInt();
            const int len = item->data(0, kLenRole).toInt();
            m_hex->setHighlight(off, len);
          });
}

void PacketDetailWidget::addFields(QTreeWidgetItem *parent,
                                   const DissectField &f) {
  auto *item = new QTreeWidgetItem(parent, {f.name, f.value});
  item->setData(0, kOffRole, f.off);
  item->setData(0, kLenRole, f.len);
  for (const auto &c : f.children)
    addFields(item, c);
}

void PacketDetailWidget::setData(const QByteArray &data) {
  m_tree->clear();
  m_data = data;
  m_hex->setData(data);
  if (data.isEmpty())
    return;

  const DissectField root = PacketDissector::dissect(data);
  // Render the root's children as top-level rows (Header / Payload / CRC).
  for (const auto &c : root.children) {
    auto *top = new QTreeWidgetItem(m_tree, {c.name, c.value});
    top->setData(0, kOffRole, c.off);
    top->setData(0, kLenRole, c.len);
    for (const auto &cc : c.children)
      addFields(top, cc);
    top->setExpanded(true);
  }
}

void PacketDetailWidget::clear() {
  m_tree->clear();
  m_data.clear();
  m_hex->setData({});
}
