#pragma once

#include "HexView.h"
#include "protocol/PacketDissector.h"

#include <QByteArray>
#include <QTreeWidget>
#include <QWidget>

/**
 * Wireshark-style per-packet detail: a dissection field tree over a hex pane.
 * Selecting a field highlights its byte range in the hex view.
 */
class PacketDetailWidget : public QWidget {
  Q_OBJECT
public:
  explicit PacketDetailWidget(QWidget *parent = nullptr);
  void setData(const QByteArray &data);
  void clear();

private:
  void setupUi();
  void addFields(QTreeWidgetItem *parent, const DissectField &f);

  QTreeWidget *m_tree = nullptr;
  HexView *m_hex = nullptr;
  QByteArray m_data;
};
