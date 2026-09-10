#pragma once

#include "PacketFilterExpr.h"

#include <QSet>
#include <QSortFilterProxyModel>
#include <QString>

/**
 * Filters the packet log without ever rebuilding it. ANDs together: enabled
 * type set, direction, device, a hex/text search, and a Wireshark-style
 * expression. All setters re-filter instantly.
 */
class PacketFilterProxy : public QSortFilterProxyModel {
  Q_OBJECT
public:
  enum Direction { Both, RxOnly, TxOnly };

  explicit PacketFilterProxy(QObject *parent = nullptr);

  void setTypeEnabled(int typeNibble, bool on); // 0xFF = RAW
  void setAllTypes(bool on);
  bool typeEnabled(int typeNibble) const {
    return m_types.contains(typeNibble);
  }

  void setDirection(Direction d);
  void setDeviceFilter(int dev); // -1 = any
  void setSearch(const QString &s);
  bool setExpression(const QString &expr); // false on parse error
  QString expressionError() const { return m_expr.error(); }

protected:
  bool filterAcceptsRow(int row, const QModelIndex &parent) const override;

private:
  QSet<int> m_types; // enabled type nibbles (+0xFF)
  Direction m_dir = Both;
  int m_dev = -1;
  QString m_search;    // normalized lower-case
  QString m_searchHex; // search with spaces stripped (for hex match)
  PacketFilterExpr m_expr;
};
