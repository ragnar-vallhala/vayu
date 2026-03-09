#pragma once

#include "protocol/PacketDecoder.h"
#include <QByteArray>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidget>

class PacketDetailWidget : public QWidget {
  Q_OBJECT
public:
  explicit PacketDetailWidget(QWidget *parent = nullptr);
  void setData(const QByteArray &data);
  void clear();

private:
  void setupUi();

  QTreeWidget *m_tree;
  PacketDecoder m_decoder;
};
