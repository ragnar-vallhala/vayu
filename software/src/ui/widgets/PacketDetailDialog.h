#pragma once

#include "protocol/PacketDecoder.h"
#include <QByteArray>
#include <QDialog>
#include <QTreeWidget>
#include <QVBoxLayout>

class PacketDetailDialog : public QDialog {
  Q_OBJECT
public:
  explicit PacketDetailDialog(const QByteArray &data,
                              QWidget *parent = nullptr);

private:
  void setupUi();
  void populateData(const QByteArray &data);

  QTreeWidget *m_tree;
};
