#pragma once

#include <QByteArray>
#include <QCheckBox>
#include <QDateTime>
#include <QFile>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QWidget>

class PacketAnalyzerWidget : public QWidget {
  Q_OBJECT

public:
  explicit PacketAnalyzerWidget(QWidget *parent = nullptr);
  ~PacketAnalyzerWidget() override = default;

public slots:
  void logRxPacket(const QByteArray &data);
  void logTxPacket(const QByteArray &data);

signals:
  void backToHomeRequested();

private slots:
  void onClearClicked();
  void onSaveClicked();
  void onBackClicked();

private:
  void addRow(const QString &dir, const QByteArray &data);

  QTableWidget *m_table;
  QPushButton *m_btnClear;
  QPushButton *m_btnSave;
  QPushButton *m_btnBack;
  QCheckBox *m_chkAutoScroll;

  // Track total packets for display
  int m_packetCount = 0;
};
