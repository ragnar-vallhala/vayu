#pragma once

#include <QByteArray>
#include <QCheckBox>
#include <QDateTime>
#include <QFile>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSet>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QWidget>

class PacketDetailWidget;
class FrequencyRibbon;
class PacketAnalyzerWidget : public QWidget {
  Q_OBJECT

public:
  explicit PacketAnalyzerWidget(QWidget *parent = nullptr);
  ~PacketAnalyzerWidget() override = default;

  void setProtocol(class DroneProtocol *protocol);

public slots:
  void logRxPacket(const QByteArray &data);
  void logTxPacket(const QByteArray &data);

signals:
  void backToHomeRequested();

private slots:
  void onClearClicked();
  void onSaveClicked();
  void onBackClicked();
  void onItemClicked(class QTableWidgetItem *item);
  void onFilterToggled(bool checked);
  void reapplyFilters();

private:
  void addRow(const QString &dir, const QByteArray &data);
  void writeToStream(const QString &dir, const QByteArray &data);

  FrequencyRibbon *m_freqRibbon = nullptr;
  QTableWidget *m_table;
  QPushButton *m_btnClear;
  QPushButton *m_btnSave;
  QPushButton *m_btnStream; // New button for toggle
  QPushButton *m_btnBack;
  QCheckBox *m_chkAutoScroll;

  // Track total packets for display
  int m_packetCount = 0;

  // Streaming state
  bool m_isStreaming = false;
  QFile m_streamFile;
  QTextStream m_streamOut;

  PacketDetailWidget *m_detailView = nullptr;
  QMap<int, QPushButton *> m_filterButtons;
  QSet<int> m_disabledTypes;
};
