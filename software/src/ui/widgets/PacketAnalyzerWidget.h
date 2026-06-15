#pragma once

#include <QByteArray>
#include <QCheckBox>
#include <QFile>
#include <QLineEdit>
#include <QMap>
#include <QPushButton>
#include <QTableView>
#include <QTextStream>
#include <QWidget>

class FrequencyRibbon;
class LinkStatsPanel;
class PacketDetailWidget;
class PacketLogModel;
class PacketFilterProxy;
class DroneProtocol;

class PacketAnalyzerWidget : public QWidget {
  Q_OBJECT

public:
  explicit PacketAnalyzerWidget(QWidget *parent = nullptr);
  ~PacketAnalyzerWidget() override = default;

  void setProtocol(DroneProtocol *protocol);
  // Advanced ▸ Packet buffer: cap on rows kept in the analyzer's ring buffer.
  void setPacketCapacity(int rows);

public slots:
  void logRxPacket(const QByteArray &data);
  void logTxPacket(const QByteArray &data);

signals:
  void backToHomeRequested();

protected:
  bool eventFilter(QObject *obj, QEvent *e) override;

private slots:
  void onClearClicked();
  void onSaveClicked();
  void onSelectionChanged();
  void onExpressionEdited();

private:
  void buildUi();
  void resizeColumns();                                  // proportional fill
  void tee(const QString &dir, const QByteArray &data); // stream-to-CSV

  FrequencyRibbon *m_freqRibbon = nullptr;
  LinkStatsPanel *m_linkStats = nullptr;
  QTableView *m_table = nullptr;
  PacketLogModel *m_model = nullptr;
  PacketFilterProxy *m_proxy = nullptr;
  PacketDetailWidget *m_detailView = nullptr;

  QPushButton *m_btnClear = nullptr;
  QPushButton *m_btnStream = nullptr;
  QPushButton *m_btnSave = nullptr;
  QCheckBox *m_chkAutoScroll = nullptr;
  QLineEdit *m_searchEdit = nullptr;
  QLineEdit *m_exprEdit = nullptr;
  QMap<int, QPushButton *> m_typeChips;

  bool m_isStreaming = false;
  QFile m_streamFile;
  QTextStream m_streamOut;
};
