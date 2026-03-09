#include "PacketDetailWidget.h"
#include <QFontDatabase>
#include <QHeaderView>
#include <QLabel>

PacketDetailWidget::PacketDetailWidget(QWidget *parent) : QWidget(parent) {
  setupUi();
}

void PacketDetailWidget::setupUi() {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  m_tree = new QTreeWidget(this);
  m_tree->setColumnCount(2);
  m_tree->setHeaderLabels({"Field", "Value"});
  m_tree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  m_tree->header()->setSectionResizeMode(1, QHeaderView::Stretch);
  m_tree->setStyleSheet("QTreeWidget { background: #2A3347; border: 1px solid "
                        "#3E4452; color: #ABB2BF; }"
                        "QHeaderView::section { background: #3E4452; color: "
                        "#E8F0FE; padding: 4px; }");

  layout->addWidget(m_tree);
}

void PacketDetailWidget::setData(const QByteArray &data) {
  m_tree->clear();
  if (data.isEmpty())
    return;

  DecodedPacket pkt = m_decoder.decode(data);

  // Header Info
  auto *headerItem = new QTreeWidgetItem(m_tree, {"Header", ""});
  headerItem->setExpanded(true);
  new QTreeWidgetItem(headerItem, {"Sync", "0x56"});
  new QTreeWidgetItem(headerItem,
                      {"Protocol Version", QString::number(pkt.version)});
  new QTreeWidgetItem(headerItem,
                      {"Packet Type", PacketDecoder::typeToString(pkt.type)});
  new QTreeWidgetItem(headerItem,
                      {"Payload Length", QString::number(pkt.length)});
  new QTreeWidgetItem(headerItem, {"Device ID", QString::number(pkt.deviceId)});
  new QTreeWidgetItem(headerItem,
                      {"Timestamp", QString("%1 ms").arg(pkt.timestamp)});

  // Payload Info
  auto *payloadItem = new QTreeWidgetItem(m_tree, {"Payload", ""});
  payloadItem->setExpanded(true);

  if (!pkt.valid) {
    new QTreeWidgetItem(payloadItem,
                        {"Error", "Checksum mismatch or malformed packet"});
  } else {
    std::visit(
        [payloadItem](auto &&arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, ImuData>) {
            new QTreeWidgetItem(
                payloadItem,
                {"Acc X", QString::number(arg.acc[0], 'f', 4) + " m/s²"});
            new QTreeWidgetItem(
                payloadItem,
                {"Acc Y", QString::number(arg.acc[1], 'f', 4) + " m/s²"});
            new QTreeWidgetItem(
                payloadItem,
                {"Acc Z", QString::number(arg.acc[2], 'f', 4) + " m/s²"});
            new QTreeWidgetItem(
                payloadItem,
                {"Gyr X", QString::number(arg.gyr[0], 'f', 4) + " °/s"});
            new QTreeWidgetItem(
                payloadItem,
                {"Gyr Y", QString::number(arg.gyr[1], 'f', 4) + " °/s"});
            new QTreeWidgetItem(
                payloadItem,
                {"Gyr Z", QString::number(arg.gyr[2], 'f', 4) + " °/s"});
            new QTreeWidgetItem(
                payloadItem,
                {"Mag X", QString::number(arg.mag[0], 'f', 4) + " µT"});
            new QTreeWidgetItem(
                payloadItem,
                {"Mag Y", QString::number(arg.mag[1], 'f', 4) + " µT"});
            new QTreeWidgetItem(
                payloadItem,
                {"Mag Z", QString::number(arg.mag[2], 'f', 4) + " µT"});
            new QTreeWidgetItem(
                payloadItem,
                {"Temp", QString::number(arg.tempC, 'f', 2) + " °C"});
          } else if constexpr (std::is_same_v<T, QString>) {
            new QTreeWidgetItem(payloadItem, {"Message", arg});
          } else {
            new QTreeWidgetItem(payloadItem,
                                {"Data", "No decoder for this type"});
          }
        },
        pkt.payload);
  }

  // Raw Data
  auto *rawItem = new QTreeWidgetItem(m_tree, {"Raw Data (Hex)", ""});
  QString hex = data.toHex(' ').toUpper();
  auto *hexLabel = new QTreeWidgetItem(rawItem, {hex});
  hexLabel->setFont(0, QFontDatabase::systemFont(QFontDatabase::FixedFont));
  hexLabel->setFirstColumnSpanned(true);
}

void PacketDetailWidget::clear() { m_tree->clear(); }
