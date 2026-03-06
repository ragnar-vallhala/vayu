#include "PacketAnalyzerWidget.h"
#include <QFileDialog>
#include <QHeaderView>
#include <QMessageBox>
#include <QScrollBar>
#include <QTextStream>

PacketAnalyzerWidget::PacketAnalyzerWidget(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(8);

  // Top control bar
  auto *topBar = new QHBoxLayout();

  m_btnBack = new QPushButton("← Back to Home", this);
  m_btnBack->setStyleSheet(
      "QPushButton { font-weight: bold; padding: 4px 12px; }");
  topBar->addWidget(m_btnBack);

  topBar->addStretch();

  m_chkAutoScroll = new QCheckBox("Auto-scroll", this);
  m_chkAutoScroll->setChecked(true);
  topBar->addWidget(m_chkAutoScroll);

  m_btnClear = new QPushButton("Clear Logs", this);
  topBar->addWidget(m_btnClear);

  m_btnSave = new QPushButton("Save to File...", this);
  topBar->addWidget(m_btnSave);

  layout->addLayout(topBar);

  // Table
  m_table = new QTableWidget(0, 4, this);
  m_table->setHorizontalHeaderLabels(
      {"Timestamp", "Dir", "Size", "Data (Hex)"});
  m_table->horizontalHeader()->setSectionResizeMode(
      0, QHeaderView::ResizeToContents);
  m_table->horizontalHeader()->setSectionResizeMode(
      1, QHeaderView::ResizeToContents);
  m_table->horizontalHeader()->setSectionResizeMode(
      2, QHeaderView::ResizeToContents);
  m_table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
  m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
  m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_table->setAlternatingRowColors(true);
  m_table->setStyleSheet("QTableWidget { background: #1E212B; color: #ABB2BF; "
                         "gridline-color: #2A3347; }"
                         "QTableWidget::item { padding: 4px; }"
                         "QHeaderView::section { background: #2A3347; color: "
                         "#ABB2BF; padding: 4px; border: 1px solid #3E4452; }");
  layout->addWidget(m_table);

  connect(m_btnBack, &QPushButton::clicked, this,
          &PacketAnalyzerWidget::onBackClicked);
  connect(m_btnClear, &QPushButton::clicked, this,
          &PacketAnalyzerWidget::onClearClicked);
  connect(m_btnSave, &QPushButton::clicked, this,
          &PacketAnalyzerWidget::onSaveClicked);
}

void PacketAnalyzerWidget::logRxPacket(const QByteArray &data) {
  addRow("RX", data);
}

void PacketAnalyzerWidget::logTxPacket(const QByteArray &data) {
  addRow("TX", data);
}

void PacketAnalyzerWidget::addRow(const QString &dir, const QByteArray &data) {
  if (data.isEmpty())
    return;

  int row = m_table->rowCount();
  m_table->insertRow(row);

  // Timestamp
  QString ts = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
  auto *itemTs = new QTableWidgetItem(ts);
  m_table->setItem(row, 0, itemTs);

  // Direction
  auto *itemDir = new QTableWidgetItem(dir);
  itemDir->setTextAlignment(Qt::AlignCenter);
  if (dir == "RX") {
    itemDir->setForeground(QBrush(QColor("#98C379"))); // Green
  } else {
    itemDir->setForeground(QBrush(QColor("#61AFEF"))); // Blue
  }
  m_table->setItem(row, 1, itemDir);

  // Size
  auto *itemSize = new QTableWidgetItem(QString::number(data.size()));
  itemSize->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
  m_table->setItem(row, 2, itemSize);

  // Data (Hex)
  QString hexStr = data.toHex(' ').toUpper();
  auto *itemData = new QTableWidgetItem(hexStr);
  itemData->setFont(QFont("Monospace", 10));
  m_table->setItem(row, 3, itemData);

  m_packetCount++;

  // Cap at 10,000 rows to prevent crazy memory usage over long flights
  if (m_table->rowCount() > 10000) {
    m_table->removeRow(0);
    row--;
  }

  if (m_chkAutoScroll->isChecked()) {
    m_table->scrollToBottom();
  }
}

void PacketAnalyzerWidget::onClearClicked() {
  m_table->setRowCount(0);
  m_packetCount = 0;
}

void PacketAnalyzerWidget::onSaveClicked() {
  QString fileName = QFileDialog::getSaveFileName(
      this, "Save Packet Log", "",
      "Log Files (*.log *.txt *.csv);;All Files (*)");
  if (fileName.isEmpty())
    return;

  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::critical(this, "Error",
                          "Failed to open file for writing:\n" +
                              file.errorString());
    return;
  }

  QTextStream out(&file);
  out << "Timestamp,Direction,Size,Data\n";

  for (int r = 0; r < m_table->rowCount(); ++r) {
    QString ts = m_table->item(r, 0)->text();
    QString dir = m_table->item(r, 1)->text();
    QString sz = m_table->item(r, 2)->text();
    QString data = m_table->item(r, 3)->text();
    out << ts << "," << dir << "," << sz << ",\"" << data << "\"\n";
  }

  file.close();
  QMessageBox::information(this, "Success",
                           "Saved " + QString::number(m_table->rowCount()) +
                               " packets to file.");
}

void PacketAnalyzerWidget::onBackClicked() { emit backToHomeRequested(); }
