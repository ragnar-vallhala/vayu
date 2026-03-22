#include "PacketAnalyzerWidget.h"
#include "FrequencyRibbon.h"
#include "PacketDetailWidget.h"
#include <QFileDialog>
#include <QHeaderView>
#include <QMessageBox>
#include <QScrollBar>
#include <QSplitter>
#include <QTextStream>

PacketAnalyzerWidget::PacketAnalyzerWidget(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 0, 8, 8); // Top margin 0 for ribbon
  layout->setSpacing(8);

  m_freqRibbon = new FrequencyRibbon(this);
  layout->addWidget(m_freqRibbon);

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

  m_btnStream = new QPushButton("Start Streaming", this);
  m_btnStream->setStyleSheet(
      "QPushButton { background: #3A5F3A; color: #98C379; }");
  topBar->addWidget(m_btnStream);

  m_btnSave = new QPushButton("Save Table...", this);
  topBar->addWidget(m_btnSave);

  layout->addLayout(topBar);

  // Filter Bar
  auto *filterBar = new QHBoxLayout();
  filterBar->setSpacing(6);
  filterBar->addWidget(new QLabel(" <b>Filter Type:</b> ", this));

  auto addFilter = [&](const QString &label, int type) {
    auto *btn = new QPushButton(label, this);
    btn->setCheckable(true);
    btn->setChecked(false);
    m_disabledTypes.insert(type);
    btn->setProperty("packetType", type);
    btn->setMinimumHeight(24);
    btn->setStyleSheet(
        "QPushButton { background: #2A3347; color: #ABB2BF; border: 1px "
        "solid #3E4452; border-radius: 4px; padding: 2px 10px; }"
        "QPushButton:checked { background: #61AFEF; color: #1A1D27; "
        "font-weight: bold; border-color: #61AFEF; }");
    connect(btn, &QPushButton::toggled, this,
            &PacketAnalyzerWidget::onFilterToggled);
    filterBar->addWidget(btn);
    m_filterButtons[type] = btn;
  };

  addFilter("Heartbeat", 0x0);
  addFilter("IMU Full", 0x1);
  addFilter("IMU Comp", 0x2);
  addFilter("Attitude", 0x4);
  addFilter("RC", 0x5);
  addFilter("Status", 0x6);
  addFilter("Log", 0x7);
  addFilter("Motor", 0x8);

  filterBar->addStretch();
  layout->addLayout(filterBar);

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
  m_detailView = new PacketDetailWidget(this);

  auto *splitter = new QSplitter(Qt::Vertical, this);
  splitter->addWidget(m_table);
  splitter->addWidget(m_detailView);
  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 2);
  layout->addWidget(splitter);

  m_masterLog.reserve(500);

  connect(m_btnBack, &QPushButton::clicked, this,
          &PacketAnalyzerWidget::onBackClicked);
  connect(m_btnClear, &QPushButton::clicked, this,
          &PacketAnalyzerWidget::onClearClicked);
  connect(m_btnSave, &QPushButton::clicked, this,
          &PacketAnalyzerWidget::onSaveClicked);
  connect(m_btnStream, &QPushButton::clicked, this, [this]() {
    if (m_isStreaming) {
      m_isStreaming = false;
      m_streamFile.close();
      m_btnStream->setText("Start Streaming");
      m_btnStream->setStyleSheet("QPushButton { background: "
                                 "#3A5F3A; "
                                 "color: #98C379; }");
    } else {
      QString fileName =
          QFileDialog::getSaveFileName(this, "Stream Packets to CSV", "",
                                       "CSV Files (*.csv);;All "
                                       "Files (*)");
      if (fileName.isEmpty())
        return;

      m_streamFile.setFileName(fileName);
      if (m_streamFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        m_streamOut.setDevice(&m_streamFile);
        m_streamOut << "Timestamp,Direction,Size,"
                       "Data\n";
        m_isStreaming = true;
        m_btnStream->setText("Stop Streaming");
        m_btnStream->setStyleSheet("QPushButton { background: "
                                   "#5A3A3A; "
                                   "color: #E06C75; }");
      }
    }
  });

  connect(m_table, &QTableWidget::itemClicked, this,
          &PacketAnalyzerWidget::onItemClicked);
  connect(m_table, &QTableWidget::itemSelectionChanged, this, [this]() {
    auto items = m_table->selectedItems();
    if (items.isEmpty()) {
      m_detailView->clear();
    } else {
      onItemClicked(items.first());
    }
  });
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

  // Store in master log
  PacketEntry entry;
  entry.timestamp = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
  entry.direction = dir;
  entry.data = data;
  m_masterLog.prepend(entry);
  if (m_masterLog.size() > 500) {
    m_masterLog.removeLast();
  }

  // Filter check
  int type = -1;
  if (data.size() >= 2) {
    type = (static_cast<uint8_t>(data[1]) >> 4) & 0x0F;
  }

  if (m_disabledTypes.contains(type) || type == -1) {
    return;
  }

  int row = m_table->rowCount();
  m_table->insertRow(row);

  // Timestamp
  auto *itemTs = new QTableWidgetItem(entry.timestamp);
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
  itemData->setData(Qt::UserRole, data); // Store raw data for decoder
  m_table->setItem(row, 3, itemData);

  m_packetCount++;

  if (m_isStreaming) {
    writeToStream(dir, data);
  }

  // Cap at 500 rows
  while (m_table->rowCount() > 500) {
    m_table->removeRow(0);
  }

  if (m_chkAutoScroll->isChecked()) {
    m_table->scrollToBottom();
  }
}

void PacketAnalyzerWidget::writeToStream(const QString &dir,
                                         const QByteArray &data) {
  if (!m_isStreaming)
    return;
  QString ts = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
  m_streamOut << ts << "," << dir << "," << data.size() << ",\""
              << data.toHex(' ').toUpper() << "\"\n";
  // Flush occasionally or every time? At 100Hz every time might be slow, but
  // safe.
  m_streamOut.flush();
}

void PacketAnalyzerWidget::onClearClicked() {
  m_masterLog.clear();
  m_table->setRowCount(0);
  m_packetCount = 0;
  m_detailView->clear();
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

void PacketAnalyzerWidget::onItemClicked(QTableWidgetItem *item) {
  if (!item)
    return;
  int row = item->row();
  QTableWidgetItem *dataItem = m_table->item(row, 3);
  if (!dataItem)
    return;

  QByteArray rawData = dataItem->data(Qt::UserRole).toByteArray();
  m_detailView->setData(rawData);
}

void PacketAnalyzerWidget::onFilterToggled(bool checked) {
  auto *btn = qobject_cast<QPushButton *>(sender());
  if (!btn)
    return;
  int type = btn->property("packetType").toInt();
  if (checked) {
    m_disabledTypes.remove(type);
  } else {
    m_disabledTypes.insert(type);
  }
  reapplyFilters();
}

void PacketAnalyzerWidget::reapplyFilters() {
  m_table->setRowCount(0);
  // Loop backwards through masterLog to show oldest at top or newest at bottom
  // Actually row 0 is top. If we want newest at bottom (auto-scroll feel):
  for (int i = m_masterLog.size() - 1; i >= 0; --i) {
    const auto &entry = m_masterLog[i];
    int type = -1;
    if (entry.data.size() >= 2) {
      type = (static_cast<uint8_t>(entry.data[1]) >> 4) & 0x0F;
    }

    if (!m_disabledTypes.contains(type) && type != -1) {
      int row = m_table->rowCount();
      m_table->insertRow(row);

      m_table->setItem(row, 0, new QTableWidgetItem(entry.timestamp));

      auto *itemDir = new QTableWidgetItem(entry.direction);
      itemDir->setTextAlignment(Qt::AlignCenter);
      itemDir->setForeground(
          QBrush(QColor(entry.direction == "RX" ? "#98C379" : "#61AFEF")));
      m_table->setItem(row, 1, itemDir);

      auto *itemSize = new QTableWidgetItem(QString::number(entry.data.size()));
      itemSize->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
      m_table->setItem(row, 2, itemSize);

      auto *itemData = new QTableWidgetItem(entry.data.toHex(' ').toUpper());
      itemData->setFont(QFont("Monospace", 10));
      itemData->setData(Qt::UserRole, entry.data);
      m_table->setItem(row, 3, itemData);
    }
  }

  if (m_chkAutoScroll->isChecked()) {
    m_table->scrollToBottom();
  }
}

void PacketAnalyzerWidget::setProtocol(DroneProtocol *protocol) {
  if (m_freqRibbon) {
    m_freqRibbon->setProtocol(protocol);
  }
}
