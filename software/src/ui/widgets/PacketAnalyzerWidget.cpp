#include "PacketAnalyzerWidget.h"

#include "FrequencyRibbon.h"
#include "LinkStatsPanel.h"
#include "PacketDetailWidget.h"
#include "core/ui/Buttons.h"

#include <QFileDialog>
#include <QHeaderView>
#include <QMessageBox>
#include <QScrollBar>
#include <QShowEvent>
#include <QSplitter>
#include <QStyle>
#include <QTextStream>

PacketAnalyzerWidget::PacketAnalyzerWidget(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 0, 8, 8); // Top margin 0 for ribbon
  layout->setSpacing(8);

  m_freqRibbon = new FrequencyRibbon(this);
  layout->addWidget(m_freqRibbon);

  // Top control bar
  auto *topBar = new QHBoxLayout();

  m_btnBack = new ui::BackButton(this);
  m_btnBack->setText(tr("← Back to Home"));
  m_btnBack->setToolTip(tr("Return to home"));
  topBar->addWidget(m_btnBack);

  topBar->addStretch();

  m_chkAutoScroll = new QCheckBox("Auto-scroll", this);
  m_chkAutoScroll->setChecked(true);
  m_chkAutoScroll->setToolTip(tr("Keep the table scrolled to the newest packet"));
  topBar->addWidget(m_chkAutoScroll);

  m_btnClear = new ui::GhostButton(tr("Clear Logs"), this);
  m_btnClear->setToolTip(tr("Drop all rows from the table (ring buffer reset)"));
  topBar->addWidget(m_btnClear);

  // Streaming toggle. Object name flips Success<->Danger on each click
  // via repolish() so the colour matches the action it'll perform.
  m_btnStream = new ui::SuccessButton(tr("Start Streaming"), this);
  m_btnStream->setToolTip(tr("Tee captured packets to a CSV file"));
  topBar->addWidget(m_btnStream);

  m_btnSave = new ui::GhostButton(tr("Save Table…"), this);
  m_btnSave->setToolTip(tr("Export the current table snapshot to CSV"));
  topBar->addWidget(m_btnSave);

  layout->addLayout(topBar);

  // Filter Bar
  auto *filterBar = new QHBoxLayout();
  filterBar->setSpacing(6);
  filterBar->addWidget(new QLabel(" <b>Filter Type:</b> ", this));

  // Filter pills — checkable, accent-fill when active. Styled via the
  // QPushButton#FilterPill selector in dark.qss.
  auto addFilter = [&](const QString &label, int type) {
    auto *btn = new QPushButton(label, this);
    btn->setObjectName("FilterPill");
    btn->setCheckable(true);
    btn->setChecked(false);
    m_disabledTypes.insert(type);
    btn->setProperty("packetType", type);
    btn->setMinimumHeight(24);
    btn->setToolTip(tr("Hide / show %1 packets").arg(label));
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
  // Table chrome (background, gridline, header) handled by global QSS.
  m_detailView = new PacketDetailWidget(this);

  auto *splitter = new QSplitter(Qt::Vertical, this);
  splitter->addWidget(m_table);
  splitter->addWidget(m_detailView);
  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 2);
  layout->addWidget(splitter);

  // Link quality + up/down throughput + byte totals, pinned at the bottom.
  m_linkStats = new LinkStatsPanel(this);
  layout->addWidget(m_linkStats);

  m_masterLog.reserve(500);

  connect(m_btnBack, &QPushButton::clicked, this,
          &PacketAnalyzerWidget::onBackClicked);
  connect(m_btnClear, &QPushButton::clicked, this,
          &PacketAnalyzerWidget::onClearClicked);
  connect(m_btnSave, &QPushButton::clicked, this,
          &PacketAnalyzerWidget::onSaveClicked);
  // Flip the streaming button's role (Success<->Danger) by changing
  // objectName + re-polishing — no inline stylesheet text.
  auto setStreamRole = [this](const char *roleName) {
    m_btnStream->setObjectName(roleName);
    m_btnStream->style()->unpolish(m_btnStream);
    m_btnStream->style()->polish(m_btnStream);
    m_btnStream->update();
  };
  connect(m_btnStream, &QPushButton::clicked, this, [this, setStreamRole]() {
    if (m_isStreaming) {
      m_isStreaming = false;
      m_streamFile.close();
      m_btnStream->setText(tr("Start Streaming"));
      setStreamRole("SuccessButton");
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
        m_btnStream->setText(tr("Stop Streaming"));
        setStreamRole("DangerButton");
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
  if (m_linkStats)
    m_linkStats->addTxBytes(data.size()); // count even when the table is hidden
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
  m_masterLog.append(entry);
  if (m_masterLog.size() > 500) {
    m_masterLog.removeFirst();
  }

  // Skip expensive table updates if the widget is hidden
  if (!isVisible())
    return;

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
  // Iterating from 0 to size-1 as we now use append (0 is oldest)
  for (int i = 0; i < m_masterLog.size(); ++i) {
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

void PacketAnalyzerWidget::showEvent(QShowEvent *event) {
  QWidget::showEvent(event);
  reapplyFilters();
}

void PacketAnalyzerWidget::setProtocol(DroneProtocol *protocol) {
  if (m_freqRibbon) {
    m_freqRibbon->setProtocol(protocol);
  }
  if (m_linkStats) {
    m_linkStats->setProtocol(protocol);
  }
}
