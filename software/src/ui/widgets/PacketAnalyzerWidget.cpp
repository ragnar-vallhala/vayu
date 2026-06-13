#include "PacketAnalyzerWidget.h"

#include "FrequencyRibbon.h"
#include "LinkStatsPanel.h"
#include "PacketDetailWidget.h"
#include "PacketFilterProxy.h"
#include "PacketLogModel.h"
#include "core/ui/Buttons.h"

#include <QButtonGroup>
#include <QComboBox>
#include <QDateTime>
#include <QEvent>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QSplitter>
#include <QStyle>
#include <QVBoxLayout>

PacketAnalyzerWidget::PacketAnalyzerWidget(QWidget *parent) : QWidget(parent) {
  buildUi();
}

void PacketAnalyzerWidget::buildUi() {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 0, 8, 8);
  layout->setSpacing(8);

  m_freqRibbon = new FrequencyRibbon(this);
  layout->addWidget(m_freqRibbon);

  // ---- Top control bar ----
  auto *topBar = new QHBoxLayout();
  m_btnBack = new ui::BackButton(this);
  m_btnBack->setText(tr("← Back to Home"));
  topBar->addWidget(m_btnBack);
  topBar->addStretch();
  m_chkAutoScroll = new QCheckBox(tr("Auto-scroll"), this);
  m_chkAutoScroll->setChecked(true);
  topBar->addWidget(m_chkAutoScroll);
  m_btnClear = new ui::GhostButton(tr("Clear"), this);
  topBar->addWidget(m_btnClear);
  m_btnStream = new ui::SuccessButton(tr("Start Streaming"), this);
  topBar->addWidget(m_btnStream);
  m_btnSave = new ui::GhostButton(tr("Save Table…"), this);
  topBar->addWidget(m_btnSave);
  layout->addLayout(topBar);

  // ---- Model / proxy / view ----
  m_model = new PacketLogModel(this);
  m_proxy = new PacketFilterProxy(this);
  m_proxy->setSourceModel(m_model);

  // ---- Filter bar: type chips + direction + device + search ----
  auto *filterBar = new QHBoxLayout();
  filterBar->setSpacing(6);
  filterBar->addWidget(new QLabel(tr("<b>Filter:</b>"), this));

  auto addChip = [&](const QString &label, int type) {
    auto *btn = new QPushButton(label, this);
    btn->setObjectName("FilterPill");
    btn->setCheckable(true);
    btn->setChecked(true);
    btn->setMinimumHeight(24);
    connect(btn, &QPushButton::toggled, this,
            [this, type](bool on) { m_proxy->setTypeEnabled(type, on); });
    filterBar->addWidget(btn);
    m_typeChips[type] = btn;
  };
  addChip("HB", 0x0);
  addChip("IMU", 0x1);
  addChip("IMUΔ", 0x2);
  addChip("Cmd", 0x3);
  addChip("Att", 0x4);
  addChip("RC", 0x5);
  addChip("Status", 0x6);
  addChip("Log", 0x7);
  addChip("Motor", 0x8);
  addChip("Perf", 0x9);
  addChip("Name", 0xA);
  addChip("RAW", 0xFF);

  filterBar->addStretch();

  // Direction: segmented All / RX / TX (mockup sw3) instead of a combo box.
  filterBar->addWidget(new QLabel(tr("Dir"), this));
  auto *dirGroup = new QButtonGroup(this);
  dirGroup->setExclusive(true);
  const char *dirLabels[] = {"All", "RX", "TX"};
  for (int i = 0; i < 3; ++i) {
    auto *b = new QPushButton(dirLabels[i], this);
    b->setObjectName("FilterPill");
    b->setCheckable(true);
    b->setMinimumHeight(24);
    b->setFixedWidth(40);
    if (i == 0) b->setChecked(true);
    dirGroup->addButton(b, i);
    filterBar->addWidget(b);
  }
  connect(dirGroup, &QButtonGroup::idClicked, this, [this](int i) {
    m_proxy->setDirection(static_cast<PacketFilterProxy::Direction>(i));
  });

  auto *devEdit = new QLineEdit(this);
  devEdit->setPlaceholderText(tr("dev"));
  devEdit->setFixedWidth(50);
  devEdit->setToolTip(tr("Filter by device id (blank = any)"));
  connect(devEdit, &QLineEdit::textChanged, this, [this](const QString &s) {
    bool ok = false;
    int d = s.trimmed().toInt(&ok);
    m_proxy->setDeviceFilter(ok ? d : -1);
  });
  filterBar->addWidget(devEdit);

  m_searchEdit = new QLineEdit(this);
  m_searchEdit->setPlaceholderText(tr("search hex or text…"));
  m_searchEdit->setFixedWidth(180);
  connect(m_searchEdit, &QLineEdit::textChanged, this,
          [this](const QString &s) { m_proxy->setSearch(s); });
  filterBar->addWidget(m_searchEdit);
  layout->addLayout(filterBar);

  // ---- Expression bar ----
  auto *exprBar = new QHBoxLayout();
  exprBar->addWidget(new QLabel(tr("<b>Display filter:</b>"), this));
  m_exprEdit = new QLineEdit(this);
  m_exprEdit->setPlaceholderText(
      tr("e.g.  type==attitude && dev==42 && size>50"));
  m_exprEdit->setClearButtonEnabled(true);
  connect(m_exprEdit, &QLineEdit::textChanged, this,
          &PacketAnalyzerWidget::onExpressionEdited);
  exprBar->addWidget(m_exprEdit);
  layout->addLayout(exprBar);

  // ---- Table ----
  m_table = new QTableView(this);
  m_table->setModel(m_proxy);
  m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
  m_table->setSelectionMode(QAbstractItemView::SingleSelection);
  m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_table->setAlternatingRowColors(true);
  m_table->setShowGrid(false);
  m_table->verticalHeader()->setVisible(false);
  m_table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
  m_table->verticalHeader()->setDefaultSectionSize(20);
  auto *hh = m_table->horizontalHeader();
  hh->setSectionResizeMode(QHeaderView::Interactive);
  // Columns are sized by weight to fill the full width (see resizeColumns()),
  // re-applied on every resize via the event filter below.
  hh->setStretchLastSection(false);
  m_table->installEventFilter(this);

  m_detailView = new PacketDetailWidget(this);

  auto *splitter = new QSplitter(Qt::Vertical, this);
  splitter->addWidget(m_table);
  splitter->addWidget(m_detailView);
  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 2);
  layout->addWidget(splitter, 1);

  m_linkStats = new LinkStatsPanel(this);
  layout->addWidget(m_linkStats);

  // ---- Connections ----
  connect(m_btnBack, &QPushButton::clicked, this,
          &PacketAnalyzerWidget::backToHomeRequested);
  connect(m_btnClear, &QPushButton::clicked, this,
          &PacketAnalyzerWidget::onClearClicked);
  connect(m_btnSave, &QPushButton::clicked, this,
          &PacketAnalyzerWidget::onSaveClicked);

  auto setStreamRole = [this](const char *role) {
    m_btnStream->setObjectName(role);
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
      QString fn = QFileDialog::getSaveFileName(
          this, tr("Stream Packets to CSV"), "", tr("CSV Files (*.csv)"));
      if (fn.isEmpty())
        return;
      m_streamFile.setFileName(fn);
      if (m_streamFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        m_streamOut.setDevice(&m_streamFile);
        m_streamOut << "Timestamp,Direction,Size,Hex\n";
        m_isStreaming = true;
        m_btnStream->setText(tr("Stop Streaming"));
        setStreamRole("DangerButton");
      }
    }
  });

  connect(m_table->selectionModel(), &QItemSelectionModel::currentRowChanged,
          this, &PacketAnalyzerWidget::onSelectionChanged);

  // Batched-insert auto-scroll: follow the tail only when enabled.
  connect(m_proxy, &QAbstractItemModel::rowsInserted, this,
          [this](const QModelIndex &, int, int) {
            if (m_chkAutoScroll->isChecked())
              m_table->scrollToBottom();
          });
}

bool PacketAnalyzerWidget::eventFilter(QObject *obj, QEvent *e) {
  if (obj == m_table && e->type() == QEvent::Resize)
    resizeColumns();
  return QWidget::eventFilter(obj, e);
}

void PacketAnalyzerWidget::resizeColumns() {
  if (!m_table)
    return;
  const int w = m_table->viewport()->width();
  if (w <= 0)
    return;
  // Relative weights — every column grows to fill the width in proportion.
  static const double wt[PacketLogModel::ColCount] = {
      /*Time*/ 1.3, /*Dir*/ 0.5, /*Type*/ 1.7, /*Dev*/ 0.5,
      /*Len*/ 0.5, /*CRC*/ 1.1, /*Payload*/ 3.2};
  double sum = 0;
  for (double x : wt)
    sum += x;
  int used = 0;
  for (int c = 0; c < PacketLogModel::ColCount - 1; ++c) {
    const int cw = int(w * wt[c] / sum);
    m_table->setColumnWidth(c, cw);
    used += cw;
  }
  m_table->setColumnWidth(PacketLogModel::ColCount - 1, qMax(80, w - used));
}

void PacketAnalyzerWidget::onExpressionEdited() {
  const QString text = m_exprEdit->text();
  const bool ok = m_proxy->setExpression(text);
  m_exprEdit->setStyleSheet(
      (ok || text.trimmed().isEmpty())
          ? QString()
          : "QLineEdit { border: 1px solid #E06C75; }");
}

void PacketAnalyzerWidget::onSelectionChanged() {
  const QModelIndex cur = m_table->selectionModel()->currentIndex();
  if (!cur.isValid()) {
    m_detailView->clear();
    return;
  }
  const QModelIndex src = m_proxy->mapToSource(cur);
  const PacketEntry *e = m_model->entry(src.row());
  if (e)
    m_detailView->setData(e->raw);
}

void PacketAnalyzerWidget::logRxPacket(const QByteArray &data) {
  if (m_isStreaming)
    tee("RX", data);
  m_model->enqueue(false, data);
}

void PacketAnalyzerWidget::logTxPacket(const QByteArray &data) {
  if (m_isStreaming)
    tee("TX", data);
  m_model->enqueue(true, data);
}

void PacketAnalyzerWidget::tee(const QString &dir, const QByteArray &data) {
  if (!m_isStreaming)
    return;
  m_streamOut << QDateTime::currentDateTime().toString("HH:mm:ss.zzz") << ","
              << dir << "," << data.size() << ",\""
              << QString::fromLatin1(data.toHex(' ').toUpper()) << "\"\n";
  m_streamOut.flush();
}

void PacketAnalyzerWidget::onClearClicked() {
  m_model->clearAll();
  m_detailView->clear();
}

void PacketAnalyzerWidget::onSaveClicked() {
  QString fn = QFileDialog::getSaveFileName(this, tr("Save Packet Log"), "",
                                            tr("CSV Files (*.csv);;All (*)"));
  if (fn.isEmpty())
    return;
  QFile f(fn);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::critical(this, tr("Error"), f.errorString());
    return;
  }
  QTextStream out(&f);
  out << "No,Time,Dir,Type,Dev,Len,Info,Hex\n";
  int written = 0;
  for (int r = 0; r < m_proxy->rowCount(); ++r) {
    const QModelIndex src = m_proxy->mapToSource(m_proxy->index(r, 0));
    const PacketEntry *e = m_model->entry(src.row());
    if (!e)
      continue;
    out << e->no << "," << e->time << "," << (e->tx ? "TX" : "RX") << ","
        << e->typeName << "," << (e->type == 0xFF ? "" : QString::number(e->dev))
        << "," << e->len << ",\"" << e->info << "\",\""
        << QString::fromLatin1(e->raw.toHex(' ').toUpper()) << "\"\n";
    ++written;
  }
  f.close();
  QMessageBox::information(this, tr("Saved"),
                          tr("Wrote %1 packets.").arg(written));
}

void PacketAnalyzerWidget::setProtocol(DroneProtocol *protocol) {
  if (m_freqRibbon)
    m_freqRibbon->setProtocol(protocol);
  if (m_linkStats)
    m_linkStats->setProtocol(protocol);
}
