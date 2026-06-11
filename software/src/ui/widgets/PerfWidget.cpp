#include "PerfWidget.h"
#include <QFrame>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QSplitter>
#include <QVBoxLayout>

namespace {
constexpr double kCpuHz = 84e6; // STM32F401 @ 84 MHz, for cycles -> µs

QString cyclesToUs(uint32_t cyc) {
  return QString::number(cyc / (kCpuHz / 1e6), 'f', 1) + " µs";
}

// Roles used to carry a bar percent + colour into BarDelegate.
constexpr int kPctRole = Qt::UserRole;
constexpr int kBarColorRole = Qt::UserRole + 1;
constexpr int kIdRole = Qt::UserRole + 2;
} // namespace

// ===========================================================================
// Sparkline
// ===========================================================================
Sparkline::Sparkline(QWidget *parent) : QWidget(parent) {
  setMinimumHeight(46);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

void Sparkline::setData(const QVector<double> &data, double maxHint) {
  m_data = data;
  m_maxHint = maxHint;
  update();
}

void Sparkline::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  QRectF r = rect().adjusted(1, 3, -1, -3);
  p.fillRect(rect(), QColor(0x1b, 0x20, 0x28));
  if (m_data.size() < 2) {
    p.setPen(QColor(0x66, 0x6c, 0x78));
    p.drawText(rect(), Qt::AlignCenter, "collecting…");
    return;
  }
  double mx = m_maxHint;
  for (double v : m_data)
    mx = qMax(mx, v);
  if (mx <= 0)
    mx = 1;

  const int n = m_data.size();
  auto xAt = [&](int i) {
    return r.left() + r.width() * i / (n - 1);
  };
  auto yAt = [&](double v) {
    return r.bottom() - r.height() * qBound(0.0, v / mx, 1.0);
  };

  QPainterPath line, area;
  line.moveTo(xAt(0), yAt(m_data[0]));
  area.moveTo(xAt(0), r.bottom());
  area.lineTo(xAt(0), yAt(m_data[0]));
  for (int i = 1; i < n; i++) {
    line.lineTo(xAt(i), yAt(m_data[i]));
    area.lineTo(xAt(i), yAt(m_data[i]));
  }
  area.lineTo(xAt(n - 1), r.bottom());
  area.closeSubpath();

  QColor fill = m_color;
  fill.setAlpha(50);
  p.fillPath(area, fill);
  p.setPen(QPen(m_color, 1.6));
  p.drawPath(line);
}

// ===========================================================================
// StatCard
// ===========================================================================
StatCard::StatCard(const QString &title, QWidget *parent) : QWidget(parent) {
  setObjectName("statCard");
  setStyleSheet("#statCard { background:#222934; border:1px solid #313a47;"
                " border-radius:8px; }");
  auto *lay = new QVBoxLayout(this);
  lay->setContentsMargins(12, 8, 12, 8);
  lay->setSpacing(2);

  auto *t = new QLabel(title.toUpper(), this);
  t->setStyleSheet("color:#8b93a1; font-size:10px; font-weight:600;"
                   " letter-spacing:1px;");
  m_value = new QLabel("—", this);
  m_value->setStyleSheet("color:#e8ecf2; font-size:20px; font-weight:700;");
  m_subtitle = new QLabel(" ", this);
  m_subtitle->setStyleSheet("color:#8b93a1; font-size:11px;");

  m_bar = new QWidget(this);
  m_bar->setFixedHeight(4);
  m_bar->setMinimumWidth(60);

  lay->addWidget(t);
  lay->addWidget(m_value);
  lay->addWidget(m_subtitle);
  lay->addWidget(m_bar);
}

void StatCard::setBar(double pct, const QColor &c) {
  m_pct = pct;
  m_barColor = c;
  m_bar->update();
  // Paint via a stylesheet gradient: filled portion then track.
  if (pct < 0) {
    m_bar->setStyleSheet("background:transparent; border-radius:2px;");
    return;
  }
  int p = qBound(0, int(pct), 100);
  m_bar->setStyleSheet(QString("border-radius:2px; background:"
                               "qlineargradient(x1:0,y1:0,x2:1,y2:0,"
                               "stop:0 %1, stop:%2 %1,"
                               "stop:%3 #2c3441, stop:1 #2c3441);")
                           .arg(m_barColor.name())
                           .arg(p / 100.0, 0, 'f', 3)
                           .arg(qMin(1.0, p / 100.0 + 0.001), 0, 'f', 3));
}

// ===========================================================================
// BarDelegate
// ===========================================================================
void BarDelegate::paint(QPainter *p, const QStyleOptionViewItem &opt,
                        const QModelIndex &idx) const {
  QVariant pv = idx.data(kPctRole);
  if (!pv.isValid()) {
    QStyledItemDelegate::paint(p, opt, idx);
    return;
  }
  if (opt.state & QStyle::State_Selected)
    p->fillRect(opt.rect, QColor(0x2d, 0x3a, 0x4d));

  double pct = qBound(0.0, pv.toDouble(), 100.0);
  QColor c = idx.data(kBarColorRole).value<QColor>();
  if (!c.isValid())
    c = QColor(0x4f, 0xc3, 0xf7);

  QRect bar = opt.rect.adjusted(8, 6, -8, -6);
  p->save();
  p->setRenderHint(QPainter::Antialiasing);
  // track
  p->setBrush(QColor(0x2c, 0x34, 0x41));
  p->setPen(Qt::NoPen);
  p->drawRoundedRect(bar, 3, 3);
  // fill
  QRect fill = bar;
  fill.setWidth(int(bar.width() * pct / 100.0));
  QColor fc = c;
  fc.setAlpha(210);
  p->setBrush(fc);
  p->drawRoundedRect(fill, 3, 3);
  // text
  p->setPen(QColor(0xe8, 0xec, 0xf2));
  p->drawText(opt.rect.adjusted(10, 0, -10, 0),
              Qt::AlignVCenter | Qt::AlignLeft, idx.data(Qt::DisplayRole).toString());
  p->restore();
}

// ===========================================================================
// PerfWidget
// ===========================================================================
PerfWidget::PerfWidget(QWidget *parent) : QWidget(parent) {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(10, 8, 10, 8);
  root->setSpacing(8);

  // --- Header --------------------------------------------------------------
  auto *header = new QHBoxLayout();
  auto *title = new QLabel("<span style='font-size:16px;font-weight:700;"
                           "color:#e8ecf2'>Kernel Observability</span>",
                           this);
  m_enabledBadge = new QLabel("", this);
  m_seq = new QLabel("—", this);
  m_seq->setStyleSheet("color:#8b93a1;");
  auto *back = new QPushButton("⟵ Home", this);
  connect(back, &QPushButton::clicked, this, &PerfWidget::backToHomeRequested);
  header->addWidget(title);
  header->addSpacing(8);
  header->addWidget(m_enabledBadge);
  header->addStretch();
  header->addWidget(new QLabel("report", this));
  header->addWidget(m_seq);
  header->addSpacing(8);
  header->addWidget(back);
  root->addLayout(header);

  // --- Overview cards ------------------------------------------------------
  auto *cards = new QHBoxLayout();
  cards->setSpacing(8);
  buildTopCards(cards);
  root->addLayout(cards);

  // --- Main split: lists (left) | detail (right) ---------------------------
  auto *split = new QSplitter(Qt::Horizontal, this);

  auto *leftSplit = new QSplitter(Qt::Vertical, split);

  // Tasks
  auto *taskBox = new QWidget(leftSplit);
  auto *taskLay = new QVBoxLayout(taskBox);
  taskLay->setContentsMargins(0, 0, 0, 0);
  auto *taskHdr = new QLabel("<b>Tasks</b>  <span style='color:#8b93a1'>"
                             "(click a row for details)</span>",
                             taskBox);
  m_taskTable = new QTableWidget(0, 4, taskBox);
  m_taskTable->setHorizontalHeaderLabels({"Task", "State", "CPU", "Stack"});
  m_taskTable->verticalHeader()->setVisible(false);
  m_taskTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_taskTable->setSelectionBehavior(QAbstractItemView::SelectRows);
  m_taskTable->setSelectionMode(QAbstractItemView::SingleSelection);
  m_taskTable->setShowGrid(false);
  m_taskTable->setItemDelegateForColumn(2, new BarDelegate(this));
  m_taskTable->setItemDelegateForColumn(3, new BarDelegate(this));
  m_taskTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  m_taskTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  m_taskTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
  m_taskTable->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
  taskLay->addWidget(taskHdr);
  taskLay->addWidget(m_taskTable);

  // FIFOs
  auto *fifoBox = new QWidget(leftSplit);
  auto *fifoLay = new QVBoxLayout(fifoBox);
  fifoLay->setContentsMargins(0, 0, 0, 0);
  auto *fifoHdr = new QLabel("<b>SPSC FIFOs</b>  <span style='color:#8b93a1'>"
                             "(fill high-water & drops)</span>",
                             fifoBox);
  m_fifoTable = new QTableWidget(0, 3, fifoBox);
  m_fifoTable->setHorizontalHeaderLabels({"FIFO", "Fill", "Drops"});
  m_fifoTable->verticalHeader()->setVisible(false);
  m_fifoTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_fifoTable->setSelectionBehavior(QAbstractItemView::SelectRows);
  m_fifoTable->setSelectionMode(QAbstractItemView::SingleSelection);
  m_fifoTable->setShowGrid(false);
  m_fifoTable->setItemDelegateForColumn(1, new BarDelegate(this));
  m_fifoTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  m_fifoTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  m_fifoTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  fifoLay->addWidget(fifoHdr);
  fifoLay->addWidget(m_fifoTable);

  leftSplit->setStretchFactor(0, 3);
  leftSplit->setStretchFactor(1, 2);

  // Detail panel
  auto *detail = new QFrame(split);
  detail->setObjectName("detailPanel");
  detail->setStyleSheet("#detailPanel{background:#1f2530;border:1px solid "
                        "#313a47;border-radius:8px;}");
  detail->setMinimumWidth(300);
  auto *dlay = new QVBoxLayout(detail);
  dlay->setContentsMargins(14, 12, 14, 12);
  m_detailHint = new QLabel("Select a task or FIFO to inspect.", detail);
  m_detailHint->setStyleSheet("color:#8b93a1;");
  m_detailHint->setWordWrap(true);
  m_detailTitle = new QLabel("", detail);
  m_detailTitle->setStyleSheet("color:#e8ecf2;font-size:15px;font-weight:700;");
  m_detailBody = new QLabel("", detail);
  m_detailBody->setStyleSheet("color:#c8cfda;font-size:12px;");
  m_detailBody->setTextFormat(Qt::RichText);
  m_detailBody->setWordWrap(true);
  m_sparkCaption = new QLabel("", detail);
  m_sparkCaption->setStyleSheet("color:#8b93a1;font-size:10px;font-weight:600;"
                                "letter-spacing:1px;");
  m_detailSpark = new Sparkline(detail);
  dlay->addWidget(m_detailHint);
  dlay->addWidget(m_detailTitle);
  dlay->addWidget(m_detailBody);
  dlay->addStretch();
  dlay->addWidget(m_sparkCaption);
  dlay->addWidget(m_detailSpark);
  m_detailTitle->hide();
  m_detailBody->hide();
  m_sparkCaption->hide();
  m_detailSpark->hide();

  split->setStretchFactor(0, 3);
  split->setStretchFactor(1, 1);
  root->addWidget(split, 1);

  // Selection wiring.
  connect(m_taskTable, &QTableWidget::itemSelectionChanged, this, [this] {
    auto items = m_taskTable->selectedItems();
    if (items.isEmpty())
      return;
    m_selTaskId = items.first()->data(kIdRole).toInt();
    m_selFifoId = -1;
    renderDetail();
  });
  connect(m_fifoTable, &QTableWidget::itemSelectionChanged, this, [this] {
    auto items = m_fifoTable->selectedItems();
    if (items.isEmpty())
      return;
    m_selFifoId = items.first()->data(kIdRole).toInt();
    m_selTaskId = -1;
    renderDetail();
  });
}

void PerfWidget::buildTopCards(QHBoxLayout *row) {
  m_cardLoad = new StatCard("CPU Load", this);
  m_cardHeap = new StatCard("Heap Peak Use", this);
  m_cardSwitch = new StatCard("Ctx Switches", this);
  m_cardSysTick = new StatCard("SysTick max", this);
  m_cardIpc = new StatCard("IPC blocked", this);
  m_cardUptime = new StatCard("Uptime", this);
  for (StatCard *c : {m_cardLoad, m_cardHeap, m_cardSwitch, m_cardSysTick,
                      m_cardIpc, m_cardUptime})
    row->addWidget(c);
}

QString PerfWidget::stateName(uint8_t s) {
  switch (s) {
  case 0: return "READY";
  case 1: return "RUNNING";
  case 2: return "BLOCKED";
  case 3: return "DELAYED";
  default: return QString("0x%1").arg(s, 2, 16, QChar('0'));
  }
}

QColor PerfWidget::stateColor(uint8_t s) {
  switch (s) {
  case 0: return QColor(0x4f, 0xc3, 0xf7); // READY - blue
  case 1: return QColor(0x66, 0xbb, 0x6a); // RUNNING - green
  case 2: return QColor(0xff, 0xa7, 0x26); // BLOCKED - amber
  case 3: return QColor(0x90, 0x97, 0xa3); // DELAYED - grey
  default: return QColor(0xb0, 0xb0, 0xb0);
  }
}

QString PerfWidget::fifoName(uint8_t id) {
  static const char *n[] = {"imu.raw",        "imu.telemetry",
                            "imu.control",    "imu.calib",
                            "imu.calib_telem", "attitude.telemetry",
                            "attitude.control", "rc.telemetry",
                            "rc.control",     "control.telemetry"};
  return id < 10 ? n[id] : QString("fifo#%1").arg(id);
}

QColor PerfWidget::stackColor(int pct) {
  if (pct >= 90) return QColor(0xef, 0x53, 0x50);
  if (pct >= 75) return QColor(0xff, 0xa7, 0x26);
  return QColor(0x66, 0xbb, 0x6a);
}

QColor PerfWidget::fillColor(int pct) {
  if (pct >= 95) return QColor(0xef, 0x53, 0x50);
  if (pct >= 75) return QColor(0xff, 0xa7, 0x26);
  return QColor(0x4f, 0xc3, 0xf7);
}

void PerfWidget::updateReport(const PerfReport &r) {
  m_current = r;
  m_haveCurrent = true;
  m_seq->setText(QString::number(r.seq));
  m_enabledBadge->setText(
      r.enabled
          ? "<span style='background:#1e3a24;color:#66bb6a;padding:2px 8px;"
            "border-radius:8px;font-size:11px'>● perf live</span>"
          : "<span style='background:#3a1e1e;color:#ef5350;padding:2px 8px;"
            "border-radius:8px;font-size:11px'>perf OFF</span>");

  // Per-task CPU% from cycle deltas (32-bit, wrap-safe subtraction).
  m_taskCpu.clear();
  uint32_t dTotal = r.cpuCyclesLo - m_prevCpuTotal;
  for (const PerfTaskRow &t : r.tasks) {
    double pct = 0.0;
    if (m_havePrev && m_taskPrevCyc.contains(t.id) && dTotal > 0) {
      uint32_t dT = t.cycles - m_taskPrevCyc[t.id];
      pct = qBound(0.0, 100.0 * double(dT) / double(dTotal), 100.0);
    }
    m_taskCpu[t.id] = pct;
    // history
    auto &h = m_taskCpuHist[t.id];
    h.push_back(pct);
    if (h.size() > kHistLen)
      h.remove(0, h.size() - kHistLen);
    m_taskPrevCyc[t.id] = t.cycles;
  }
  m_prevCpuTotal = r.cpuCyclesLo;
  m_havePrev = true;

  // FIFO history.
  for (const PerfFifoRow &f : r.fifos) {
    int pct = f.capacity ? (100 * f.peak) / f.capacity : 0;
    auto &fh = m_fifoFillHist[f.id];
    fh.push_back(pct);
    if (fh.size() > kHistLen)
      fh.remove(0, fh.size() - kHistLen);
    auto &dh = m_fifoDropHist[f.id];
    dh.push_back(f.drops);
    if (dh.size() > kHistLen)
      dh.remove(0, dh.size() - kHistLen);
  }

  // --- Overview cards ---
  // CPU load = 1 - idle share, from idle-cycle deltas (wrap-safe).
  static uint32_t prevIdle = 0;
  double idlePct = 100.0;
  double load = -1.0;
  if (m_havePrev && dTotal > 0) {
    uint32_t dIdle = r.idleCyclesLo - prevIdle;
    if (dIdle <= dTotal) {
      idlePct = 100.0 * double(dIdle) / double(dTotal);
      load = 100.0 - idlePct;
    }
  }
  prevIdle = r.idleCyclesLo;
  if (load >= 0) {
    m_cardLoad->setValue(QString::number(load, 'f', 1) + "%");
    m_cardLoad->setSubtitle(QString("idle %1%").arg(idlePct, 0, 'f', 1));
    m_cardLoad->setBar(load, load >= 85   ? QColor(0xef, 0x53, 0x50)
                             : load >= 60 ? QColor(0xff, 0xa7, 0x26)
                                          : QColor(0x66, 0xbb, 0x6a));
  } else {
    m_cardLoad->setValue("…");
    m_cardLoad->setBar(-1, Qt::gray);
  }

  double heapPct =
      r.heapTotalBytes ? 100.0 * double(r.heapPeakBytes) / r.heapTotalBytes
                       : -1.0;
  if (heapPct >= 0) {
    m_cardHeap->setValue(QString("%1%").arg(heapPct, 0, 'f', 1));
    m_cardHeap->setSubtitle(QString("%1 / %2 KB peak · %3 alloc/%4 free/%5 oom")
                                .arg(r.heapPeakBytes / 1024.0, 0, 'f', 1)
                                .arg(r.heapTotalBytes / 1024.0, 0, 'f', 1)
                                .arg(r.heapAllocs)
                                .arg(r.heapFrees)
                                .arg(r.heapOom));
    m_cardHeap->setBar(heapPct, heapPct >= 85   ? QColor(0xef, 0x53, 0x50)
                                : heapPct >= 70 ? QColor(0xff, 0xa7, 0x26)
                                                : QColor(0x66, 0xbb, 0x6a));
  } else {
    // Firmware without heap-total (older wire): fall back to absolute peak.
    m_cardHeap->setValue(QString::number(r.heapPeakBytes / 1024.0, 'f', 1) +
                         " KB");
    m_cardHeap->setSubtitle(QString("%1 alloc · %2 free · %3 oom")
                                .arg(r.heapAllocs)
                                .arg(r.heapFrees)
                                .arg(r.heapOom));
    m_cardHeap->setBar(-1, Qt::gray);
  }

  m_cardSwitch->setValue(QString::number(r.schedSwitches));
  m_cardSwitch->setSubtitle("total context switches");
  m_cardSwitch->setBar(-1, Qt::gray);

  m_cardSysTick->setValue(QString::number(r.systickMaxCyc) + " cyc");
  m_cardSysTick->setSubtitle(QString("last %1 · %2")
                                 .arg(r.systickLastCyc)
                                 .arg(cyclesToUs(r.systickMaxCyc)));
  m_cardSysTick->setBar(-1, Qt::gray);

  double ipcBlkPct =
      r.ipcTakes ? 100.0 * double(r.ipcBlocked) / double(r.ipcTakes) : 0.0;
  m_cardIpc->setValue(QString::number(ipcBlkPct, 'f', 0) + "%");
  m_cardIpc->setSubtitle(QString("%1 of %2 takes blocked")
                             .arg(r.ipcBlocked)
                             .arg(r.ipcTakes));
  m_cardIpc->setBar(ipcBlkPct, QColor(0x4f, 0xc3, 0xf7));

  m_cardUptime->setValue(QString::number(r.uptimeTicks / 1000.0, 'f', 0) + " s");
  m_cardUptime->setSubtitle(QString("%1 ticks").arg(r.uptimeTicks));
  m_cardUptime->setBar(-1, Qt::gray);

  rebuildTasks(r, m_taskCpu);
  rebuildFifos(r);
  renderDetail();
}

void PerfWidget::rebuildTasks(const PerfReport &r,
                              const QHash<int, double> &cpu) {
  m_taskTable->setRowCount(r.tasks.size());
  for (int i = 0; i < r.tasks.size(); i++) {
    const PerfTaskRow &t = r.tasks[i];
    int spct = t.stackSize ? (100 * t.stackPeak) / t.stackSize : 0;
    double cpct = cpu.value(t.id, 0.0);

    // Resolve the name from cache; otherwise ask the FC. Re-ask each report
    // until a reply actually lands (requests/replies can be dropped under
    // telemetry load) — m_answered is set only on receipt, in setTaskName, so
    // a lost request isn't mistaken for an answered one. The unresolved set
    // shrinks as replies arrive, so this converges without a standing burst.
    if (!m_nameCache.contains(t.id) && !m_answered.contains(t.id))
      emit requestTaskName(t.id);
    auto *idItem = new QTableWidgetItem(taskLabel(t.id));
    idItem->setData(kIdRole, t.id);
    idItem->setToolTip(QString("task #%1").arg(t.id));

    auto *stItem = new QTableWidgetItem("● " + stateName(t.state));
    stItem->setForeground(stateColor(t.state));
    stItem->setData(kIdRole, t.id);

    auto *cpuItem = new QTableWidgetItem(QString::number(cpct, 'f', 1) + "%");
    cpuItem->setData(kPctRole, cpct);
    cpuItem->setData(kBarColorRole, QColor(0x4f, 0xc3, 0xf7));
    cpuItem->setData(kIdRole, t.id);

    auto *stkItem = new QTableWidgetItem(
        QString("%1%  (%2/%3)").arg(spct).arg(t.stackPeak).arg(t.stackSize));
    stkItem->setData(kPctRole, spct);
    stkItem->setData(kBarColorRole, stackColor(spct));
    stkItem->setData(kIdRole, t.id);

    m_taskTable->setItem(i, 0, idItem);
    m_taskTable->setItem(i, 1, stItem);
    m_taskTable->setItem(i, 2, cpuItem);
    m_taskTable->setItem(i, 3, stkItem);

    if (t.id == m_selTaskId)
      m_taskTable->selectRow(i);
  }
}

void PerfWidget::rebuildFifos(const PerfReport &r) {
  m_fifoTable->setRowCount(r.fifos.size());
  for (int i = 0; i < r.fifos.size(); i++) {
    const PerfFifoRow &f = r.fifos[i];
    int pct = f.capacity ? (100 * f.peak) / f.capacity : 0;

    auto *nameItem = new QTableWidgetItem(fifoName(f.id));
    nameItem->setData(kIdRole, f.id);

    auto *fillItem = new QTableWidgetItem(
        QString("%1%  (%2/%3)").arg(pct).arg(f.peak).arg(f.capacity));
    fillItem->setData(kPctRole, pct);
    fillItem->setData(kBarColorRole, fillColor(pct));
    fillItem->setData(kIdRole, f.id);

    auto *dropItem = new QTableWidgetItem(
        f.drops == 0xFFFF ? QString("≥65535") : QString::number(f.drops));
    dropItem->setData(kIdRole, f.id);
    if (f.drops > 0)
      dropItem->setForeground(QColor(0xef, 0x53, 0x50));
    else
      dropItem->setForeground(QColor(0x66, 0xbb, 0x6a));

    m_fifoTable->setItem(i, 0, nameItem);
    m_fifoTable->setItem(i, 1, fillItem);
    m_fifoTable->setItem(i, 2, dropItem);

    if (f.id == m_selFifoId)
      m_fifoTable->selectRow(i);
  }
}

QString PerfWidget::taskLabel(int id) const {
  QString nm = m_nameCache.value(id);
  return nm.isEmpty() ? QString("#%1").arg(id) : nm;
}

void PerfWidget::setTaskName(int taskId, const QString &name) {
  m_answered.insert(taskId); // got a reply (even an empty one) — stop asking
  if (name.isEmpty())
    return;                  // unnamed/unknown on the FC; keep "#id"
  if (m_nameCache.value(taskId) == name)
    return;
  m_nameCache[taskId] = name;

  // Update the visible row label in place (no full rebuild needed).
  for (int r = 0; r < m_taskTable->rowCount(); r++) {
    QTableWidgetItem *it = m_taskTable->item(r, 0);
    if (it && it->data(Qt::UserRole + 2).toInt() == taskId) {
      it->setText(name);
      break;
    }
  }
  if (m_selTaskId == taskId)
    renderDetail();
}

void PerfWidget::renderDetail() {
  auto showDetail = [this](bool on) {
    m_detailHint->setVisible(!on);
    m_detailTitle->setVisible(on);
    m_detailBody->setVisible(on);
    m_sparkCaption->setVisible(on);
    m_detailSpark->setVisible(on);
  };

  if (m_selTaskId >= 0 && m_haveCurrent) {
    const PerfTaskRow *t = nullptr;
    for (const auto &row : m_current.tasks)
      if (row.id == m_selTaskId)
        t = &row;
    if (!t) {
      showDetail(false);
      return;
    }
    showDetail(true);
    int spct = t->stackSize ? (100 * t->stackPeak) / t->stackSize : 0;
    QString nm = m_nameCache.value(t->id);
    m_detailTitle->setText(nm.isEmpty()
                               ? QString("Task #%1").arg(t->id)
                               : QString("%1  <span style='color:#8b93a1;"
                                         "font-size:12px'>#%2</span>")
                                     .arg(nm)
                                     .arg(t->id));
    m_detailBody->setText(
        QString("<table cellspacing=6>"
                "<tr><td style='color:#8b93a1'>State</td><td>"
                "<b style='color:%1'>%2</b></td></tr>"
                "<tr><td style='color:#8b93a1'>Priority</td><td>%3</td></tr>"
                "<tr><td style='color:#8b93a1'>CPU (this sec)</td><td>"
                "<b>%4%</b></td></tr>"
                "<tr><td style='color:#8b93a1'>Cycles (lo)</td><td>%5</td></tr>"
                "<tr><td style='color:#8b93a1'>Switch-ins</td><td>%6</td></tr>"
                "<tr><td style='color:#8b93a1'>Max burst</td><td>%7 (%8)</td></tr>"
                "<tr><td style='color:#8b93a1'>Stack peak</td><td>"
                "<b style='color:%9'>%10 / %11 B  (%12%)</b></td></tr>"
                "</table>")
            .arg(stateColor(t->state).name())
            .arg(stateName(t->state))
            .arg(t->priority)
            .arg(m_taskCpu.value(t->id, 0.0), 0, 'f', 1)
            .arg(t->cycles)
            .arg(t->switches)
            .arg(t->maxBurst)
            .arg(cyclesToUs(t->maxBurst))
            .arg(stackColor(spct).name())
            .arg(t->stackPeak)
            .arg(t->stackSize)
            .arg(spct));
    m_sparkCaption->setText("CPU % — LAST 60 REPORTS");
    m_detailSpark->setColor(QColor(0x4f, 0xc3, 0xf7));
    m_detailSpark->setData(m_taskCpuHist.value(t->id), 100.0);
    return;
  }

  if (m_selFifoId >= 0 && m_haveCurrent) {
    const PerfFifoRow *f = nullptr;
    for (const auto &row : m_current.fifos)
      if (row.id == m_selFifoId)
        f = &row;
    if (!f) {
      showDetail(false);
      return;
    }
    showDetail(true);
    int pct = f->capacity ? (100 * f->peak) / f->capacity : 0;
    bool dropping = f->drops > 0;
    m_detailTitle->setText(fifoName(f->id));
    m_detailBody->setText(
        QString("<table cellspacing=6>"
                "<tr><td style='color:#8b93a1'>Peak fill</td><td>"
                "<b style='color:%1'>%2 / %3  (%4%)</b></td></tr>"
                "<tr><td style='color:#8b93a1'>Drops</td><td>"
                "<b style='color:%5'>%6</b></td></tr>"
                "<tr><td style='color:#8b93a1'>Backpressure</td><td>%7</td></tr>"
                "</table>"
                "<p style='color:#8b93a1;font-size:11px'>%8</p>")
            .arg(fillColor(pct).name())
            .arg(f->peak)
            .arg(f->capacity)
            .arg(pct)
            .arg(dropping ? "#ef5350" : "#66bb6a")
            .arg(f->drops == 0xFFFF ? "≥65535 (saturated)"
                                    : QString::number(f->drops))
            .arg(dropping ? "producer outruns consumer" : "keeping up")
            .arg(dropping
                     ? "This ring is shedding items. If it's an OVERWRITE "
                       "stream (telemetry) that's by design; if not, size it up "
                       "or speed the consumer."
                     : "No items lost since boot."));
    m_sparkCaption->setText("FILL % — LAST 60 REPORTS");
    m_detailSpark->setColor(fillColor(pct));
    m_detailSpark->setData(m_fifoFillHist.value(f->id), 100.0);
    return;
  }

  showDetail(false);
}
