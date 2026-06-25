#include "SettingsWidget.h"

#include "core/Notify.h"
#include "core/Theme.h"
#include "core/ui/Icons.h"

#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QSerialPortInfo>
#include <QStackedWidget>
#include <QStyle>
#include <QStyledItemDelegate>
#include <QTableWidget>
#include <QVBoxLayout>

// The Settings page mirrors the mockup (docs/ui-mockup): a left nav of seven
// panes. Wired controls stage their edits — changing one marks its row yellow
// (dirty) and enables Apply, which commits everything to disk in one shot.
// Controls the app can't yet act on are DISABLED with a red "coming soon" tag.
namespace {
const char *kSoon = "Coming soon — not yet wired to a backend";

constexpr int kNavIconPx = 30;

// Nav-rail icon: muted when idle, accent-coloured when the row is selected
// (QIcon::Selected/Active modes), so selection recolours the glyph rather than
// drawing a highlight bar behind it.
QIcon navIcon(ui::Icon id) {
  QIcon ic;
  ic.addPixmap(ui::svgPixmap(id, Theme::kTextMuted, kNavIconPx), QIcon::Normal);
  const QPixmap sel = ui::svgPixmap(id, Theme::kAccent, kNavIconPx);
  ic.addPixmap(sel, QIcon::Selected);
  ic.addPixmap(sel, QIcon::Active);
  return ic;
}

// Paints the nav glyph centred both ways (IconMode top-aligns it; ListMode
// left-aligns it). Selection just swaps to the accent pixmap — no highlight bar;
// hover gets a subtle rounded background.
class NavDelegate : public QStyledItemDelegate {
public:
  using QStyledItemDelegate::QStyledItemDelegate;

  QSize sizeHint(const QStyleOptionViewItem &, const QModelIndex &) const override {
    return QSize(0, 54);  // row taller than the icon → even vertical gaps
  }

  void paint(QPainter *p, const QStyleOptionViewItem &opt,
             const QModelIndex &idx) const override {
    p->save();
    p->setRenderHint(QPainter::Antialiasing, true);
    const bool selected = opt.state & QStyle::State_Selected;
    const bool hover = opt.state & QStyle::State_MouseOver;
    const QRect r = opt.rect;
    if (hover && !selected) {
      p->setPen(Qt::NoPen);
      p->setBrush(QColor(255, 255, 255, 16));
      p->drawRoundedRect(r.adjusted(8, 6, -8, -6), 8, 8);
    }
    const QIcon ic = qvariant_cast<QIcon>(idx.data(Qt::DecorationRole));
    const QSize isz(kNavIconPx, kNavIconPx);
    const QPixmap pm =
        ic.pixmap(isz, selected ? QIcon::Selected : QIcon::Normal);
    p->drawPixmap(QPoint(r.center().x() - isz.width() / 2,
                         r.center().y() - isz.height() / 2),
                  pm);
    p->restore();
  }
};

QString rowLabelStyle() { return QStringLiteral("font-weight:600;"); }

QLabel *paneHeader(const QString &title) {
  auto *h = new QLabel(title);
  h->setStyleSheet(QString("font-size:14px; font-weight:700; "
                           "margin-bottom:10px; color:%1;")
                       .arg(Theme::hex(Theme::kAccent)));
  return h;
}

// A name/description ── control row. Returns the container and its name label so
// callers can mark it dirty.
struct Row { QWidget *w; QLabel *name; };
Row makeRow(const QString &name, const QString &desc, QWidget *ctl) {
  auto *w = new QWidget;
  auto *h = new QHBoxLayout(w);
  h->setContentsMargins(0, 5, 0, 5);
  auto *left = new QVBoxLayout;
  left->setSpacing(1);
  auto *n = new QLabel(name);
  n->setStyleSheet(rowLabelStyle());
  left->addWidget(n);
  if (!desc.isEmpty()) {
    auto *d = new QLabel(desc);
    d->setStyleSheet(
        QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextMuted)));
    d->setWordWrap(true);
    left->addWidget(d);
  }
  h->addLayout(left, 1);
  // AlignVCenter so the control takes its own sizeHint height instead of being
  // stretched to fill the row (a QStackedWidget control otherwise balloons).
  h->addWidget(ctl, 0, Qt::AlignRight | Qt::AlignVCenter);
  return {w, n};
}

// Red "coming soon" tag, so placeholders stand out from wired settings.
QString soonTag() {
  return QStringLiteral(
             "<span style=\"color:%1; font-weight:700;\">coming soon</span>")
      .arg(Theme::hex(Theme::kDanger));
}

// Placeholder row: disables the control, tags it "coming soon" in red.
QWidget *soonRow(const QString &name, const QString &desc, QWidget *ctl) {
  ctl->setEnabled(false);
  ctl->setToolTip(kSoon);
  const QString d = desc.isEmpty() ? soonTag() : desc + " · " + soonTag();
  return makeRow(name, d, ctl).w;
}

QComboBox *combo(std::initializer_list<QString> items) {
  auto *c = new QComboBox;
  for (const auto &i : items) c->addItem(i);
  return c;
}
QCheckBox *check(bool on) {
  auto *c = new QCheckBox;
  c->setChecked(on);
  return c;
}
QSpinBox *spin(int lo, int hi, int val) {
  auto *s = new QSpinBox;
  s->setRange(lo, hi);
  s->setValue(val);
  return s;
}
QDoubleSpinBox *dspin(double lo, double hi, double val, double step) {
  auto *s = new QDoubleSpinBox;
  s->setRange(lo, hi);
  s->setValue(val);
  s->setSingleStep(step);
  return s;
}
}  // namespace

SettingsWidget::SettingsWidget(QWidget *parent) : QWidget(parent) {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(20, 16, 20, 16);
  root->setSpacing(12);

  auto *title = new QLabel("<h2>Settings</h2>", this);
  title->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kAccent)));
  root->addWidget(title);

  // Apply button + dirty hint live at the bottom; create the button up front so
  // the dirty-tracking helpers below can enable it.
  m_applyBtn = new QPushButton(tr("Apply"), this);
  m_applyBtn->setEnabled(false);

  // track(ctl, label): edits to ctl mark its row dirty (staged, not applied).
  auto track = [this](QWidget *ctl, QLabel *lbl) {
    if (auto *sb = qobject_cast<QSpinBox *>(ctl))
      connect(sb, QOverload<int>::of(&QSpinBox::valueChanged), this,
              [this, lbl] { markDirty(lbl); });
    else if (auto *db = qobject_cast<QDoubleSpinBox *>(ctl))
      connect(db, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
              [this, lbl] { markDirty(lbl); });
    else if (auto *cb = qobject_cast<QComboBox *>(ctl))
      connect(cb, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
              [this, lbl] { markDirty(lbl); });
    else if (auto *ck = qobject_cast<QCheckBox *>(ctl))
      connect(ck, &QCheckBox::toggled, this, [this, lbl] { markDirty(lbl); });
  };

  auto *split = new QHBoxLayout();
  split->setSpacing(0);
  // Icon-only rail (Mission-Planner style): large centred glyphs, the pane name
  // on hover, and selection that recolours the icon instead of a highlight bar.
  auto *nav = new QListWidget(this);
  nav->setFixedWidth(64);
  nav->setFrameShape(QFrame::NoFrame);
  nav->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  nav->setMouseTracking(true);  // so hover repaints
  nav->setIconSize(QSize(kNavIconPx, kNavIconPx));
  nav->setItemDelegate(new NavDelegate(nav));
  nav->setStyleSheet(
      "QListWidget { background: transparent; border: none; outline: 0; }");
  split->addWidget(nav);
  auto *stack = new QStackedWidget(this);
  split->addWidget(stack, 1);
  root->addLayout(split, 1);

  connect(nav, &QListWidget::currentRowChanged, stack,
          &QStackedWidget::setCurrentIndex);

  auto newPane = [](const QString &title) {
    auto *w = new QWidget;
    auto *v = new QVBoxLayout(w);
    v->setContentsMargins(18, 14, 18, 14);
    v->setSpacing(2);
    v->addWidget(paneHeader(title));
    return v;
  };
  auto finishPane = [&](const QString &navText, ui::Icon icon,
                        QVBoxLayout *lay) {
    lay->addStretch();
    auto *sc = new QScrollArea;
    sc->setWidgetResizable(true);
    sc->setFrameShape(QFrame::NoFrame);
    sc->setWidget(lay->parentWidget());
    stack->addWidget(sc);
    auto *item = new QListWidgetItem(navIcon(icon), QString());
    item->setToolTip(navText);  // name on hover (delegate centres the glyph)
    nav->addItem(item);
  };

  // ---- Telemetry Streams (all placeholder — needs SET_STREAM_RATE) ----
  {
    auto *v = newPane("Telemetry Streams");
    auto *note = new QLabel(
        "Per-stream enable + request rate. Disable unused streams or lower "
        "their rate to save link bandwidth.");
    note->setWordWrap(true);
    note->setStyleSheet(
        QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextMuted)));
    v->addWidget(note);

    struct S { const char *name; const char *type; const char *rate; bool on; };
    const S streams[] = {
        {"Heartbeat", "0x0", "1", true},
        {"IMU — full", "0x1", "200", true},
        {"IMU — compressed", "0x2", "50", false},
        {"Attitude", "0x4", "100", true},
        {"RC Channels", "0x5", "50", true},
        {"System Status", "0x6", "5", true},
        {"Motor Outputs", "0x8", "50", true},
        {"Perf / Kernel", "0x9", "2", true},
        {"Log Messages", "0x7", "on event", true},
    };
    auto *tbl = new QTableWidget(int(sizeof(streams) / sizeof(streams[0])), 4);
    tbl->setHorizontalHeaderLabels({"Stream", "Type", "Rate (Hz)", "On"});
    tbl->verticalHeader()->setVisible(false);
    tbl->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    tbl->setEditTriggers(QAbstractItemView::NoEditTriggers);
    tbl->setSelectionMode(QAbstractItemView::NoSelection);
    int r = 0;
    for (const auto &s : streams) {
      tbl->setItem(r, 0, new QTableWidgetItem(s.name));
      tbl->setItem(r, 1, new QTableWidgetItem(s.type));
      tbl->setItem(r, 2, new QTableWidgetItem(s.rate));
      tbl->setItem(r, 3, new QTableWidgetItem(s.on ? "✓" : "—"));
      tbl->item(r, 3)->setTextAlignment(Qt::AlignCenter);
      ++r;
    }
    tbl->setEnabled(false);
    tbl->setToolTip(kSoon);
    v->addWidget(tbl);
    auto *est = new QLabel("est. downlink ≈ 31.4 KB/s of 115 KB/s · " +
                           soonTag());
    est->setStyleSheet(
        QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextDim)));
    est->setAlignment(Qt::AlignRight);
    v->addWidget(est);
    finishPane("Telemetry Streams", ui::Icon::Streams, v);
  }

  // ---- Link & Connection ----
  {
    auto *v = newPane("Link & Connection");

    m_transportCombo = new QComboBox(this);
    m_transportCombo->addItems({"Serial", "UDP"});
    {
      auto rr = makeRow("Default transport", "selected on launch",
                        m_transportCombo);
      v->addWidget(rr.w);
      track(m_transportCombo, rr.name);
    }
    // Switching transport reconfigures the port + baud controls live.
    connect(m_transportCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int) { updateTransportDependent(); });

    // Port: a serial-port combo (Serial) or a numeric UDP port (UDP), swapped
    // by the stack so the field always matches the transport. The combo is a
    // fixed pick-list (detected ports + "Ask each time") — not editable, so the
    // "Ask each time" sentinel can't be typed over.
    m_portCombo = new QComboBox(this);
    m_portCombo->addItem(tr("Ask each time"));  // index 0 → no override
    for (const auto &pi : QSerialPortInfo::availablePorts())
      m_portCombo->addItem(pi.portName());
    m_udpPortSpin = new QSpinBox(this);
    m_udpPortSpin->setRange(1, 65535);
    m_udpPortSpin->setValue(14550);
    m_portStack = new QStackedWidget(this);
    m_portStack->addWidget(m_portCombo);    // page 0 = Serial
    m_portStack->addWidget(m_udpPortSpin);  // page 1 = UDP
    {
      auto rr = makeRow("Default port", "pre-filled on launch", m_portStack);
      v->addWidget(rr.w);
      track(m_portCombo, rr.name);
      track(m_udpPortSpin, rr.name);
    }

    m_baudCombo = new QComboBox(this);
    for (int b : {115200, 921600, 460800, 57600, 9600})
      m_baudCombo->addItem(QString::number(b), b);
    {
      auto rr = makeRow("Default baud", "pre-filled on launch", m_baudCombo);
      m_baudRow = rr.w;  // for the "locked" tooltip when UDP
      v->addWidget(rr.w);
      track(m_baudCombo, rr.name);
    }

    m_autoReconnectChk = new QCheckBox(this);
    m_autoReconnectChk->setToolTip(
        "Re-open the link with the same port and baud after a transient error "
        "(up to 5 attempts with exponential backoff).");
    {
      auto rr = makeRow("Auto-reconnect", "re-open the link after a drop",
                        m_autoReconnectChk);
      v->addWidget(rr.w);
      track(m_autoReconnectChk, rr.name);
    }

    m_reconnectSpin = new QDoubleSpinBox(this);
    m_reconnectSpin->setRange(0.1, 30.0);
    m_reconnectSpin->setSingleStep(0.5);
    m_reconnectSpin->setValue(1.0);
    m_reconnectSpin->setSuffix(" s");
    {
      auto rr = makeRow("Reconnect interval", "base delay between retries",
                        m_reconnectSpin);
      v->addWidget(rr.w);
      track(m_reconnectSpin, rr.name);
    }

    m_linkLossSpin = new QSpinBox(this);
    m_linkLossSpin->setRange(200, 60000);
    m_linkLossSpin->setSingleStep(100);
    m_linkLossSpin->setValue(1500);
    m_linkLossSpin->setSuffix(" ms");
    {
      auto rr = makeRow("Link-loss timeout",
                        "no-heartbeat → flag the link disconnected (ms)",
                        m_linkLossSpin);
      v->addWidget(rr.w);
      track(m_linkLossSpin, rr.name);
    }

    m_syncPeriodSpin = new QSpinBox(this);
    m_syncPeriodSpin->setRange(100, 60000);
    m_syncPeriodSpin->setValue(5000);
    m_syncPeriodSpin->setSingleStep(100);
    m_syncPeriodSpin->setSuffix(" ms");
    {
      auto rr = makeRow("Time-sync period",
                        "heartbeat interval to sync the FC clock (ms)",
                        m_syncPeriodSpin);
      v->addWidget(rr.w);
      track(m_syncPeriodSpin, rr.name);
    }
    finishPane("Link & Connection", ui::Icon::Link, v);
  }

  // ---- Units & Display ----
  {
    auto *v = newPane("Units & Display");
    m_themeCombo = new QComboBox(this);
    m_themeCombo->addItem("Dark (Navigator)");
    m_themeCombo->addItem("Midnight");
    m_themeCombo->addItem("High Contrast");
    m_themeCombo->setToolTip(
        "Application colour scheme (only Dark is currently themed).");
    {
      auto rr = makeRow("Theme", "application color scheme", m_themeCombo);
      v->addWidget(rr.w);
      track(m_themeCombo, rr.name);
    }

    // Unit system is a convenience preset: picking Metric/Imperial snaps the
    // altitude + speed dropdowns to the matching units (each still freely
    // overridable below). Angles stay independent — both systems use degrees.
    m_unitSystemCombo = combo({"Metric", "Imperial"});
    {
      auto rr = makeRow("Unit system", "preset for altitude + speed",
                        m_unitSystemCombo);
      v->addWidget(rr.w);
      track(m_unitSystemCombo, rr.name);
    }

    m_angleCombo = combo({"deg", "rad"});
    {
      auto rr = makeRow("Angles", "attitude readout units", m_angleCombo);
      v->addWidget(rr.w);
      track(m_angleCombo, rr.name);
    }

    m_altCombo = combo({"metres (m)", "feet (ft)"});
    {
      auto rr = makeRow("Altitude", "HUD altitude tape units", m_altCombo);
      v->addWidget(rr.w);
      track(m_altCombo, rr.name);
    }

    m_speedCombo = combo({"m/s", "km/h", "mph"});
    {
      auto rr = makeRow("Speed", "HUD speed tape units", m_speedCombo);
      v->addWidget(rr.w);
      track(m_speedCombo, rr.name);
    }

    // Metric ⇒ m + m/s; Imperial ⇒ ft + mph. Connected after the alt/speed
    // combos exist; fires only on a real user pick (setSettings blocks signals).
    connect(m_unitSystemCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int i) {
              const bool imperial = i == 1;
              m_altCombo->setCurrentIndex(imperial ? 1 : 0);
              m_speedCombo->setCurrentIndex(imperial ? 2 : 0);
            });

    m_decimalsSpin = spin(0, 6, 2);
    {
      auto rr = makeRow("Decimal places", "readout precision", m_decimalsSpin);
      v->addWidget(rr.w);
      track(m_decimalsSpin, rr.name);
    }

    m_startupCombo =
        combo({"Last viewed", "Flight Dashboard", "Simulator"});
    {
      auto rr = makeRow("Startup page", "page shown on launch", m_startupCombo);
      v->addWidget(rr.w);
      track(m_startupCombo, rr.name);
    }

    m_recentViewsSpin = new QSpinBox(this);
    m_recentViewsSpin->setRange(2, 9);
    m_recentViewsSpin->setValue(5);
    m_recentViewsSpin->setToolTip(
        "How many recently-viewed pages the Ctrl+Tab switcher cycles through.");
    {
      auto rr = makeRow("Recent-views switcher",
                        "depth of the Ctrl+Tab MRU cycle", m_recentViewsSpin);
      v->addWidget(rr.w);
      track(m_recentViewsSpin, rr.name);
    }

    m_restoreLayoutChk = check(true);
    {
      auto rr = makeRow("Restore layout",
                        "remember window size, splitters + last page",
                        m_restoreLayoutChk);
      v->addWidget(rr.w);
      track(m_restoreLayoutChk, rr.name);
    }
    finishPane("Units & Display", ui::Icon::Display, v);
  }

  // ---- Plots & Graphs ----
  {
    auto *v = newPane("Plots & Graphs");
    m_graphWindowSpin = new QSpinBox(this);
    m_graphWindowSpin->setRange(1, 60);
    m_graphWindowSpin->setValue(5);
    m_graphWindowSpin->setSuffix(" s");
    {
      auto rr = makeRow("Graph window", "time span shown (s)", m_graphWindowSpin);
      v->addWidget(rr.w);
      track(m_graphWindowSpin, rr.name);
    }

    m_graphDropoutSpin = new QDoubleSpinBox(this);
    m_graphDropoutSpin->setRange(0.0, 0.99);
    m_graphDropoutSpin->setValue(0.0);
    m_graphDropoutSpin->setSingleStep(0.1);
    m_graphDropoutSpin->setDecimals(2);
    m_graphDropoutSpin->setToolTip(
        "Decimation for high-rate streams — drop samples to save RAM.");
    {
      auto rr = makeRow("Downsample factor", "decimation for high-rate streams",
                        m_graphDropoutSpin);
      v->addWidget(rr.w);
      track(m_graphDropoutSpin, rr.name);
    }

    m_sigmaChk = check(true);
    {
      auto rr = makeRow("Rolling σ traces", "overlay std-dev on IMU graphs",
                        m_sigmaChk);
      v->addWidget(rr.w);
      track(m_sigmaChk, rr.name);
    }

    m_stateBandChk = check(false);
    {
      auto rr = makeRow("Vehicle-state band",
                        "colour the plot background by FC state", m_stateBandChk);
      v->addWidget(rr.w);
      track(m_stateBandChk, rr.name);
    }

    m_traceWidthSpin = dspin(0.5, 4.0, 1.4, 0.1);
    {
      auto rr = makeRow("Trace width", "graph line thickness", m_traceWidthSpin);
      v->addWidget(rr.w);
      track(m_traceWidthSpin, rr.name);
    }

    m_antialiasChk = check(false);
    {
      auto rr = makeRow("Antialias traces", "smoother lines (slightly more CPU)",
                        m_antialiasChk);
      v->addWidget(rr.w);
      track(m_antialiasChk, rr.name);
    }
    finishPane("Plots & Graphs", ui::Icon::Plots, v);
  }

  // ---- Logging & Recording ----
  {
    auto *v = newPane("Logging & Recording");

    m_recordOnConnectChk = new QCheckBox(this);
    m_recordOnConnectChk->setToolTip(
        "Tee every received frame to a timestamped .bin under the log directory "
        "while connected, for later replay.");
    {
      auto rr = makeRow("Record telemetry", "auto-log all packets on connect",
                        m_recordOnConnectChk);
      v->addWidget(rr.w);
      track(m_recordOnConnectChk, rr.name);
    }

    // Log directory: a read-only path field + Browse. Empty ⇒ the ~/vayu-logs
    // default (shown as placeholder text).
    {
      auto *dirW = new QWidget;
      auto *dh = new QHBoxLayout(dirW);
      dh->setContentsMargins(0, 0, 0, 0);
      dh->setSpacing(6);
      m_logDirEdit = new QLineEdit;
      m_logDirEdit->setReadOnly(true);
      m_logDirEdit->setPlaceholderText(QStringLiteral("~/vayu-logs (default)"));
      m_logDirEdit->setMinimumWidth(180);
      auto *browse = new QPushButton(tr("Browse…"));
      dh->addWidget(m_logDirEdit, 1);
      dh->addWidget(browse);
      auto rr = makeRow("Log directory", "where recordings + exports are written",
                        dirW);
      v->addWidget(rr.w);
      connect(browse, &QPushButton::clicked, this, [this, lbl = rr.name] {
        const QString start = m_logDirEdit->text().isEmpty()
                                  ? QDir::home().filePath("vayu-logs")
                                  : m_logDirEdit->text();
        const QString dir = QFileDialog::getExistingDirectory(
            this, tr("Choose log directory"), start);
        if (!dir.isEmpty()) {
          m_logDirEdit->setText(dir);
          markDirty(lbl);
        }
      });
    }

    m_timestampCombo = combo({"Local", "UTC", "Sim t (T+)"});
    {
      auto rr = makeRow("Timestamps", "System Log time prefix", m_timestampCombo);
      v->addWidget(rr.w);
      track(m_timestampCombo, rr.name);
    }

    m_maxLogLinesSpin = spin(100, 100000, 2000);
    {
      auto rr = makeRow("Max log lines", "ring-buffer cap in the System Log",
                        m_maxLogLinesSpin);
      v->addWidget(rr.w);
      track(m_maxLogLinesSpin, rr.name);
    }

    m_exportOnDisconnectChk = check(false);
    {
      auto rr = makeRow("Export on disconnect",
                        "dump the session log to a .log automatically",
                        m_exportOnDisconnectChk);
      v->addWidget(rr.w);
      track(m_exportOnDisconnectChk, rr.name);
    }
    finishPane("Logging & Recording", ui::Icon::Logging, v);
  }

  // ---- Alerts & Audio (all placeholder) ----
  {
    auto *v = newPane("Alerts & Audio");

    m_toastChk = check(true);
    {
      auto rr = makeRow("Toast notifications", "in-app status pop-ups",
                        m_toastChk);
      v->addWidget(rr.w);
      track(m_toastChk, rr.name);
    }

    m_audioAlertsChk = check(true);
    {
      auto rr = makeRow("Audio alerts", "chimes for arm / disarm / failsafe",
                        m_audioAlertsChk);
      v->addWidget(rr.w);
      track(m_audioAlertsChk, rr.name);
    }

    // Battery warn/crit stay coming-soon — no battery voltage in telemetry yet.
    v->addWidget(soonRow("Battery warning", "amber alert below (V/cell)",
                         dspin(2.5, 4.2, 3.50, 0.05)));
    v->addWidget(soonRow("Battery critical", "red alert below (V/cell)",
                         dspin(2.5, 4.2, 3.30, 0.05)));

    m_confirmArmChk = check(true);
    {
      auto rr = makeRow("Confirm before ARM", "require a click-through to arm",
                        m_confirmArmChk);
      v->addWidget(rr.w);
      track(m_confirmArmChk, rr.name);
    }

    m_simPropAudioChk = check(false);
    {
      auto rr = makeRow("Sim prop audio", "synthesize rotor noise by default",
                        m_simPropAudioChk);
      v->addWidget(rr.w);
      track(m_simPropAudioChk, rr.name);
    }
    finishPane("Alerts & Audio", ui::Icon::Alerts, v);
  }

  // ---- Advanced ----
  {
    auto *v = newPane("Advanced");

    m_packetBufferSpin = spin(100, 100000, 5000);
    {
      auto rr = makeRow("Packet buffer", "rows kept in the Packet Analyzer",
                        m_packetBufferSpin);
      v->addWidget(rr.w);
      track(m_packetBufferSpin, rr.name);
    }

    // Perf poll rate stays coming-soon: kernel-perf is pushed by the firmware
    // (broadcast), not polled, so there's no rate for the GCS to set yet.
    v->addWidget(soonRow("Perf poll rate", "kernel-perf refresh (Hz)",
                         spin(1, 60, 5)));

    m_crcCheckChk = check(true);
    {
      auto rr = makeRow("CRC checking", "drop packets that fail CRC",
                        m_crcCheckChk);
      v->addWidget(rr.w);
      track(m_crcCheckChk, rr.name);
    }

    // Settings file import/export — immediate actions (not part of Apply).
    {
      auto *ie = new QWidget;
      auto *ieh = new QHBoxLayout(ie);
      ieh->setContentsMargins(0, 0, 0, 0);
      auto *importBtn = new QPushButton(tr("Import…"));
      auto *exportBtn = new QPushButton(tr("Export…"));
      ieh->addWidget(importBtn);
      ieh->addWidget(exportBtn);
      v->addWidget(makeRow("Settings file", "import / export the config", ie).w);
      connect(importBtn, &QPushButton::clicked, this,
              &SettingsWidget::importSettings);
      connect(exportBtn, &QPushButton::clicked, this,
              &SettingsWidget::exportSettings);
    }

    {
      auto *reset = new QPushButton(tr("Reset…"));
      v->addWidget(makeRow("Reset to defaults", "restore all settings", reset).w);
      connect(reset, &QPushButton::clicked, this,
              &SettingsWidget::resetToDefaults);
    }
    finishPane("Advanced", ui::Icon::Advanced, v);
  }

  // ---- Apply bar (explicit commit) ----
  auto *applyRow = new QHBoxLayout();
  m_dirtyHint = new QLabel(tr("● Unsaved changes"), this);
  m_dirtyHint->setStyleSheet(
      QString("color:%1; font-weight:600;").arg(Theme::hex(Theme::kWarn)));
  m_dirtyHint->setVisible(false);
  applyRow->addWidget(m_dirtyHint);
  applyRow->addStretch();
  m_applyBtn->setStyleSheet(
      QString("QPushButton { background:%1; color:#10131A; font-weight:600; "
              "padding:5px 18px; border-radius:4px; }"
              "QPushButton:disabled { background:%2; color:%3; }")
          .arg(Theme::hex(Theme::kAccent), Theme::hex(Theme::kBorderStrong),
               Theme::hex(Theme::kTextDim)));
  applyRow->addWidget(m_applyBtn);
  root->addLayout(applyRow);

  connect(m_applyBtn, &QPushButton::clicked, this, [this] {
    emit applyRequested();  // MainWindow reads getSettings(), applies + saves
    clearDirty();
  });

  nav->setCurrentRow(0);
  updateTransportDependent();
}

void SettingsWidget::markDirty(QLabel *label) {
  if (label) {
    label->setStyleSheet(QString("font-weight:600; color:%1;")
                             .arg(Theme::hex(Theme::kWarn)));
    m_dirtyLabels.insert(label);
  }
  if (m_applyBtn) m_applyBtn->setEnabled(true);
  if (m_dirtyHint) m_dirtyHint->setVisible(true);
}

void SettingsWidget::clearDirty() {
  for (QLabel *l : m_dirtyLabels) l->setStyleSheet(rowLabelStyle());
  m_dirtyLabels.clear();
  if (m_applyBtn) m_applyBtn->setEnabled(false);
  if (m_dirtyHint) m_dirtyHint->setVisible(false);
}

void SettingsWidget::exportSettings() {
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Export settings"), QStringLiteral("vayu_settings.dat"),
      tr("Vayu settings (*.dat);;All files (*)"));
  if (path.isEmpty()) return;
  if (SettingsManager::saveToPath(path, getSettings()))
    Notify::ok(this, tr("Exported settings → %1").arg(path));
  else
    Notify::error(this, tr("Failed to export settings"));
}

void SettingsWidget::importSettings() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Import settings"), QString(),
      tr("Vayu settings (*.dat);;All files (*)"));
  if (path.isEmpty()) return;
  GcsSettings s;
  if (!SettingsManager::loadFromPath(path, s)) {
    Notify::error(this, tr("Not a valid Vayu settings file"));
    return;
  }
  setSettings(s);         // populate the form (clears dirty)
  emit applyRequested();  // apply + persist to the default store via MainWindow
  Notify::ok(this, tr("Imported settings from %1").arg(path));
}

void SettingsWidget::resetToDefaults() {
  if (QMessageBox::question(
          this, tr("Reset to defaults"),
          tr("Restore all settings to their defaults? This applies and saves "
             "immediately.")) != QMessageBox::Yes)
    return;
  setSettings(GcsSettings{});  // default-constructed = the defaults
  emit applyRequested();
  Notify::ok(this, tr("Settings reset to defaults"));
}

void SettingsWidget::updateTransportDependent() {
  if (!m_transportCombo || !m_portStack || !m_baudCombo) return;
  const bool udp = m_transportCombo->currentIndex() == 1;
  m_portStack->setCurrentIndex(udp ? 1 : 0);
  // Baud is meaningless over UDP — lock + fade it, and tip the row (a disabled
  // widget can't show its own tooltip, so the row container carries it).
  m_baudCombo->setEnabled(!udp);
  const QString tip = udp ? tr("Not available for the UDP transport") : QString();
  m_baudCombo->setToolTip(tip);
  if (m_baudRow) m_baudRow->setToolTip(tip);
}

void SettingsWidget::setSettings(const GcsSettings &s) {
  const QSignalBlocker bSync(m_syncPeriodSpin);
  m_syncPeriodSpin->setValue(s.syncPeriodMs);
  const QSignalBlocker bWin(m_graphWindowSpin);
  m_graphWindowSpin->setValue(s.graphWindowSec);
  const QSignalBlocker bDrop(m_graphDropoutSpin);
  m_graphDropoutSpin->setValue(s.graphDropoutRate);
  const QSignalBlocker bSig(m_sigmaChk);
  m_sigmaChk->setChecked(s.sigmaTraces);
  const QSignalBlocker bBand(m_stateBandChk);
  m_stateBandChk->setChecked(s.stateBand);
  const QSignalBlocker bTw(m_traceWidthSpin);
  m_traceWidthSpin->setValue(s.traceWidth);
  const QSignalBlocker bAa(m_antialiasChk);
  m_antialiasChk->setChecked(s.antialias);
  const QSignalBlocker bAuto(m_autoReconnectChk);
  m_autoReconnectChk->setChecked(s.autoReconnect);
  const QSignalBlocker bRec(m_recordOnConnectChk);
  m_recordOnConnectChk->setChecked(s.recordOnConnect);
  if (m_logDirEdit) m_logDirEdit->setText(s.logDirectory);
  const QSignalBlocker bTs(m_timestampCombo);
  m_timestampCombo->setCurrentIndex(s.timestampMode);
  const QSignalBlocker bMll(m_maxLogLinesSpin);
  m_maxLogLinesSpin->setValue(s.maxLogLines);
  const QSignalBlocker bEod(m_exportOnDisconnectChk);
  m_exportOnDisconnectChk->setChecked(s.exportOnDisconnect);
  const QSignalBlocker bToast(m_toastChk);
  m_toastChk->setChecked(s.toastNotifications);
  const QSignalBlocker bAud(m_audioAlertsChk);
  m_audioAlertsChk->setChecked(s.audioAlerts);
  const QSignalBlocker bCa(m_confirmArmChk);
  m_confirmArmChk->setChecked(s.confirmBeforeArm);
  const QSignalBlocker bSpa(m_simPropAudioChk);
  m_simPropAudioChk->setChecked(s.simPropAudio);
  const QSignalBlocker bPkt(m_packetBufferSpin);
  m_packetBufferSpin->setValue(s.packetBufferRows);
  const QSignalBlocker bCrc(m_crcCheckChk);
  m_crcCheckChk->setChecked(s.crcCheck);
  const QSignalBlocker bMru(m_recentViewsSpin);
  m_recentViewsSpin->setValue(s.recentViewsCount);
  const QSignalBlocker bTheme(m_themeCombo);
  m_themeCombo->setCurrentIndex(s.theme);

  const QSignalBlocker bUsys(m_unitSystemCombo);
  m_unitSystemCombo->setCurrentIndex(s.unitSystem);
  const QSignalBlocker bAng(m_angleCombo);
  m_angleCombo->setCurrentIndex(s.angleUnit);
  const QSignalBlocker bAlt(m_altCombo);
  m_altCombo->setCurrentIndex(s.altitudeUnit);
  const QSignalBlocker bSpd(m_speedCombo);
  m_speedCombo->setCurrentIndex(s.speedUnit);
  const QSignalBlocker bDec(m_decimalsSpin);
  m_decimalsSpin->setValue(s.decimals);
  const QSignalBlocker bStart(m_startupCombo);
  m_startupCombo->setCurrentIndex(s.startupPage);
  const QSignalBlocker bRl(m_restoreLayoutChk);
  m_restoreLayoutChk->setChecked(s.restoreLayout);

  {
    const QSignalBlocker bT(m_transportCombo);
    m_transportCombo->setCurrentIndex(s.defaultTransport);
  }
  updateTransportDependent();  // reflect the stack/baud state for this transport
  if (s.defaultTransport == 1) {  // UDP → defaultPort holds the port number
    const QSignalBlocker bU(m_udpPortSpin);
    m_udpPortSpin->setValue(s.defaultPort.isEmpty() ? 14550
                                                    : s.defaultPort.toInt());
  } else {
    const QSignalBlocker bP(m_portCombo);
    if (s.defaultPort.isEmpty()) {
      m_portCombo->setCurrentIndex(0);  // Ask each time
    } else {
      int idx = m_portCombo->findText(s.defaultPort);
      if (idx < 0) {  // saved port not currently present — keep it selectable
        m_portCombo->addItem(s.defaultPort);
        idx = m_portCombo->count() - 1;
      }
      m_portCombo->setCurrentIndex(idx);
    }
  }
  {
    const QSignalBlocker bB(m_baudCombo);
    const int idx = m_baudCombo->findData(s.defaultBaud);
    m_baudCombo->setCurrentIndex(idx >= 0 ? idx : 0);
  }
  const QSignalBlocker bRi(m_reconnectSpin);
  m_reconnectSpin->setValue(s.reconnectIntervalSec);
  const QSignalBlocker bLl(m_linkLossSpin);
  m_linkLossSpin->setValue(s.linkLossTimeoutMs);

  clearDirty();  // a programmatic load is not a user edit
}

GcsSettings SettingsWidget::getSettings() const {
  GcsSettings s;
  s.syncPeriodMs = m_syncPeriodSpin->value();
  s.graphWindowSec = m_graphWindowSpin->value();
  s.graphDropoutRate = m_graphDropoutSpin->value();
  s.sigmaTraces = !m_sigmaChk || m_sigmaChk->isChecked();
  s.stateBand = !m_stateBandChk || m_stateBandChk->isChecked();
  s.traceWidth = m_traceWidthSpin ? m_traceWidthSpin->value() : 1.4;
  s.antialias = !m_antialiasChk || m_antialiasChk->isChecked();
  s.autoReconnect = m_autoReconnectChk && m_autoReconnectChk->isChecked();
  s.recordOnConnect =
      m_recordOnConnectChk && m_recordOnConnectChk->isChecked();
  s.logDirectory = m_logDirEdit ? m_logDirEdit->text() : QString();
  s.timestampMode = m_timestampCombo ? m_timestampCombo->currentIndex() : 0;
  s.maxLogLines = m_maxLogLinesSpin ? m_maxLogLinesSpin->value() : 2000;
  s.exportOnDisconnect =
      m_exportOnDisconnectChk && m_exportOnDisconnectChk->isChecked();
  s.toastNotifications = !m_toastChk || m_toastChk->isChecked();
  s.audioAlerts = !m_audioAlertsChk || m_audioAlertsChk->isChecked();
  s.confirmBeforeArm = !m_confirmArmChk || m_confirmArmChk->isChecked();
  s.simPropAudio = m_simPropAudioChk && m_simPropAudioChk->isChecked();
  s.packetBufferRows = m_packetBufferSpin ? m_packetBufferSpin->value() : 5000;
  s.crcCheck = !m_crcCheckChk || m_crcCheckChk->isChecked();
  s.recentViewsCount = m_recentViewsSpin ? m_recentViewsSpin->value() : 5;
  s.theme = m_themeCombo ? m_themeCombo->currentIndex() : 0;
  s.unitSystem = m_unitSystemCombo ? m_unitSystemCombo->currentIndex() : 0;
  s.angleUnit = m_angleCombo ? m_angleCombo->currentIndex() : 0;
  s.altitudeUnit = m_altCombo ? m_altCombo->currentIndex() : 0;
  s.speedUnit = m_speedCombo ? m_speedCombo->currentIndex() : 0;
  s.decimals = m_decimalsSpin ? m_decimalsSpin->value() : 2;
  s.startupPage = m_startupCombo ? m_startupCombo->currentIndex() : 0;
  s.restoreLayout = !m_restoreLayoutChk || m_restoreLayoutChk->isChecked();
  s.defaultTransport = m_transportCombo ? m_transportCombo->currentIndex() : 0;
  if (s.defaultTransport == 1) {
    s.defaultPort = m_udpPortSpin ? QString::number(m_udpPortSpin->value())
                                  : QString();
  } else {
    s.defaultPort =
        (!m_portCombo || m_portCombo->currentText() == tr("Ask each time"))
            ? QString()
            : m_portCombo->currentText();
  }
  s.defaultBaud = m_baudCombo ? m_baudCombo->currentData().toInt() : 115200;
  s.reconnectIntervalSec = m_reconnectSpin ? m_reconnectSpin->value() : 1.0;
  s.linkLossTimeoutMs = m_linkLossSpin ? m_linkLossSpin->value() : 1500;
  return s;
}
