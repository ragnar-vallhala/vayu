#include "SettingsWidget.h"

#include "core/Theme.h"

#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

SettingsWidget::SettingsWidget(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(20, 20, 20, 20);
  layout->setSpacing(20);

  auto *headerLabel = new QLabel("<h2>Configuration</h2>", this);
  headerLabel->setStyleSheet(
      QString("color: %1;").arg(Theme::hex(Theme::kAccent)));
  layout->addWidget(headerLabel);

  // ---- Communication Group ----
  auto *commGroup = new QGroupBox("Communication", this);
  auto *commLayout = new QVBoxLayout(commGroup);

  auto *syncRow = new QHBoxLayout();
  syncRow->addWidget(new QLabel("Sync Message Period (ms):", this));

  m_syncPeriodSpin = new QSpinBox(this);
  m_syncPeriodSpin->setRange(100, 60000);
  m_syncPeriodSpin->setValue(5000);
  m_syncPeriodSpin->setSingleStep(100);
  m_syncPeriodSpin->setSuffix(" ms");
  // Spinbox chrome comes from the global QSS now; no inline style needed.

  connect(m_syncPeriodSpin, QOverload<int>::of(&QSpinBox::valueChanged), this,
          &SettingsWidget::syncPeriodChanged);

  syncRow->addWidget(m_syncPeriodSpin);
  syncRow->addStretch();
  commLayout->addLayout(syncRow);

  m_autoReconnectChk = new QCheckBox("Auto-reconnect on serial error", this);
  m_autoReconnectChk->setToolTip(
      "When set, the serial connection is automatically re-opened with the "
      "same port and baud after a transient error (up to 5 attempts with "
      "exponential backoff).");
  connect(m_autoReconnectChk, &QCheckBox::toggled, this,
          &SettingsWidget::autoReconnectChanged);
  commLayout->addWidget(m_autoReconnectChk);

  m_recordOnConnectChk =
      new QCheckBox("Record telemetry to a log on connect", this);
  m_recordOnConnectChk->setToolTip(
      "When set, every received telemetry frame is teed to a timestamped "
      ".bin under ~/vayu-logs while connected, for later replay.");
  connect(m_recordOnConnectChk, &QCheckBox::toggled, this,
          &SettingsWidget::recordOnConnectChanged);
  commLayout->addWidget(m_recordOnConnectChk);

  layout->addWidget(commGroup);

  // ---- Graph Settings Group ----
  auto *graphGroup = new QGroupBox("Graph Settings", this);
  auto *graphLayout = new QVBoxLayout(graphGroup);

  auto *windowRow = new QHBoxLayout();
  windowRow->addWidget(new QLabel("Window Duration (sec):", this));
  m_graphWindowSpin = new QSpinBox(this);
  m_graphWindowSpin->setRange(1, 60);
  m_graphWindowSpin->setValue(5);
  m_graphWindowSpin->setSuffix(" s");
  connect(m_graphWindowSpin, QOverload<int>::of(&QSpinBox::valueChanged), this,
          &SettingsWidget::graphWindowChanged);
  windowRow->addWidget(m_graphWindowSpin);
  windowRow->addStretch();
  graphLayout->addLayout(windowRow);

  auto *dropoutRow = new QHBoxLayout();
  dropoutRow->addWidget(new QLabel("Data Dropout Rate (0-1):", this));
  m_graphDropoutSpin = new QDoubleSpinBox(this);
  m_graphDropoutSpin->setRange(0.0, 0.99);
  m_graphDropoutSpin->setValue(0.0);
  m_graphDropoutSpin->setSingleStep(0.1);
  m_graphDropoutSpin->setDecimals(2);
  m_graphDropoutSpin->setToolTip(
      "Higher rate helps save RAM by dropping samples for visualization");
  connect(m_graphDropoutSpin,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &SettingsWidget::graphDropoutChanged);
  dropoutRow->addWidget(m_graphDropoutSpin);
  dropoutRow->addStretch();
  graphLayout->addLayout(dropoutRow);

  auto *recentRow = new QHBoxLayout();
  recentRow->addWidget(new QLabel("Recent-views switcher depth:", this));
  m_recentViewsSpin = new QSpinBox(this);
  m_recentViewsSpin->setRange(2, 9);
  m_recentViewsSpin->setValue(5);
  m_recentViewsSpin->setToolTip(
      "How many recently-viewed pages the Ctrl+Tab switcher cycles through.");
  connect(m_recentViewsSpin, QOverload<int>::of(&QSpinBox::valueChanged), this,
          &SettingsWidget::recentViewsCountChanged);
  recentRow->addWidget(m_recentViewsSpin);
  recentRow->addStretch();
  graphLayout->addLayout(recentRow);

  layout->addWidget(graphGroup);

  // ---- Units & Display Group ----
  auto *displayGroup = new QGroupBox("Units && Display", this);
  auto *displayLayout = new QVBoxLayout(displayGroup);

  auto *themeRow = new QHBoxLayout();
  themeRow->addWidget(new QLabel("Theme:", this));
  m_themeCombo = new QComboBox(this);
  m_themeCombo->addItem("Dark (Navigator)");
  m_themeCombo->addItem("Midnight");
  m_themeCombo->addItem("High Contrast");
  m_themeCombo->setToolTip(tr("Application colour scheme (mockup parity; only "
                              "Dark is currently themed)."));
  connect(m_themeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &SettingsWidget::themeChanged);
  themeRow->addWidget(m_themeCombo);
  themeRow->addStretch();
  displayLayout->addLayout(themeRow);

  layout->addWidget(displayGroup);

  layout->addStretch();
}

void SettingsWidget::setSettings(const GcsSettings &s) {
  m_syncPeriodSpin->blockSignals(true);
  m_syncPeriodSpin->setValue(s.syncPeriodMs);
  m_syncPeriodSpin->blockSignals(false);

  m_graphWindowSpin->blockSignals(true);
  m_graphWindowSpin->setValue(s.graphWindowSec);
  m_graphWindowSpin->blockSignals(false);

  m_graphDropoutSpin->blockSignals(true);
  m_graphDropoutSpin->setValue(s.graphDropoutRate);
  m_graphDropoutSpin->blockSignals(false);

  if (m_autoReconnectChk) {
    m_autoReconnectChk->blockSignals(true);
    m_autoReconnectChk->setChecked(s.autoReconnect);
    m_autoReconnectChk->blockSignals(false);
  }

  if (m_recordOnConnectChk) {
    m_recordOnConnectChk->blockSignals(true);
    m_recordOnConnectChk->setChecked(s.recordOnConnect);
    m_recordOnConnectChk->blockSignals(false);
  }

  if (m_recentViewsSpin) {
    m_recentViewsSpin->blockSignals(true);
    m_recentViewsSpin->setValue(s.recentViewsCount);
    m_recentViewsSpin->blockSignals(false);
  }

  if (m_themeCombo) {
    m_themeCombo->blockSignals(true);
    m_themeCombo->setCurrentIndex(s.theme);
    m_themeCombo->blockSignals(false);
  }
}

GcsSettings SettingsWidget::getSettings() const {
  GcsSettings s;
  s.syncPeriodMs = m_syncPeriodSpin->value();
  s.graphWindowSec = m_graphWindowSpin->value();
  s.graphDropoutRate = m_graphDropoutSpin->value();
  s.autoReconnect = m_autoReconnectChk && m_autoReconnectChk->isChecked();
  s.recordOnConnect =
      m_recordOnConnectChk && m_recordOnConnectChk->isChecked();
  s.recentViewsCount = m_recentViewsSpin ? m_recentViewsSpin->value() : 5;
  s.theme = m_themeCombo ? m_themeCombo->currentIndex() : 0;
  return s;
}
