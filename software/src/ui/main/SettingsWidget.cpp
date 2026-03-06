#include "SettingsWidget.h"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

SettingsWidget::SettingsWidget(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(20, 20, 20, 20);
  layout->setSpacing(20);

  auto *headerLabel = new QLabel("<h2>Settings</h2>", this);
  headerLabel->setStyleSheet("color: #61AFEF;");
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
  m_syncPeriodSpin->setStyleSheet("background: #21252B; color: #ABB2BF; "
                                  "border: 1px solid #3E4452; padding: 4px;");

  connect(m_syncPeriodSpin, QOverload<int>::of(&QSpinBox::valueChanged), this,
          &SettingsWidget::syncPeriodChanged);

  syncRow->addWidget(m_syncPeriodSpin);
  syncRow->addStretch();
  commLayout->addLayout(syncRow);

  layout->addWidget(commGroup);

  layout->addStretch();

  // ---- Back Button ----
  auto *backBtn = new QPushButton("Back to Home", this);
  backBtn->setFixedWidth(150);
  backBtn->setStyleSheet(
      "QPushButton { background: #2C313A; color: #ABB2BF; border: 1px solid "
      "#3E4452; padding: 8px; font-weight: bold; borderRadius: 4px; }"
      "QPushButton:hover { background: #3E4452; }");
  connect(backBtn, &QPushButton::clicked, this,
          &SettingsWidget::backToHomeRequested);
  layout->addWidget(backBtn);
}
