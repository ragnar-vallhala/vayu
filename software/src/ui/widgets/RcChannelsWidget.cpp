#include "RcChannelsWidget.h"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>

RcChannelsWidget::RcChannelsWidget(QWidget *parent) : QWidget(parent) {
  setWindowTitle("RC Channels Monitor");
  setMinimumSize(400, 600);
  setAttribute(Qt::WA_DeleteOnClose,
               false); // Keep it alive or handle destruction

  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(20, 20, 20, 20);
  mainLayout->setSpacing(10);

  // Back button
  QHBoxLayout *topRow = new QHBoxLayout();
  QPushButton *backBtn = new QPushButton(" ←  Back", this);
  backBtn->setFixedWidth(100);
  backBtn->setStyleSheet("QPushButton { background: #3E4452; color: #fff; "
                         "border-radius: 4px; padding: 6px; }");
  connect(backBtn, &QPushButton::clicked, this,
          &RcChannelsWidget::backToHomeRequested);
  topRow->addWidget(backBtn);
  topRow->addStretch();
  mainLayout->addLayout(topRow);

  QGroupBox *groupBox = new QGroupBox("Raw RC Values (us)", this);
  QVBoxLayout *groupLayout = new QVBoxLayout(groupBox);

  for (int i = 0; i < 14; ++i) {
    QHBoxLayout *row = new QHBoxLayout();

    QLabel *nameLabel = new QLabel(QString("CH %1:").arg(i + 1), this);
    nameLabel->setFixedWidth(50);

    QProgressBar *bar = new QProgressBar(this);
    bar->setRange(800, 2200);
    bar->setValue(1500);
    bar->setTextVisible(false);
    bar->setStyleSheet(
        "QProgressBar { border: 1px solid #444; border-radius: 3px; "
        "background: #222; height: 15px; }"
        "QProgressBar::chunk { background: qlineargradient(x1:0, y1:0, x2:1, "
        "y2:0, stop:0 #4a9eff, stop:1 #80c0ff); border-radius: 2px; }");

    QLabel *valLabel = new QLabel("1500", this);
    valLabel->setFixedWidth(40);
    valLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    row->addWidget(nameLabel);
    row->addWidget(bar);
    row->addWidget(valLabel);

    groupLayout->addLayout(row);

    m_bars.append(bar);
    m_labels.append(valLabel);
  }

  mainLayout->addWidget(groupBox);

  setStyleSheet("QWidget { background-color: #1e1e1e; color: #eee; "
                "font-family: 'Segoe UI', Arial; } "
                "QGroupBox { font-weight: bold; border: 1px solid #444; "
                "margin-top: 10px; padding-top: 10px; } "
                "QLabel { font-size: 11px; }");
}

void RcChannelsWidget::updateChannels(const RcData &data) {
  for (int i = 0; i < 14; ++i) {
    uint16_t val = data.channels[i];
    m_bars[i]->setValue(val);
    m_labels[i]->setText(QString::number(val));

    // Smooth color change if failsafe (0)
    if (val == 0) {
      m_bars[i]->setStyleSheet("QProgressBar::chunk { background: #ff4a4a; }");
    } else {
      m_bars[i]->setStyleSheet(
          "QProgressBar::chunk { background: qlineargradient(x1:0, y1:0, x2:1, "
          "y2:0, stop:0 #4a9eff, stop:1 #80c0ff); }");
    }
  }
}
