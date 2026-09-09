#include "GyroNotchWidget.h"

#include "../../protocol/CommandCodec.h"

#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace {
// Compile-time firmware defaults (gyro_notch.c) so the panel opens on the values
// the FC actually boots with.
constexpr double kDefQ = 8.0;
constexpr double kDefFmin = 60.0;
constexpr double kDefFmax = 450.0;
constexpr double kDefMinRatio = 4.0;

QDoubleSpinBox *mkSpin(double lo, double hi, double step, double val,
                       const QString &suffix) {
  auto *s = new QDoubleSpinBox;
  s->setRange(lo, hi);
  s->setSingleStep(step);
  s->setValue(val);
  s->setDecimals(2);
  if (!suffix.isEmpty())
    s->setSuffix(suffix);
  return s;
}
} // namespace

GyroNotchWidget::GyroNotchWidget(QWidget *parent) : QWidget(parent) {
  auto *root = new QVBoxLayout(this);

  auto *title = new QLabel(tr("Dynamic Gyro Notch"));
  title->setStyleSheet("font-size:18px; font-weight:600;");
  root->addWidget(title);

  // ---- Tuning (CMD_SET_GYRO_NOTCH) ----------------------------------------
  auto *tune = new QGroupBox(tr("Detection"));
  auto *form = new QFormLayout(tune);

  m_enable = new QCheckBox(tr("Enabled"));
  form->addRow(m_enable);

  m_autoband = new QCheckBox(tr("Auto-band (learn on next hover)"));
  m_autoband->setToolTip(
      tr("One-shot: the FC characterises the hover spectrum and tightens the "
         "band around it. Cleared after it is sent."));
  form->addRow(m_autoband);

  m_q = mkSpin(0.5, 30.0, 0.5, kDefQ, QString());
  m_fmin = mkSpin(10.0, 490.0, 5.0, kDefFmin, tr(" Hz"));
  m_fmax = mkSpin(10.0, 490.0, 5.0, kDefFmax, tr(" Hz"));
  m_minRatio = mkSpin(1.0, 50.0, 0.5, kDefMinRatio, QString());
  form->addRow(tr("Q (width)"), m_q);
  form->addRow(tr("Band min"), m_fmin);
  form->addRow(tr("Band max"), m_fmax);
  form->addRow(tr("Min peak ratio"), m_minRatio);

  m_applyBtn = new QPushButton(tr("Apply to FC"));
  form->addRow(m_applyBtn);
  root->addWidget(tune);

  // ---- Live readout (NOTCH_STATUS) ----------------------------------------
  auto *live = new QGroupBox(tr("Tracked centers"));
  auto *grid = new QGridLayout(live);
  grid->addWidget(new QLabel(tr("Axis")), 0, 0);
  for (int slot = 0; slot < 3; ++slot)
    grid->addWidget(new QLabel(tr("Notch %1").arg(slot + 1)), 0, slot + 1);

  const char *axisNames[3] = {"Roll", "Pitch", "Yaw"};
  for (int axis = 0; axis < 3; ++axis) {
    grid->addWidget(new QLabel(tr(axisNames[axis])), axis + 1, 0);
    for (int slot = 0; slot < 3; ++slot) {
      m_center[axis][slot] = new QLabel(QStringLiteral("—"));
      m_center[axis][slot]->setStyleSheet("font-family:monospace;");
      grid->addWidget(m_center[axis][slot], axis + 1, slot + 1);
    }
  }
  root->addWidget(live);

  m_statusLabel = new QLabel(tr("NOT CONNECTED"));
  root->addWidget(m_statusLabel);

  root->addStretch(1);
  auto *back = new QPushButton(tr("Back"));
  root->addWidget(back);

  connect(back, &QPushButton::clicked, this,
          &GyroNotchWidget::backToHomeRequested);
  connect(m_applyBtn, &QPushButton::clicked, this,
          &GyroNotchWidget::onApplyClicked);

  setConnected(false);
}

void GyroNotchWidget::setProtocol(DroneProtocol *protocol) {
  m_protocol = protocol;
  if (m_protocol)
    connect(m_protocol, &DroneProtocol::notchStatusReceived, this,
            &GyroNotchWidget::onNotchStatus);
}

void GyroNotchWidget::setConnected(bool connected) {
  m_connected = connected;
  m_applyBtn->setEnabled(connected);
  if (!connected)
    m_statusLabel->setText(tr("NOT CONNECTED"));
}

void GyroNotchWidget::onApplyClicked() {
  // The FC treats a non-positive detection field as "leave unchanged"; the panel
  // always sends real positive values, so a full tune is applied every time.
  emit commandRequested(CommandCodec::encodeSetGyroNotch(
      m_enable->isChecked(), static_cast<float>(m_q->value()),
      static_cast<float>(m_fmin->value()), static_cast<float>(m_fmax->value()),
      static_cast<float>(m_minRatio->value()), m_autoband->isChecked()));
  // Auto-band is a one-shot trigger; untick so it isn't re-armed on the next Apply.
  m_autoband->setChecked(false);
}

void GyroNotchWidget::onNotchStatus(const NotchStatusData &d) {
  m_statusLabel->setText(d.enabled ? tr("Notch ENABLED")
                                   : tr("Notch disabled"));
  for (int axis = 0; axis < 3; ++axis) {
    for (int slot = 0; slot < 3; ++slot) {
      float hz = d.centerHz[axis][slot];
      m_center[axis][slot]->setText(
          hz > 0.0f
              ? QStringLiteral("%1 Hz").arg(static_cast<double>(hz), 0, 'f', 1)
              : QStringLiteral("—"));
    }
  }
}
