#pragma once

#include "../../core/SettingsManager.h"
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QSpinBox>
#include <QWidget>

class SettingsWidget : public QWidget {
  Q_OBJECT

public:
  explicit SettingsWidget(QWidget *parent = nullptr);

  void setSettings(const GcsSettings &s);
  GcsSettings getSettings() const;

signals:
  void backToHomeRequested();
  void syncPeriodChanged(int ms);
  void graphWindowChanged(int seconds);
  void graphDropoutChanged(double rate);
  void autoReconnectChanged(bool enabled);
  void recordOnConnectChanged(bool enabled);

private:
  QSpinBox *m_syncPeriodSpin = nullptr;
  QSpinBox *m_graphWindowSpin = nullptr;
  QDoubleSpinBox *m_graphDropoutSpin = nullptr;
  QCheckBox *m_autoReconnectChk = nullptr;
  QCheckBox *m_recordOnConnectChk = nullptr;
};
