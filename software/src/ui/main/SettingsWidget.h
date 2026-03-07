#pragma once

#include <QDoubleSpinBox>
#include <QPushButton>
#include <QSpinBox>
#include <QWidget>

class SettingsWidget : public QWidget {
  Q_OBJECT

public:
  explicit SettingsWidget(QWidget *parent = nullptr);

signals:
  void backToHomeRequested();
  void syncPeriodChanged(int ms);
  void graphWindowChanged(int seconds);
  void graphDropoutChanged(double rate);

private:
  QSpinBox *m_syncPeriodSpin = nullptr;
  QSpinBox *m_graphWindowSpin = nullptr;
  QDoubleSpinBox *m_graphDropoutSpin = nullptr;
};
