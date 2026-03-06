#pragma once

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

private:
  QSpinBox *m_syncPeriodSpin = nullptr;
};
