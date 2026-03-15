#pragma once

#include "../../protocol/DroneProtocol.h"
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

class CalibrationWidget : public QWidget {
  Q_OBJECT

public:
  explicit CalibrationWidget(QWidget *parent = nullptr);
  void setProtocol(DroneProtocol *protocol);

signals:
  void backToHomeRequested();
  void commandRequested(const QByteArray &data);

private slots:
  void onStartClicked();
  void onCancelClicked();
  void onStatusReceived(const QString &msg);

private:
  DroneProtocol *m_protocol = nullptr;
  QPushButton *m_startBtn;
  QPushButton *m_cancelBtn;
  QProgressBar *m_progressBar;
  QLabel *m_statusLabel;
};
