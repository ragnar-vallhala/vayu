#pragma once

#include "../../protocol/DroneProtocol.h"
#include "../core/Types.h"
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QWidget>

// Tuning + readout surface for the FFT dynamic gyro notch.
//
// Top half sends CMD_SET_GYRO_NOTCH (master enable + detection band / Q /
// prominence, applied live to every axis and persisted on the FC). Bottom half
// is a live readout of NOTCH_STATUS: the tracked center frequencies, 3 slots per
// axis (0 Hz = that slot is currently bypassed). Mirrors CalibrationWidget's
// command-emitting shape: it emits commandRequested() and MainWindow forwards it
// to the FC when TX is allowed.
class GyroNotchWidget : public QWidget {
  Q_OBJECT

public:
  explicit GyroNotchWidget(QWidget *parent = nullptr);
  void setProtocol(DroneProtocol *protocol);

public slots:
  // Gate the Apply control on the link state (same policy as CalibrationWidget).
  void setConnected(bool connected);

signals:
  void backToHomeRequested();
  void commandRequested(const QByteArray &data);

private slots:
  void onApplyClicked();
  void onNotchStatus(const NotchStatusData &d);

private:
  DroneProtocol *m_protocol = nullptr;
  bool m_connected = false;

  QCheckBox *m_enable = nullptr;
  QDoubleSpinBox *m_q = nullptr;
  QDoubleSpinBox *m_fmin = nullptr;
  QDoubleSpinBox *m_fmax = nullptr;
  QDoubleSpinBox *m_minRatio = nullptr;
  QPushButton *m_applyBtn = nullptr;

  QLabel *m_statusLabel = nullptr;      // enabled / not
  QLabel *m_center[3][3] = {{nullptr}}; // [axis][slot] center-freq readouts
};
