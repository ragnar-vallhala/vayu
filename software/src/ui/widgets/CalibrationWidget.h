#pragma once

#include "../../protocol/DroneProtocol.h"
#include "../core/CalibrationWizard.h"
#include "../core/Types.h"
#include <QButtonGroup>
#include <QGroupBox>
#include <QLabel>
#include <QListWidget>
#include <QMap>
#include <QProgressBar>
#include <QPushButton>
#include <QRadioButton>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QWidget>

class CalibrationWidget : public QWidget {
  Q_OBJECT

public:
  explicit CalibrationWidget(QWidget *parent = nullptr);
  void setProtocol(DroneProtocol *protocol);

public slots:
  // Enable / disable the calibration action surface based on the
  // current serial connection. When disconnected, sensor cards +
  // Start / Cancel are disabled and the footer reads "NOT CONNECTED"
  // so the user doesn't trigger a no-op (FR-UX-18 / Phase-0 0o).
  void setConnected(bool connected);

signals:
  void backToHomeRequested();
  void commandRequested(const QByteArray &data);

private slots:
  void onStartClicked();
  void onCancelClicked();
  void onStatusReceived(const QString &msg);
  void onSensorSelected(int id);
  void onProgressReceived(float pct);
  void onInstructionReceived(int type);

private:
  bool m_connected = false;

private:
  DroneProtocol *m_protocol = nullptr;

  // Sensor Selection Cards
  QWidget *m_sensorSelectArea;
  QPushButton *m_accBtn;
  QPushButton *m_gyrBtn;
  QPushButton *m_magBtn;
  int m_selectedImuId = 2; // Default to Gyro

  // Configuration Area
  QGroupBox *m_configGroup;
  QButtonGroup *m_typeGroup;
  QRadioButton *m_biasOnlyRadio;
  QRadioButton *m_fullCalibRadio;

  // Interactive Instruction Panel
  QGroupBox *m_instructionGroup;
  QLabel *m_instructionText;
  QProgressBar *m_progressBar;

  // Axis Status (for 6-axis)
  QWidget *m_axisStatusArea;
  QLabel *m_axisLabelX;
  QLabel *m_axisLabelY;
  QLabel *m_axisLabelZ;
  QLabel *m_axisLabelNX;
  QLabel *m_axisLabelNY;
  QLabel *m_axisLabelNZ;

  QMap<CalibUpdateType, QLabel *> m_axisMap;
  CalibUpdateType m_currentAxis = CalibUpdateType::Progress;

  // Gated step wizard: the guided checklist driven by firmware instructions.
  CalibrationWizard m_wizard;
  QListWidget *m_stepList = nullptr;
  CalibMode currentMode() const;
  void refreshSteps();

  // Main Controls
  QPushButton *m_startBtn;
  QPushButton *m_cancelBtn;
  QLabel *m_statusLabel;

  void sendCalibrationCommand(int imu_id, int type);
};
