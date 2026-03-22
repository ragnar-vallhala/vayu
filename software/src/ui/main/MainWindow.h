#pragma once

#include <QComboBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QStackedWidget>
#include <QTimer>

#include "../widgets/CalibrationWidget.h"
#include "AttitudeWidget.h"
#include "Drone3DWidget.h" // Added
#include "DroneProtocol.h"
#include "ImuPanel.h"
#include "LogPanel.h"
#include "MotorStatusWidget.h"
#include "PacketAnalyzerWidget.h"
#include "RcChannelsWidget.h"
#include "RollingStats.h"
#include "SerialManager.h"
#include "SettingsWidget.h"
#include "Types.h"

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow() override = default;

private slots:
  // Navigation
  void showHome();
  void showPacketAnalyzer();
  void showRcMonitor();
  void showSettings();
  void showCalibration();
  void showMotorStatus();
  // Toolbar actions
  void onConnectClicked();
  void onRefreshPorts();
  void onArmClicked();
  void onToggle3d(bool checked);

  // Data callbacks
  void onImuReceived(const ImuData &data);
  void onAttitudeReceived(const AttitudeData &data);
  void onRcReceived(const RcData &data);
  void onLogReceived(const QString &msg);
  void onStatusReceived(const QString &msg);

  // Serial state
  void onConnectionStateChanged(bool connected);
  void onSerialError(const QString &msg);

  // Periodic UI refresh
  void onUiTimer();

  // New data slots
  void onHeartbeatReceived(uint64_t timestamp, uint8_t deviceId);
  void onTimeSyncRequested();

private:
  void buildUi();
  void buildMenuBar();
  void buildToolBar();
  void applyDarkTheme();
  void setConnected(bool on);

  // ---- Toolbar widgets ----
  QComboBox *m_portCombo = nullptr;
  QComboBox *m_baudCombo = nullptr;
  QPushButton *m_connectBtn = nullptr;
  QPushButton *m_armBtn = nullptr;
  QLabel *m_liveLabel = nullptr;

  // ---- Central panels ----
  ImuPanel *m_imuPanel = nullptr;
  AttitudeWidget *m_attitude = nullptr;
  LogPanel *m_logPanel = nullptr;
  PacketAnalyzerWidget *m_analyzerWidget = nullptr;
  RcChannelsWidget *m_rcWidget = nullptr;
  SettingsWidget *m_settingsWidget = nullptr;
  CalibrationWidget *m_calibrationWidget = nullptr;
  MotorStatusWidget *m_motorWidget = nullptr;
  QStackedWidget *m_stackedWidget = nullptr;
  QStackedWidget *m_attStack = nullptr;
  Drone3DWidget *m_drone3d = nullptr;
  QWidget *m_homeWidget = nullptr;

  QAction *m_homeAction = nullptr;
  QAction *m_analyzerAction = nullptr;

  // ---- Attitude numeric labels ----
  QLabel *m_rollLabel = nullptr;
  QLabel *m_pitchLabel = nullptr;
  QLabel *m_yawLabel = nullptr;
  QLabel *m_rollStd = nullptr;
  QLabel *m_pitchStd = nullptr;
  QLabel *m_yawStd = nullptr;
  RollingStats m_attStats[3];

  // ---- Status bar ----
  QLabel *m_connStatus = nullptr;
  QLabel *m_pktStatus = nullptr;
  QLabel *m_syncStatus = nullptr;
  QLabel *m_statusLabel = nullptr;

  // ---- Back-end ----
  SerialManager *m_serial = nullptr;
  DroneProtocol *m_protocol = nullptr;
  QTimer *m_uiTimer = nullptr;
  QTimer *m_syncTimer = nullptr;
  QElapsedTimer m_elapsed;

  // ---- State ----
  bool m_connected = false;
  bool m_armed = false;
  int m_pktCount = 0;
  ImuData m_latestImu;
  AttitudeData m_latestAtt;
  qint64 m_lastHbTime = 0;
};
