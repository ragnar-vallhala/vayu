#pragma once

#include <QComboBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QTimer>

#include "AttitudeWidget.h"
#include "DroneProtocol.h"
#include "ImuPanel.h"
#include "LogPanel.h"
#include "SerialManager.h"
#include "Types.h"

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow() override = default;

private slots:
  // Toolbar actions
  void onConnectClicked();
  void onRefreshPorts();
  void onArmClicked();

  // Data callbacks
  void onImuReceived(const ImuData &data);
  void onAttitudeReceived(const AttitudeData &data);
  void onLogReceived(const QString &msg);

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

  // ---- Attitude numeric labels ----
  QLabel *m_rollLabel = nullptr;
  QLabel *m_pitchLabel = nullptr;
  QLabel *m_yawLabel = nullptr;

  // ---- Status bar ----
  QLabel *m_connStatus = nullptr;
  QLabel *m_pktStatus = nullptr;

  // ---- Back-end ----
  SerialManager *m_serial = nullptr;
  DroneProtocol *m_protocol = nullptr;
  QTimer *m_uiTimer = nullptr;
  QElapsedTimer m_elapsed;

  // ---- State ----
  bool m_connected = false;
  bool m_armed = false;
  int m_pktCount = 0;
  ImuData m_latestImu;
  AttitudeData m_latestAtt;
  qint64 m_lastHbTime = 0;
};
