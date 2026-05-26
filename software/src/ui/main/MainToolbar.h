#pragma once

#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QToolBar>

// Top-of-window toolbar for Navigator. Owns the port + baud combos,
// connect / arm buttons, page-jump shortcuts (RC / CALIB), the 3D-view
// toggle, and the LIVE heartbeat blinker. Stays dumb about app state:
// emits intent signals (connectRequested / showRc / …) and exposes
// setters MainWindow can drive (`setConnected`, `setLiveActive`).
//
// Lives in its own translation unit so MainWindow doesn't have to
// know about the styling and event wiring of every toolbar widget.
//
// LIVE blinker: the toolbar provides a stable `liveLabel()` handle so
// MainStatusBar / MainWindow can repaint the per-frame fade without
// the toolbar having to know what a heartbeat is.
class MainToolbar : public QToolBar {
  Q_OBJECT

public:
  explicit MainToolbar(QWidget *parent = nullptr);

  // ---- State updaters driven by MainWindow ---------------------------------
  void setConnected(bool on, const QString &portLabel = {});

  // ARM stays disabled until FR-TX-02 lands; MainWindow flips this
  // once the firmware-side packet is defined and the connection is up.
  void setArmEnabled(bool on);

  // ---- Combo accessors for QSettings round-trip (persistence) --------------
  // currentPort returns the bare device path (userData if present, edit
  // text otherwise). currentBaud is the numeric value from userData.
  QString currentPort() const;
  int     currentBaud() const;
  void    setPort(const QString &path);
  void    setBaud(int baud);

  // ---- LIVE blinker handle for heartbeat fade ------------------------------
  // The MainStatusBar / heartbeat handler repaints this label every
  // frame; the toolbar just owns the widget so layout stays consistent.
  QLabel *liveLabel() const { return m_liveLabel; }

public slots:
  void refreshPorts();
  // Public so the Ctrl+K shortcut in MainWindow can fire the same
  // intent as a physical click on the Connect button.
  void onConnectClicked();

signals:
  // Intent-only — MainWindow decides what to do (open serial, log, etc.).
  void connectRequested(const QString &port, int baud);
  void disconnectRequested();
  void armClicked();
  void showRcRequested();
  void showCalibRequested();

private:
  void buildContent();

  QComboBox   *m_portCombo  = nullptr;
  QComboBox   *m_baudCombo  = nullptr;
  QPushButton *m_connectBtn = nullptr;
  QPushButton *m_armBtn     = nullptr;
  QLabel      *m_liveLabel  = nullptr;

  bool m_connected = false;
};
