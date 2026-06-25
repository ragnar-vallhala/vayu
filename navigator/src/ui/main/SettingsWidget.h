#pragma once

#include "../../core/SettingsManager.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QSet>
#include <QSpinBox>
#include <QWidget>

class QLabel;
class QStackedWidget;

class SettingsWidget : public QWidget {
  Q_OBJECT

public:
  explicit SettingsWidget(QWidget *parent = nullptr);

  void setSettings(const GcsSettings &s);
  GcsSettings getSettings() const;

signals:
  void backToHomeRequested();
  // Emitted when the user clicks Apply — MainWindow reads getSettings(),
  // applies everything, and persists. Edits before Apply are staged only.
  void applyRequested();

private:
  void markDirty(QLabel *label);        // yellow the row + enable Apply
  void clearDirty();                    // reset after Apply / programmatic load
  void updateTransportDependent();      // UDP ⇒ numeric port + locked baud
  // Advanced ▸ Settings file / Reset — immediate actions (not staged).
  void importSettings();
  void exportSettings();
  void resetToDefaults();

  QSpinBox *m_syncPeriodSpin = nullptr;
  QSpinBox *m_graphWindowSpin = nullptr;
  QDoubleSpinBox *m_graphDropoutSpin = nullptr;
  // Plots & Graphs appearance.
  QCheckBox *m_sigmaChk = nullptr;
  QCheckBox *m_stateBandChk = nullptr;
  QDoubleSpinBox *m_traceWidthSpin = nullptr;
  QCheckBox *m_antialiasChk = nullptr;
  QCheckBox *m_autoReconnectChk = nullptr;
  QCheckBox *m_recordOnConnectChk = nullptr;
  // Logging & Recording.
  class QLineEdit *m_logDirEdit = nullptr;
  QComboBox *m_timestampCombo = nullptr;
  QSpinBox *m_maxLogLinesSpin = nullptr;
  QCheckBox *m_exportOnDisconnectChk = nullptr;
  // Alerts & Audio.
  QCheckBox *m_toastChk = nullptr;
  QCheckBox *m_audioAlertsChk = nullptr;
  QCheckBox *m_confirmArmChk = nullptr;
  QCheckBox *m_simPropAudioChk = nullptr;
  // Advanced.
  QSpinBox *m_packetBufferSpin = nullptr;
  QCheckBox *m_crcCheckChk = nullptr;
  QSpinBox *m_recentViewsSpin = nullptr;
  QComboBox *m_themeCombo = nullptr;
  // Units & Display.
  QComboBox *m_unitSystemCombo = nullptr;  // Metric / Imperial (preset)
  QComboBox *m_angleCombo = nullptr;       // deg / rad
  QComboBox *m_altCombo = nullptr;         // metres / feet
  QComboBox *m_speedCombo = nullptr;       // m/s / km/h / mph
  QSpinBox *m_decimalsSpin = nullptr;      // readout precision
  QComboBox *m_startupCombo = nullptr;     // last viewed / dashboard / sim
  QCheckBox *m_restoreLayoutChk = nullptr;
  // Link & Connection defaults.
  QComboBox *m_transportCombo = nullptr;
  QComboBox *m_portCombo = nullptr;       // serial port (transport = Serial)
  QSpinBox *m_udpPortSpin = nullptr;      // UDP port number (transport = UDP)
  QStackedWidget *m_portStack = nullptr;  // swaps the two above
  QComboBox *m_baudCombo = nullptr;
  QWidget *m_baudRow = nullptr;           // row container (tooltip when locked)
  QDoubleSpinBox *m_reconnectSpin = nullptr;
  QSpinBox *m_linkLossSpin = nullptr;
  // Apply / dirty state.
  QPushButton *m_applyBtn = nullptr;
  QLabel *m_dirtyHint = nullptr;
  QSet<QLabel *> m_dirtyLabels;
};
