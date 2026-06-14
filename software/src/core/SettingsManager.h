#pragma once

#include <QDataStream>
#include <QFile>
#include <QString>

struct GcsSettings {
  int syncPeriodMs = 5000;
  int graphWindowSec = 5;
  double graphDropoutRate = 0.0;
  bool autoReconnect = false;
  bool recordOnConnect = false;  // tee telemetry to a .bin on connect (1C)
  int recentViewsCount = 5;      // MRU switcher depth (2C); clamped on apply
  int theme = 0;                 // 0=Dark (Navigator), 1=Midnight, 2=High-Contrast

  // Link & Connection defaults (mockup parity, wired 2026-06-14).
  int defaultTransport = 0;          // 0 = Serial, 1 = UDP
  QString defaultPort;               // empty = "Ask each time" (keep last-used)
  int defaultBaud = 115200;          // applied to the toolbar on launch
  double reconnectIntervalSec = 1.0; // base auto-reconnect retry delay
  int linkLossTimeoutMs = 1500;      // no-heartbeat → flag the pill disconnected

  // Units & Display (mockup parity, wired 2026-06-14). Unit choices flow into
  // the Units:: helper that the live readouts pull from; startupPage and
  // restoreLayout drive MainWindow::restoreUiState().
  int unitSystem = 0;    // 0 = Metric, 1 = Imperial (preset for alt + speed)
  int angleUnit = 0;     // 0 = degrees, 1 = radians
  int altitudeUnit = 0;  // 0 = metres, 1 = feet
  int speedUnit = 0;     // 0 = m/s, 1 = km/h, 2 = mph
  int decimals = 2;      // readout precision (clamped 0..6 on apply)
  int startupPage = 0;   // 0 = last viewed, 1 = Flight Dashboard, 2 = Simulator
  bool restoreLayout = true;  // remember window geometry, splitters + last page

  // Plots & Graphs appearance (mockup parity, wired 2026-06-14).
  bool sigmaTraces = true;   // overlay rolling-σ traces on the IMU graphs
  bool stateBand = false;    // colour the plot background by FC state
  double traceWidth = 1.4;   // trace pen width
  bool antialias = false;    // antialiased trace painting

  // Logging & Recording (mockup parity, wired 2026-06-14).
  QString logDirectory;      // recording dir; empty = ~/vayu-logs default
  int timestampMode = 0;     // System Log: 0 = Local, 1 = UTC, 2 = Elapsed (T+)
  int maxLogLines = 2000;    // System Log ring-buffer cap
  bool exportOnDisconnect = false;  // dump the session log on disconnect

  // Alerts & Audio (mockup parity, wired 2026-06-14). Battery warn/crit are not
  // here yet — no battery voltage in telemetry, so they stay coming-soon.
  bool toastNotifications = true;  // in-app status-bar toasts (Notify)
  bool audioAlerts = true;         // arm/disarm/failsafe chimes (ChimeAudio)
  bool confirmBeforeArm = true;    // require a confirm dialog before ARM
  bool simPropAudio = false;       // default state of the sim's prop-audio

  // Versioned (de)serialisation — the leading version field lets us
  // append fields without breaking existing settings files. The trailing
  // atEnd() sentinels make appended fields backward-compatible without a
  // Version bump: autoReconnect, recordOnConnect, recentViewsCount, theme,
  // the Link & Connection defaults, and the Units & Display block.
  friend QDataStream &operator<<(QDataStream &out, const GcsSettings &s) {
    out << s.syncPeriodMs << s.graphWindowSec << s.graphDropoutRate
        << s.autoReconnect << s.recordOnConnect << s.recentViewsCount
        << s.theme << s.defaultTransport << s.defaultPort << s.defaultBaud
        << s.reconnectIntervalSec << s.linkLossTimeoutMs << s.unitSystem
        << s.angleUnit << s.altitudeUnit << s.speedUnit << s.decimals
        << s.startupPage << s.restoreLayout << s.sigmaTraces << s.stateBand
        << s.traceWidth << s.antialias << s.logDirectory
        << s.timestampMode << s.maxLogLines << s.exportOnDisconnect
        << s.toastNotifications << s.audioAlerts << s.confirmBeforeArm
        << s.simPropAudio;
    return out;
  }

  friend QDataStream &operator>>(QDataStream &in, GcsSettings &s) {
    in >> s.syncPeriodMs >> s.graphWindowSec >> s.graphDropoutRate;
    if (!in.atEnd()) in >> s.autoReconnect;
    if (!in.atEnd()) in >> s.recordOnConnect;
    if (!in.atEnd()) in >> s.recentViewsCount;
    if (!in.atEnd()) in >> s.theme;
    if (!in.atEnd()) in >> s.defaultTransport;
    if (!in.atEnd()) in >> s.defaultPort;
    if (!in.atEnd()) in >> s.defaultBaud;
    if (!in.atEnd()) in >> s.reconnectIntervalSec;
    if (!in.atEnd()) in >> s.linkLossTimeoutMs;
    if (!in.atEnd()) in >> s.unitSystem;
    if (!in.atEnd()) in >> s.angleUnit;
    if (!in.atEnd()) in >> s.altitudeUnit;
    if (!in.atEnd()) in >> s.speedUnit;
    if (!in.atEnd()) in >> s.decimals;
    if (!in.atEnd()) in >> s.startupPage;
    if (!in.atEnd()) in >> s.restoreLayout;
    if (!in.atEnd()) in >> s.sigmaTraces;
    if (!in.atEnd()) in >> s.stateBand;
    if (!in.atEnd()) in >> s.traceWidth;
    if (!in.atEnd()) in >> s.antialias;
    if (!in.atEnd()) in >> s.logDirectory;
    if (!in.atEnd()) in >> s.timestampMode;
    if (!in.atEnd()) in >> s.maxLogLines;
    if (!in.atEnd()) in >> s.exportOnDisconnect;
    if (!in.atEnd()) in >> s.toastNotifications;
    if (!in.atEnd()) in >> s.audioAlerts;
    if (!in.atEnd()) in >> s.confirmBeforeArm;
    if (!in.atEnd()) in >> s.simPropAudio;
    return in;
  }
};

class SettingsManager {
public:
  static bool save(const GcsSettings &s);
  static bool load(GcsSettings &s);

private:
  static const quint32 Magic = 0x56415955; // "VAYU"
  // Bump when struct layout changes in a backwards-incompatible way.
  // Appending optional fields with sentinels (see operator>>) does NOT
  // require a bump.
  static const int Version = 2;
  static const QString FileName;
};
