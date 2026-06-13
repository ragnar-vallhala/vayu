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

  // Versioned (de)serialisation — the leading version field lets us
  // append fields without breaking existing settings files. The trailing
  // atEnd() sentinels make appended fields backward-compatible without a
  // Version bump: autoReconnect, then recordOnConnect.
  friend QDataStream &operator<<(QDataStream &out, const GcsSettings &s) {
    out << s.syncPeriodMs << s.graphWindowSec << s.graphDropoutRate
        << s.autoReconnect << s.recordOnConnect;
    return out;
  }

  friend QDataStream &operator>>(QDataStream &in, GcsSettings &s) {
    in >> s.syncPeriodMs >> s.graphWindowSec >> s.graphDropoutRate;
    if (!in.atEnd()) in >> s.autoReconnect;
    if (!in.atEnd()) in >> s.recordOnConnect;
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
