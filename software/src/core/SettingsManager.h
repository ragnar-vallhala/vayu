#pragma once

#include <QDataStream>
#include <QFile>
#include <QString>

struct GcsSettings {
  int syncPeriodMs = 5000;
  int graphWindowSec = 5;
  double graphDropoutRate = 0.0;
  bool autoReconnect = false;

  // Versioned (de)serialisation — the leading version field lets us
  // append fields without breaking existing settings files. v1 is the
  // original three-field layout; v2 adds autoReconnect.
  friend QDataStream &operator<<(QDataStream &out, const GcsSettings &s) {
    out << s.syncPeriodMs << s.graphWindowSec << s.graphDropoutRate
        << s.autoReconnect;
    return out;
  }

  friend QDataStream &operator>>(QDataStream &in, GcsSettings &s) {
    in >> s.syncPeriodMs >> s.graphWindowSec >> s.graphDropoutRate;
    if (!in.atEnd()) in >> s.autoReconnect;
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
