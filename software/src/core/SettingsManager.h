#pragma once

#include <QDataStream>
#include <QFile>
#include <QString>

struct GcsSettings {
  int syncPeriodMs = 5000;
  int graphWindowSec = 5;
  double graphDropoutRate = 0.0;

  friend QDataStream &operator<<(QDataStream &out, const GcsSettings &s) {
    out << s.syncPeriodMs << s.graphWindowSec << s.graphDropoutRate;
    return out;
  }

  friend QDataStream &operator>>(QDataStream &in, GcsSettings &s) {
    in >> s.syncPeriodMs >> s.graphWindowSec >> s.graphDropoutRate;
    return in;
  }
};

class SettingsManager {
public:
  static bool save(const GcsSettings &s);
  static bool load(GcsSettings &s);

private:
  static const quint32 Magic = 0x56415955; // "VAYU"
  static const int Version = 1;
  static const QString FileName;
};
