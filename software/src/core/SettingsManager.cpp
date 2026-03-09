#include "SettingsManager.h"

const QString SettingsManager::FileName = "vayu_settings.dat";

bool SettingsManager::save(const GcsSettings &s) {
  QFile file(FileName);
  if (!file.open(QIODevice::WriteOnly)) {
    return false;
  }

  QDataStream out(&file);
  out << Magic;
  out << Version;
  out << s;

  return true;
}

bool SettingsManager::load(GcsSettings &s) {
  QFile file(FileName);
  if (!file.exists() || !file.open(QIODevice::ReadOnly)) {
    return false;
  }

  QDataStream in(&file);
  quint32 magic;
  int version;
  in >> magic;
  in >> version;

  if (magic != Magic || version > Version) {
    return false;
  }

  in >> s;
  return true;
}
