#include "SettingsManager.h"

const QString SettingsManager::FileName = "vayu_settings.dat";

bool SettingsManager::saveToPath(const QString &path, const GcsSettings &s) {
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    return false;
  }

  QDataStream out(&file);
  out << Magic;
  out << Version;
  out << s;

  return true;
}

bool SettingsManager::loadFromPath(const QString &path, GcsSettings &s) {
  QFile file(path);
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

bool SettingsManager::save(const GcsSettings &s) { return saveToPath(FileName, s); }

bool SettingsManager::load(GcsSettings &s) { return loadFromPath(FileName, s); }
