#include "Logger.h"
#include "MainWindow.h"
#include "Theme.h"
#include <QApplication>
#include <QSurfaceFormat>

int main(int argc, char *argv[]) {
  // Request OpenGL Core profile for the attitude widget
  QSurfaceFormat fmt;
  fmt.setVersion(3, 3);
  fmt.setProfile(QSurfaceFormat::CoreProfile);
  fmt.setSamples(4); // MSAA
  QSurfaceFormat::setDefaultFormat(fmt);

  QApplication app(argc, argv);
  app.setApplicationName("Vayu GCS");
  app.setApplicationVersion("1.0.0");
  app.setOrganizationName("Vayu");

  // Apply the single source-of-truth dark theme + palette. Every
  // per-widget styling decision should reference Theme.h or the qss
  // bundled in resources/styles/dark.qss; avoid inline setStyleSheet.
  Theme::apply();

  // Bring the persistent file logger up before any widget is
  // constructed so the LogPanel's initial messages get teed to disk.
  // Defaults are sensible (AppDataLocation/logs, 5 MB rotation,
  // 10 files retained); override via Logger::init({...}) if needed.
  Logger::init();

  MainWindow w;
  w.show();

  const int rc = app.exec();
  Logger::shutdown();
  return rc;
}
