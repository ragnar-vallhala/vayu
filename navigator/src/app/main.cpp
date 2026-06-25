#include "Logger.h"
#include "MainWindow.h"
#include "Theme.h"
#include "WheelGuard.h"
#include <QApplication>
#include <QSurfaceFormat>

int main(int argc, char *argv[]) {
  // Request OpenGL 4.6 Core (with a graceful drop to whatever the driver gives).
  // 4.3+ unlocks compute shaders + indirect draw, which the GPU grass uses; the
  // rest of the renderer is 3.3-core code that runs unchanged on a 4.x core
  // context. On a driver that can't give 4.3 the grass falls back to the CPU path.
  QSurfaceFormat fmt;
  fmt.setVersion(4, 6);
  fmt.setProfile(QSurfaceFormat::CoreProfile);
  fmt.setSamples(4); // MSAA
  QSurfaceFormat::setDefaultFormat(fmt);

  QApplication app(argc, argv);
  app.setApplicationName("Vayu GCS");
  app.setApplicationVersion("1.0.0");
  app.setOrganizationName("Vayu");

  // Use Qt's own file dialogs, never the platform/portal native ones: the native
  // GTK/xdg-desktop-portal chooser hangs the event loop on this setup (every
  // Open/Save/Export froze the app). Qt's widget dialog runs in our own loop and
  // works. Applies to every QFileDialog/QColorDialog/QFontDialog app-wide.
  QApplication::setAttribute(Qt::AA_DontUseNativeDialogs);

  // Stop the mouse wheel from silently editing spin boxes / combos while the
  // user is just scrolling a panel. Values change via the up/down buttons or
  // typing instead. (Filter is owned by `app` and lives for the whole run.)
  app.installEventFilter(new ui::WheelGuard(&app));

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
