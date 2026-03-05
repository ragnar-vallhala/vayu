#include "MainWindow.h"
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

  MainWindow w;
  w.show();

  return app.exec();
}
