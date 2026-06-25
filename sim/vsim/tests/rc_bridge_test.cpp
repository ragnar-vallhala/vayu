// End-to-end check for RcBridge: spin it against the live joystick, open
// the pty slave it created, and read back the CSV RC frames the firmware's
// host_rc_feeder would see. Prints the last frame (sticks at rest →
// throttle ~1000). Exit 0 if well-formed CSV frames arrive.

#include "RcBridge.h"

#include <QCoreApplication>
#include <QTimer>

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

int main(int argc, char** argv) {
  QCoreApplication app(argc, argv);
  RcBridge rc;
  if (argc > 1) rc.setJoystickPath(argv[1]);

  QString err;
  if (!rc.openPty(&err)) {
    std::printf("FAIL openPty: %s\n", err.toLocal8Bit().constData());
    return 2;
  }
  std::printf("pty slave: %s\n", rc.slavePath().toLocal8Bit().constData());
  const int sfd = ::open(rc.slavePath().toLocal8Bit().constData(),
                         O_RDONLY | O_NONBLOCK);

  QObject::connect(&rc, &RcBridge::logLine, &app, [](const QString& s) {
    std::printf("[rc] %s\n", s.toLocal8Bit().constData());
  });
  rc.start();

  QTimer::singleShot(700, &app, [&] {
    int frames = 0;
    char lastLine[128] = {0};
    char buf[4096];
    for (int i = 0; i < 25; ++i) {     // read across the window (tty is line-buffered)
      const ssize_t n = (sfd >= 0) ? ::read(sfd, buf, sizeof(buf) - 1) : -1;
      if (n > 0) {
        buf[n] = 0;
        for (char* s = buf; (s = std::strchr(s, '\n')); ++s) ++frames;
        char* p = std::strrchr(buf, '\n');
        if (p && p != buf) { *p = 0; char* q = std::strrchr(buf, '\n');
          std::snprintf(lastLine, sizeof(lastLine), "%s", q ? q + 1 : buf); }
      }
      ::usleep(20000);
    }
    rc.requestStop();
    rc.wait(500);
    std::printf("frames=%d  last=\"%s\"\n", frames, lastLine);
    std::printf("%s\n", frames > 5 ? "PASS" : "FAIL");
    app.exit(frames > 5 ? 0 : 1);
  });
  return app.exec();
}
