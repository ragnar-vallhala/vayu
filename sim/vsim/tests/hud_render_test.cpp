// Headless render test for SimHudWidget. Renders the FPV HUD to an
// offscreen image (QT_QPA_PLATFORM=offscreen) with a sample snapshot and
// asserts it actually painted HUD geometry (greenish, non-transparent
// pixels). Verifies the paintEvent runs and produces output without a
// display — the Wayland session blocks X screenshots.
//
// Exit 0 on pass.

#include "SimHudWidget.h"

#include <QApplication>
#include <QImage>

#include <cstdio>

int main(int argc, char** argv) {
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);

  SimHudWidget hud;
  hud.resize(800, 600);

  vsim::SimSnapshot s;
  s.pos_w = vsim::Vec3(1.0f, 2.0f, -5.0f);             // altitude 5 m
  s.att = vsim::Quat::fromEulerAngles(8.0f, 35.0f, 12.0f);  // pitch,yaw,roll
  s.vel_w = vsim::Vec3(3.0f, 1.0f, -1.0f);
  s.motor_duty = {0.5f, 0.6f, 0.7f, 0.85f};
  hud.setSnapshot(s);

  QImage img(hud.size(), QImage::Format_ARGB32);
  img.fill(Qt::transparent);
  hud.render(&img);

  int green = 0;
  for (int y = 0; y < img.height(); y += 2) {
    for (int x = 0; x < img.width(); x += 2) {
      const QColor c = img.pixelColor(x, y);
      if (c.alpha() > 20 && c.green() > 140 && c.green() > c.red() + 20 &&
          c.green() > c.blue() + 20)
        ++green;
    }
  }

  std::printf("HUD green pixels (sampled): %d\n", green);
  const bool ok = green > 50;
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
