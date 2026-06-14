#include "Drone3DWidget.h"

#include <QMatrix4x4>
#include <QPainter>
#include <QPolygonF>
#include <QRadialGradient>
#include <QTimer>
#include <QVector3D>
#include <QtMath>
#include <algorithm>
#include <array>

namespace {

// Quad geometry in body units. Front = +Y, right = +X, up = +Z. The four
// motors sit on the X-frame diagonals; front pair is red, rear pair green.
struct Motor {
  float x, y;
  bool front;
};
constexpr float kArmReach = 0.86f;  // motor offset on each axis
const std::array<Motor, 4> kMotors = {{
    {+kArmReach, +kArmReach, true},   // front-right
    {-kArmReach, +kArmReach, true},   // front-left
    {+kArmReach, -kArmReach, false},  // rear-right
    {-kArmReach, -kArmReach, false},  // rear-left
}};

constexpr float kViewTilt = 60.0f;  // deg; look down on the top, like .scene

const QColor kArmEdge(0x3e, 0x44, 0x52);
const QColor kArmFill(0x2c, 0x31, 0x3a);
const QColor kHub(0x3a, 0x41, 0x4e);
const QColor kHubEdge(0x4a, 0x52, 0x5f);
const QColor kFront(0xE0, 0x6C, 0x75);   // --danger (front pods)
const QColor kRear(0x98, 0xC3, 0x79);    // --ok (rear pods)
const QColor kAccent(0x61, 0xAF, 0xEF);  // --accent (nose)
const QColor kPodFill(0x20, 0x24, 0x2e);
const QColor kBlade(190, 205, 225, 95);

}  // namespace

Drone3DWidget::Drone3DWidget(QWidget *parent) : QOpenGLWidget(parent) {
  setMinimumSize(260, 260);
  // Gentle prop spin so the airframe reads as "live", as in the mockup.
  m_spinTimer = new QTimer(this);
  m_spinTimer->setInterval(33);  // ~30 fps
  connect(m_spinTimer, &QTimer::timeout, this, [this] {
    m_propPhase += 9.0f;
    update();
  });
}

void Drone3DWidget::setAttitude(const AttitudeData &att) {
  setAttitude(att.roll, att.pitch, att.yaw);
}

void Drone3DWidget::setAttitude(float roll, float pitch, float yaw) {
  // Cache only — the ~30 Hz spin timer already repaints while the view is shown,
  // so the new attitude is picked up on the next frame without forcing an extra
  // GL repaint per call. See docs/ui-rendering-decoupling.md.
  m_roll = roll;
  m_pitch = pitch;
  m_yaw = yaw;
}

void Drone3DWidget::showEvent(QShowEvent *event) {
  QOpenGLWidget::showEvent(event);
  m_spinTimer->start();
}

void Drone3DWidget::hideEvent(QHideEvent *event) {
  QOpenGLWidget::hideEvent(event);
  m_spinTimer->stop();  // don't burn CPU while the 3D view is hidden
}

void Drone3DWidget::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  const int w = width();
  const int h = height();

  // Background: radial vignette matching the mockup's .adi3d.
  QRadialGradient bg(w * 0.5, h * 0.30, qMax(w, h) * 0.78);
  bg.setColorAt(0.0, QColor(0x1c, 0x25, 0x33));
  bg.setColorAt(1.0, QColor(0x0c, 0x0f, 0x16));
  p.fillRect(rect(), bg);

  const QPointF center(w * 0.5, h * 0.50);
  const float scale = qMin(w, h) * 0.30f;

  // Orientation: live attitude, then a fixed view tilt that looks down on the
  // top of the airframe (the mockup's `scene { rotateX(58deg) }`).
  QMatrix4x4 R;
  R.rotate(-kViewTilt, 1, 0, 0);  // view tilt about screen X (look down)
  R.rotate(m_yaw, 0, 0, 1);       // yaw about up (+Z)
  R.rotate(m_roll, 0, 1, 0);      // roll about forward (+Y)
  R.rotate(m_pitch, 1, 0, 0);     // pitch about right (+X)

  // Orthographic projection (mild, like the mockup's large perspective): map a
  // body point to screen and report its camera-depth for painter ordering.
  auto project = [&](float x, float y, float z, float *depth = nullptr) {
    const QVector3D r = R.map(QVector3D(x, y, z));
    if (depth) *depth = r.z();
    return center + QPointF(r.x() * scale, -r.y() * scale);
  };

  // A disc lying flat in the body's rotor plane (normal = +Z), sampled and
  // projected to a polygon so it foreshortens into an ellipse that banks and
  // yaws with the airframe instead of always facing the camera.
  auto projectedDisc = [&](float cx, float cy, float cz, float r) {
    constexpr int kSeg = 32;
    QPolygonF poly;
    for (int i = 0; i < kSeg; ++i) {
      const float a = qDegreesToRadians(360.0f * i / kSeg);
      poly << project(cx + r * std::cos(a), cy + r * std::sin(a), cz);
    }
    return poly;
  };

  // --- ground shadow (static, on the deck below the drone) ---
  {
    const QPointF sc(center.x(), center.y() + scale * 0.62);
    QRadialGradient sh(sc, scale);
    sh.setColorAt(0.0, QColor(0, 0, 0, 150));
    sh.setColorAt(0.75, QColor(0, 0, 0, 0));
    p.setBrush(sh);
    p.setPen(Qt::NoPen);
    p.drawEllipse(sc, scale * 0.95, scale * 0.30);
  }

  // --- arms: two crossed bars through the centre ---
  p.setPen(QPen(kArmFill, qMax(2.0f, scale * 0.10f), Qt::SolidLine, Qt::RoundCap));
  p.drawLine(project(kMotors[1].x, kMotors[1].y, 0),
             project(kMotors[2].x, kMotors[2].y, 0));  // FL — RR
  p.drawLine(project(kMotors[0].x, kMotors[0].y, 0),
             project(kMotors[3].x, kMotors[3].y, 0));  // FR — RL
  // thin highlight edge
  p.setPen(QPen(kArmEdge, qMax(1.0f, scale * 0.03f), Qt::SolidLine, Qt::RoundCap));
  p.drawLine(project(kMotors[1].x, kMotors[1].y, 0.02f),
             project(kMotors[2].x, kMotors[2].y, 0.02f));
  p.drawLine(project(kMotors[0].x, kMotors[0].y, 0.02f),
             project(kMotors[3].x, kMotors[3].y, 0.02f));

  // --- motors: pods + spinning props, drawn far-to-near ---
  struct Drawn {
    int i;
    float depth;
  };
  std::array<Drawn, 4> order;
  for (int i = 0; i < 4; ++i) {
    float d;
    project(kMotors[i].x, kMotors[i].y, 0.0f, &d);
    order[i] = {i, d};
  }
  std::sort(order.begin(), order.end(),
            [](const Drawn &a, const Drawn &b) { return a.depth < b.depth; });

  const float podR = 0.24f;    // body units (motor housing radius)
  const float bladeR = 0.30f;  // body units
  for (const Drawn &o : order) {
    const Motor &m = kMotors[o.i];
    const QColor ring = m.front ? kFront : kRear;

    // pod (motor housing) — a foreshortened disc in the rotor plane
    p.setBrush(kPodFill);
    p.setPen(QPen(ring, qMax(1.5f, scale * 0.028f)));
    p.drawPolygon(projectedDisc(m.x, m.y, 0.05f, podR));

    // prop blades in the rotor plane just above the pod (so they foreshorten
    // and yaw with the airframe), two bars crossed at 90°.
    p.setPen(QPen(kBlade, qMax(2.0f, scale * 0.03f), Qt::SolidLine, Qt::RoundCap));
    for (int k = 0; k < 2; ++k) {
      const float a = qDegreesToRadians(m_propPhase + o.i * 35.0f + k * 90.0f);
      const float dx = std::cos(a) * bladeR;
      const float dy = std::sin(a) * bladeR;
      p.drawLine(project(m.x - dx, m.y - dy, 0.22f),
                 project(m.x + dx, m.y + dy, 0.22f));
    }
  }

  // --- central hub + nose heading marker (on top) ---
  {
    QPolygonF hub;
    hub << project(-0.22f, +0.18f, 0.13f) << project(+0.22f, +0.18f, 0.13f)
        << project(+0.22f, -0.18f, 0.13f) << project(-0.22f, -0.18f, 0.13f);
    p.setBrush(kHub);
    p.setPen(QPen(kHubEdge, qMax(1.5f, scale * 0.022f)));
    p.drawConvexPolygon(hub);

    QPolygonF nose;
    nose << project(0.0f, +0.40f, 0.14f) << project(-0.11f, +0.17f, 0.14f)
         << project(+0.11f, +0.17f, 0.14f);
    p.setBrush(kAccent);
    p.setPen(Qt::NoPen);
    p.drawConvexPolygon(nose);
  }

  // --- legend, like the mockup's .view-hint ---
  p.setPen(QColor(0x7d, 0x86, 0x94));
  QFont f = p.font();
  f.setPixelSize(10);
  p.setFont(f);
  p.drawText(8, h - 8, QStringLiteral("Front · red   |   Rear · green"));
}
