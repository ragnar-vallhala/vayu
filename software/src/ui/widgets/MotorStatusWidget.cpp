#include "MotorStatusWidget.h"
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QVBoxLayout>
#include <QtMath>

MotorStatusWidget::MotorStatusWidget(QWidget *parent) : QWidget(parent) {
  m_speeds = {0.0f, 0.0f, 0.0f, 0.0f};

  // Layout for the "Back" button
  auto *layout = new QVBoxLayout(this);
  auto *topRow = new QHBoxLayout();

  auto *backBtn = new QPushButton("← BACK TO HOME", this);
  backBtn->setFixedSize(140, 32);
  backBtn->setStyleSheet(
      "QPushButton { background: #2C313A; color: #61AFEF; border: 1px solid "
      "#3E4452; "
      "border-radius: 4px; font-weight: bold; font-size: 11px; }"
      "QPushButton:hover { background: #3E4452; }");
  connect(backBtn, &QPushButton::clicked, this,
          &MotorStatusWidget::backToHomeRequested);

  topRow->addWidget(backBtn);
  topRow->addStretch();

  auto *title = new QLabel("MOTOR STATUS", this);
  title->setStyleSheet("color: #ABB2BF; font-weight: bold; font-size: 14px;");
  topRow->addWidget(title);
  topRow->addStretch();

  // Placeholder for symmetry
  auto *spacer = new QWidget(this);
  spacer->setFixedSize(140, 32);
  topRow->addWidget(spacer);

  layout->addLayout(topRow);
  layout->addStretch();
}

void MotorStatusWidget::setMotorSpeeds(const QVector<float> &speeds) {
  if (speeds.size() >= 4) {
    m_speeds = speeds;
    update();
  }
}

void MotorStatusWidget::paintEvent(QPaintEvent *event) {
  Q_UNUSED(event);
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  int w = width();
  int h = height();

  // Background
  p.fillRect(rect(), QColor(26, 29, 39));

  drawDrone(p, w, h);
}

void MotorStatusWidget::drawDrone(QPainter &p, int w, int h) {
  int cx = w / 2;
  int cy = h / 2;
  int size = qMin(w, h) * 0.7;
  int armLen = size / 2;

  p.save();
  p.translate(cx, cy);

  // Draw arms (X configuration)
  p.setPen(QPen(QColor(62, 68, 82), 12, Qt::SolidLine, Qt::RoundCap));
  p.drawLine(-armLen, -armLen, armLen, armLen);
  p.drawLine(-armLen, armLen, armLen, -armLen);

  // Draw center body
  p.setPen(QPen(QColor(97, 175, 239), 2));
  p.setBrush(QColor(33, 37, 43));
  p.drawRoundedRect(-40, -60, 80, 120, 15, 15);

  // "Front" indicator
  p.setBrush(QColor(224, 108, 117));
  p.setPen(Qt::NoPen);
  p.drawEllipse(-10, -50, 20, 20);

  // Draw motors at ends of arms
  // Motor order: Front-Right (1), Back-Right (2), Back-Left (3), Front-Left (4)
  // Coords: (+, -), (+, +), (-, +), (-, -)
  drawMotor(p, armLen, -armLen, m_speeds[0], 1);
  drawMotor(p, armLen, armLen, m_speeds[1], 2);
  drawMotor(p, -armLen, armLen, m_speeds[2], 3);
  drawMotor(p, -armLen, -armLen, m_speeds[3], 4);

  p.restore();
}

void MotorStatusWidget::drawMotor(QPainter &p, int x, int y, float speed,
                                  int motorIdx) {
  int r = 40;

  p.save();
  p.translate(x, y);

  // Motor base
  p.setPen(QPen(QColor(92, 99, 112), 2));
  p.setBrush(QColor(40, 44, 52));
  p.drawEllipse(-r, -r, 2 * r, 2 * r);

  // Speed arc
  QRectF rect(-r + 4, -r + 4, 2 * r - 8, 2 * r - 8);
  int spanAngle = -static_cast<int>(speed * 360 * 16);
  p.setPen(QPen(QColor(152, 195, 121), 6, Qt::SolidLine, Qt::RoundCap));
  p.drawArc(rect, 90 * 16, spanAngle);

  // Motor label
  p.setPen(QColor(171, 178, 191));
  QFont font = p.font();
  font.setBold(true);
  font.setPixelSize(12);
  p.setFont(font);
  p.drawText(QRect(-r, -r, 2 * r, 2 * r), Qt::AlignCenter,
             QString("M%1").arg(motorIdx));

  // Percentage
  font.setPixelSize(10);
  p.setFont(font);
  p.setPen(QColor(152, 195, 121));
  p.drawText(QRect(-r, r + 5, 2 * r, 20), Qt::AlignCenter,
             QString("%1%").arg(static_cast<int>(speed * 100)));

  p.restore();
}

void MotorStatusWidget::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);
}
