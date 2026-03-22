#pragma once

#include <QTimer>
#include <QVector>
#include <QWidget>

class MotorStatusWidget : public QWidget {
  Q_OBJECT

public:
  explicit MotorStatusWidget(QWidget *parent = nullptr);
  ~MotorStatusWidget() override = default;

  void setMotorSpeeds(const QVector<float> &speeds);

signals:
  void backToHomeRequested();

protected:
  void paintEvent(QPaintEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;

private slots:
  void onUpdateTimer();

private:
  void drawDrone(QPainter &p, int w, int h);
  void drawMotor(QPainter &p, int x, int y, float speed, int motorIdx);

  QVector<float> m_speeds;
  QTimer *m_dummyTimer;
  float m_phase = 0.0f;
};
