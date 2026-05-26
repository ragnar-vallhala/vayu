#include "MotorStatusWidget.h"

#include "core/CsvExport.h"
#include "core/Notify.h"
#include "core/ui/Buttons.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QVBoxLayout>
#include <QtMath>

// ----------------------------------------------------------------------------
// DroneViewWidget: Encapsulates the drone schematic drawing
// ----------------------------------------------------------------------------
class DroneViewWidget : public QWidget {
public:
  explicit DroneViewWidget(QWidget *parent = nullptr) : QWidget(parent) {
    m_speeds = {0.0f, 0.0f, 0.0f, 0.0f};
    setMinimumSize(300, 300);
  }

  void setSpeeds(const QVector<float> &speeds) {
    m_speeds = speeds;
    update();
  }

protected:
  void paintEvent(QPaintEvent *event) override {
    Q_UNUSED(event);
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    int w = width();
    int h = height();
    int cx = w / 2;
    int cy = h / 2;
    int size = qMin(w, h) * 0.8;
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

    // Motors
    drawMotor(p, armLen, -armLen, m_speeds[0], 1);
    drawMotor(p, armLen, armLen, m_speeds[1], 2);
    drawMotor(p, -armLen, armLen, m_speeds[2], 3);
    drawMotor(p, -armLen, -armLen, m_speeds[3], 4);

    p.restore();
  }

private:
  void drawMotor(QPainter &p, int x, int y, float speed, int motorIdx) {
    int r = 35;
    p.save();
    p.translate(x, y);

    p.setPen(QPen(QColor(92, 99, 112), 2));
    p.setBrush(QColor(40, 44, 52));
    p.drawEllipse(-r, -r, 2 * r, 2 * r);

    QRectF rect(-r + 4, -r + 4, 2 * r - 8, 2 * r - 8);
    int spanAngle = -static_cast<int>(speed * 360 * 16);
    p.setPen(QPen(QColor(152, 195, 121), 6, Qt::SolidLine, Qt::RoundCap));
    p.drawArc(rect, 90 * 16, spanAngle);

    p.setPen(QColor(171, 178, 191));
    QFont font = p.font();
    font.setBold(true);
    font.setPixelSize(11);
    p.setFont(font);
    p.drawText(QRect(-r, -r, 2 * r, 2 * r), Qt::AlignCenter,
               QString("M%1").arg(motorIdx));

    p.restore();
  }

  QVector<float> m_speeds;
};

// ----------------------------------------------------------------------------
// MotorStatusWidget
// ----------------------------------------------------------------------------
MotorStatusWidget::MotorStatusWidget(QWidget *parent) : QWidget(parent) {
  m_speeds = {0.0f, 0.0f, 0.0f, 0.0f};

  auto *mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(15, 15, 15, 15);
  mainLayout->setSpacing(10);

  // Header
  auto *header = new QHBoxLayout();
  auto *backBtn = new ui::BackButton(this);
  backBtn->setText(tr("← BACK TO HOME"));
  backBtn->setFixedSize(140, 32);
  backBtn->setToolTip(tr("Return to home"));
  connect(backBtn, &QPushButton::clicked, this,
          &MotorStatusWidget::backToHomeRequested);

  auto *title = new QLabel("MOTOR STATUS", this);
  title->setStyleSheet("color: #ABB2BF; font-weight: bold; font-size: 14px;");

  // Export CSV — the 4-series graph keyed by motor.
  auto *exportBtn = new ui::GhostButton(tr("Export CSV"), this);
  exportBtn->setToolTip(tr("Save the currently-buffered motor speed traces"));
  connect(exportBtn, &QPushButton::clicked, this, [this] {
    const QString stamp =
        QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss");
    const QString path = CsvExport::promptAndWriteCombined(
        this, QString("motors-%1.csv").arg(stamp),
        {{m_graph, {"m1_fr", "m2_rr", "m3_rl", "m4_fl"}}});
    if (!path.isEmpty()) Notify::ok(this, tr("Wrote %1").arg(path));
  });

  header->addWidget(backBtn);
  header->addStretch();
  header->addWidget(title);
  header->addStretch();
  header->addWidget(exportBtn);
  mainLayout->addLayout(header);

  // Content Area
  auto *content = new QHBoxLayout();
  content->setSpacing(20);

  // Left: Drone View
  m_droneView = new DroneViewWidget(this);
  content->addWidget(m_droneView, 3); // More weight for drone

  // Right: Stats and Graph
  auto *rightPanel = new QVBoxLayout();
  rightPanel->setSpacing(10);

  // Motor Stats Cards
  const char *colors[] = {"#E06C75", "#98C379", "#61AFEF", "#D19A66"};
  for (int i = 0; i < 4; ++i) {
    auto *card = new QWidget(this);
    card->setStyleSheet(
        "background: #282C34; border-radius: 6px; border: 1px solid #3E4452;");
    auto *cardLayout = new QHBoxLayout(card);
    cardLayout->setContentsMargins(10, 5, 10, 5);

    auto *nameLabel = new QLabel(QString("M%1").arg(i + 1), this);
    nameLabel->setStyleSheet(
        QString("color: %1; font-weight: bold; font-size: 14px; border: none;")
            .arg(colors[i]));

    m_valLabels[i] = new QLabel("0%", this);
    m_valLabels[i]->setStyleSheet(
        "color: #E8F0FE; font-family: 'Monospace'; font-weight: bold; "
        "font-size: 16px; border: none;");
    m_valLabels[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    m_valLabels[i]->setFixedWidth(60);

    m_stdLabels[i] = new QLabel("± σ 0.000", this);
    m_stdLabels[i]->setStyleSheet("color: #5C6370; font-family: 'Monospace'; "
                                  "font-size: 11px; border: none;");
    m_stdLabels[i]->setFixedWidth(100);

    cardLayout->addWidget(nameLabel);
    cardLayout->addStretch();
    cardLayout->addWidget(m_valLabels[i]);
    cardLayout->addSpacing(10);
    cardLayout->addWidget(m_stdLabels[i]);

    rightPanel->addWidget(card);
  }

  // Graph
  m_graph = new RealTimeGraph(this, 4);
  for (int i = 0; i < 4; ++i) {
    m_graph->setColor(i, QColor(colors[i]));
  }
  m_graph->setWindowSeconds(10);
  rightPanel->addWidget(m_graph, 1);

  content->addLayout(rightPanel, 2);
  mainLayout->addLayout(content, 1);

  // Page background comes from the global QSS QMainWindow rule.
}

void MotorStatusWidget::setMotorSpeeds(const QVector<float> &speeds) {
  if (speeds.size() >= 4) {
    m_speeds = speeds;
    m_droneView->setSpeeds(speeds);

    for (int i = 0; i < 4; ++i) {
      m_stats[i].push(speeds[i]);
      m_valLabels[i]->setText(
          QString("%1%").arg(static_cast<int>(speeds[i] * 100)));
      m_stdLabels[i]->setText(QString("± σ %1").arg(
          static_cast<double>(m_stats[i].stdDev()), 0, 'f', 3));
      m_graph->appendData(speeds[i], i);
    }
  }
}
