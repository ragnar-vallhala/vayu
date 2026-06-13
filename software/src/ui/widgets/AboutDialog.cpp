#include "AboutDialog.h"

#include <QApplication>
#include <QDialogButtonBox>
#include <QDir>
#include <QLabel>
#include <QVBoxLayout>

#ifndef NAVIGATOR_VERSION
#define NAVIGATOR_VERSION "dev"
#endif

AboutDialog::AboutDialog(QWidget *parent) : QDialog(parent) {
  setWindowTitle(tr("About Navigator"));

  auto *v = new QVBoxLayout(this);

  auto *title = new QLabel(tr("Navigator"), this);
  QFont tf = title->font();
  tf.setPointSizeF(tf.pointSizeF() * 1.6);
  tf.setBold(true);
  title->setFont(tf);
  v->addWidget(title);

  // Rich-text body: version/build/runtime + flight-stack components.
  const QString body =
      tr("<b>Version</b> %1<br>"
         "<b>Built</b> %2 %3<br>"
         "<b>Qt</b> %4<br>"
         "<b>Telemetry protocol</b> v1<br><br>"
         "<b>Flight stack</b><br>"
         "• Navigator — ground control station<br>"
         "• Vayu — flight controller firmware<br>"
         "• vaios — real-time OS<br>"
         "• NavHAL — hardware abstraction layer")
          .arg(QStringLiteral(NAVIGATOR_VERSION), QStringLiteral(__DATE__),
               QStringLiteral(__TIME__), QStringLiteral(QT_VERSION_STR));
  auto *label = new QLabel(body, this);
  label->setTextFormat(Qt::RichText);
  label->setTextInteractionFlags(Qt::TextBrowserInteraction);
  v->addWidget(label);

  auto *box = new QDialogButtonBox(QDialogButtonBox::Close, this);
  connect(box, &QDialogButtonBox::rejected, this, &QDialog::reject);
  connect(box, &QDialogButtonBox::accepted, this, &QDialog::accept);
  v->addWidget(box);
}

QString AboutDialog::docsPath() {
  const QString appDir = QCoreApplication::applicationDirPath();
  const QStringList candidates = {
      appDir + "/docs",
      appDir + "/../docs",
      appDir + "/../software/docs",
      // Source-tree fallback for a dev run from the build directory.
      appDir + "/../../docs",
  };
  for (const QString &c : candidates) {
    QDir d(c);
    if (d.exists())
      return d.absolutePath();
  }
  return QString();
}
