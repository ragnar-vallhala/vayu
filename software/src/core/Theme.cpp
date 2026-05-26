#include "Theme.h"

#include <QApplication>
#include <QFile>
#include <QPalette>
#include <QStyleFactory>
#include <QTextStream>

namespace Theme {

QString loadStyleSheet() {
  QFile f(":/styles/dark.qss");
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return {};
  return QString::fromUtf8(f.readAll());
}

void apply() {
  qApp->setStyle(QStyleFactory::create("Fusion"));

  QPalette p;
  p.setColor(QPalette::Window,          kBg);
  p.setColor(QPalette::WindowText,      kTextMuted);
  p.setColor(QPalette::Base,            kBase);
  p.setColor(QPalette::AlternateBase,   kSurface);
  p.setColor(QPalette::ToolTipBase,     kSurfaceAlt);
  p.setColor(QPalette::ToolTipText,     kTextMuted);
  p.setColor(QPalette::Text,            kTextMuted);
  p.setColor(QPalette::Button,          kSurfaceAlt);
  p.setColor(QPalette::ButtonText,      kTextMuted);
  p.setColor(QPalette::BrightText,      kDanger);
  p.setColor(QPalette::Link,            kAccent);
  p.setColor(QPalette::Highlight,       kAccent);
  p.setColor(QPalette::HighlightedText, Qt::black);
  p.setColor(QPalette::Disabled, QPalette::Text,       kTextDim);
  p.setColor(QPalette::Disabled, QPalette::ButtonText, kTextDim);
  qApp->setPalette(p);

  const QString qss = loadStyleSheet();
  if (!qss.isEmpty()) qApp->setStyleSheet(qss);
}

}  // namespace Theme
