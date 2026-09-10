#pragma once

#include <QAbstractSpinBox>
#include <QApplication>
#include <QComboBox>
#include <QEvent>
#include <QObject>
#include <QWidget>

// Application-wide event filter that stops the mouse wheel from editing spin
// boxes and combo boxes. Scrolling the mouse over a field used to silently
// change its value while the user was only trying to scroll the surrounding
// panel -- annoying and error-prone, especially since these editors live
// inside a QScrollArea. Here we swallow the wheel for those widgets and
// re-dispatch it to their parent so the enclosing scroll area still scrolls.
// Change values with the spin box up/down buttons (or by typing) instead.
//
// Install once on the QApplication: app.installEventFilter(new ui::WheelGuard(&app));
namespace ui {

class WheelGuard : public QObject {
public:
  using QObject::QObject;

protected:
  bool eventFilter(QObject *obj, QEvent *ev) override {
    if (ev->type() == QEvent::Wheel && (qobject_cast<QAbstractSpinBox *>(obj) ||
                                        qobject_cast<QComboBox *>(obj))) {
      // Forward the wheel to the parent so a wrapping QScrollArea keeps
      // scrolling, then consume it so the field's value is left untouched.
      if (auto *w = qobject_cast<QWidget *>(obj)) {
        if (QWidget *parent = w->parentWidget())
          QApplication::sendEvent(parent, ev);
      }
      return true;
    }
    return false;
  }
};

} // namespace ui
