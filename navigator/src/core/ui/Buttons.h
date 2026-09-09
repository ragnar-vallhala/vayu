#pragma once

#include <QPushButton>

// Thin QPushButton subclasses whose only job is to set objectName so
// resources/styles/dark.qss can style them.
//
// Why subclasses rather than helper functions: object names compose
// with Qt's stylesheet selector engine, propagate through promotion
// in Designer, and stay tagged after `setStyleSheet("")` resets.
// Pure helpers would re-leak inline styling on every reassignment.
//
// Use whichever name reflects the semantic role of the button, not
// its colour:
//   PrimaryButton  — main accent action on a panel (blue)
//   SuccessButton  — affirmative connect / start (green)
//   DangerButton   — destructive or armed (red); supports :checked
//   GhostButton    — quiet secondary action (flat grey)
//   BackButton     — page-back navigation (smaller padding)

namespace ui {

class PrimaryButton : public QPushButton {
  Q_OBJECT
public:
  explicit PrimaryButton(QWidget *parent = nullptr) : QPushButton(parent) {
    setObjectName("PrimaryButton");
  }
  explicit PrimaryButton(const QString &text, QWidget *parent = nullptr)
      : QPushButton(text, parent) {
    setObjectName("PrimaryButton");
  }
};

class SuccessButton : public QPushButton {
  Q_OBJECT
public:
  explicit SuccessButton(QWidget *parent = nullptr) : QPushButton(parent) {
    setObjectName("SuccessButton");
  }
  explicit SuccessButton(const QString &text, QWidget *parent = nullptr)
      : QPushButton(text, parent) {
    setObjectName("SuccessButton");
  }
};

class DangerButton : public QPushButton {
  Q_OBJECT
public:
  explicit DangerButton(QWidget *parent = nullptr) : QPushButton(parent) {
    setObjectName("DangerButton");
  }
  explicit DangerButton(const QString &text, QWidget *parent = nullptr)
      : QPushButton(text, parent) {
    setObjectName("DangerButton");
  }
};

class GhostButton : public QPushButton {
  Q_OBJECT
public:
  explicit GhostButton(QWidget *parent = nullptr) : QPushButton(parent) {
    setObjectName("GhostButton");
  }
  explicit GhostButton(const QString &text, QWidget *parent = nullptr)
      : QPushButton(text, parent) {
    setObjectName("GhostButton");
  }
};

class BackButton : public QPushButton {
  Q_OBJECT
public:
  explicit BackButton(QWidget *parent = nullptr) : QPushButton(parent) {
    setObjectName("BackButton");
    setText(tr("Back"));
  }
};

} // namespace ui
