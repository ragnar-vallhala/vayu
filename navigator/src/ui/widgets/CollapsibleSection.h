#pragma once

#include <QWidget>

class QToolButton;
class QVBoxLayout;

// CollapsibleSection — a Blender-style property panel: a clickable header
// with a disclosure triangle that expands/collapses a content widget.
// Multiple sections stack in a scroll area to form the categorized
// right-hand properties panel.
class CollapsibleSection : public QWidget {
  Q_OBJECT
public:
  explicit CollapsibleSection(const QString &title, QWidget *parent = nullptr,
                              bool expanded = true);

  // Reparents `content` into the section body.
  void setContentWidget(QWidget *content);
  void setExpanded(bool on);
  bool isExpanded() const;

private:
  void updateArrow();

  QToolButton *header_ = nullptr;
  QWidget *body_ = nullptr;
  QVBoxLayout *bodyLayout_ = nullptr;
};
