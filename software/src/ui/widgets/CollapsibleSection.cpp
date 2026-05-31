#include "CollapsibleSection.h"

#include "../../core/Theme.h"

#include <QToolButton>
#include <QVBoxLayout>

CollapsibleSection::CollapsibleSection(const QString& title, QWidget* parent,
                                       bool expanded)
    : QWidget(parent) {
  // Hug content vertically so stacked sections don't grow to fill slack.
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  header_ = new QToolButton(this);
  header_->setText(title);
  header_->setCheckable(true);
  header_->setChecked(expanded);
  header_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  header_->setArrowType(expanded ? Qt::DownArrow : Qt::RightArrow);
  header_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  header_->setStyleSheet(
      QString("QToolButton { background:%1; color:%2; border:none; "
              "padding:6px 8px; font-weight:bold; text-align:left; }"
              "QToolButton:hover { background:%3; }")
          .arg(Theme::hex(Theme::kSurfaceAlt))
          .arg(Theme::hex(Theme::kText))
          .arg(Theme::hex(Theme::kBorder)));
  root->addWidget(header_);

  body_ = new QWidget(this);
  bodyLayout_ = new QVBoxLayout(body_);
  bodyLayout_->setContentsMargins(8, 8, 8, 8);
  bodyLayout_->setSpacing(6);
  body_->setVisible(expanded);
  root->addWidget(body_);

  connect(header_, &QToolButton::toggled, this, [this](bool on) {
    body_->setVisible(on);
    updateArrow();
  });
}

void CollapsibleSection::setContentWidget(QWidget* content) {
  content->setParent(body_);
  bodyLayout_->addWidget(content);
}

void CollapsibleSection::setExpanded(bool on) { header_->setChecked(on); }
bool CollapsibleSection::isExpanded() const { return header_->isChecked(); }

void CollapsibleSection::updateArrow() {
  header_->setArrowType(header_->isChecked() ? Qt::DownArrow : Qt::RightArrow);
}
