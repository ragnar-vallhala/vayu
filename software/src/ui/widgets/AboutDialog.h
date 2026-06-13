#pragma once

#include <QDialog>
#include <QString>

// Static About dialog (FR-UX-22): app name, version, build stamp, Qt + protocol
// versions, and the flight-stack components. Opened from Help ▸ About.
class AboutDialog : public QDialog {
  Q_OBJECT

public:
  explicit AboutDialog(QWidget *parent = nullptr);

  // First existing candidate for the bundled documentation directory, or an
  // empty string if none is found. Static + filesystem-only so Help ▸
  // Documentation (FR-UX-23) and tests can both resolve it.
  static QString docsPath();
};
