#include <QLabel>
#include <QtTest>

#include "AboutDialog.h"

// Phase-3A/3B smoke: the About dialog renders the app identity, and docsPath()
// returns either a real directory or an empty string (never garbage).
class TstAboutDialog : public QObject {
  Q_OBJECT

private slots:
  void showsAppName();
  void docsPathIsEmptyOrExists();
};

void TstAboutDialog::showsAppName() {
  AboutDialog dlg;
  bool found = false;
  for (auto *l : dlg.findChildren<QLabel *>())
    if (l->text().contains("Navigator")) {
      found = true;
      break;
    }
  QVERIFY(found);
}

void TstAboutDialog::docsPathIsEmptyOrExists() {
  const QString p = AboutDialog::docsPath();
  if (!p.isEmpty())
    QVERIFY(QDir(p).exists());
}

QTEST_MAIN(TstAboutDialog)
#include "tst_about_dialog.moc"
