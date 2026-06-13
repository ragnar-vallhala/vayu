#include <QtTest>

#include "CalibrationWizard.h"

// Gated calibration wizard step machine: per-mode step lists, firmware-driven
// advancement (current + done tracking), progress, and completion.
class TstCalibrationWizard : public QObject {
  Q_OBJECT

private slots:
  void modesHaveExpectedSteps();
  void sixAxisAdvancesAndCompletes();
  void instructionMarksEarlierStepsDone();
  void nonStepInstructionIgnored();
  void magIsSingleFigure8();
};

void TstCalibrationWizard::modesHaveExpectedSteps() {
  CalibrationWizard w;
  w.begin(CalibMode::Gyro);
  QCOMPARE(w.stepCount(), 1);
  w.begin(CalibMode::AccelBias);
  QCOMPARE(w.stepCount(), 1);
  w.begin(CalibMode::Accel6Axis);
  QCOMPARE(w.stepCount(), 6);
  w.begin(CalibMode::Mag);
  QCOMPARE(w.stepCount(), 1);
  QCOMPARE(w.steps().first().orient, CalibUpdateType::FreeRot);
}

void TstCalibrationWizard::sixAxisAdvancesAndCompletes() {
  CalibrationWizard w;
  w.begin(CalibMode::Accel6Axis);
  QVERIFY(w.isActive());
  QVERIFY(!w.isComplete());
  QCOMPARE(w.currentIndex(), -1);
  QCOMPARE(w.progress(), 0.0);

  w.onInstruction(CalibUpdateType::NoseUp);
  QCOMPARE(w.currentIndex(), 0);
  QCOMPARE(w.doneCount(), 0);

  w.markComplete();
  QVERIFY(w.isComplete());
  QCOMPARE(w.doneCount(), 6);
  QCOMPARE(w.progress(), 1.0);
  QVERIFY(!w.isActive());
}

void TstCalibrationWizard::instructionMarksEarlierStepsDone() {
  CalibrationWizard w;
  w.begin(CalibMode::Accel6Axis);
  // Firmware jumps to the 3rd orientation -> steps 0,1 are done, step 2 current.
  w.onInstruction(CalibUpdateType::RightDown);  // index 2
  QCOMPARE(w.currentIndex(), 2);
  QCOMPARE(w.doneCount(), 2);
  QVERIFY(qFuzzyCompare(w.progress(), 2.0 / 6.0));
}

void TstCalibrationWizard::nonStepInstructionIgnored() {
  CalibrationWizard w;
  w.begin(CalibMode::Accel6Axis);
  w.onInstruction(CalibUpdateType::LeftDown);  // index 3
  QCOMPARE(w.currentIndex(), 3);
  // A Progress "still calibrating" status must not move the wizard.
  w.onInstruction(CalibUpdateType::Progress);
  QCOMPARE(w.currentIndex(), 3);
  QCOMPARE(w.doneCount(), 3);
}

void TstCalibrationWizard::magIsSingleFigure8() {
  CalibrationWizard w;
  w.begin(CalibMode::Mag);
  w.onInstruction(CalibUpdateType::FreeRot);
  QCOMPARE(w.currentIndex(), 0);
  w.markComplete();
  QVERIFY(w.isComplete());
  QCOMPARE(w.progress(), 1.0);
}

QTEST_APPLESS_MAIN(TstCalibrationWizard)
#include "tst_calibration_wizard.moc"
