#include <QtTest>

#include "SitlStack.h"

// LIVE bring-up test for the C++ SitlStack (the no-Python autotune driver).
// It spawns the real vsim_d + vayu_sitl, so it only runs when the binaries are
// pointed to via env vars; otherwise it skips. Run it with the sim built:
//
//   VAYU_VSIM_BIN=<repo>/build_vsim/vsim_d \
//   VAYU_SITL_BIN=<repo>/tools/sim_host/build_sitl/vayu_sitl \
//   ctest -R tst_sitl_stack --output-on-failure
//
// Verifies: the stack starts, firmware telemetry flows (control-loop samples +
// a reported system state), and the firmware arms from C++-driven RC.
class TstSitlStack : public QObject {
  Q_OBJECT

private slots:
  void bringUpAndArm();
};

void TstSitlStack::bringUpAndArm() {
  const QString vsim = qEnvironmentVariable("VAYU_VSIM_BIN");
  const QString sitl = qEnvironmentVariable("VAYU_SITL_BIN");
  if (vsim.isEmpty() || sitl.isEmpty())
    QSKIP("set VAYU_VSIM_BIN and VAYU_SITL_BIN to run the live SITL test");
  if (!QFile::exists(vsim) || !QFile::exists(sitl))
    QSKIP("VAYU_VSIM_BIN / VAYU_SITL_BIN do not exist");

  SitlStack::Config cfg;
  cfg.vsimBin = vsim;
  cfg.sitlBin = sitl;
  cfg.suffix = "_cpptest";
  cfg.quiet = true;

  SitlStack stack(cfg);
  QString err;
  QVERIFY2(stack.start(&err), qPrintable("start failed: " + err));

  // Telemetry must be flowing and a state reported.
  QVERIFY(!stack.snapshot().empty());
  qInfo() << "system state after boot:" << stack.lastState();
  QVERIFY(!stack.lastState().isEmpty());

  // Put it on the tuning rig, level, and arm from C++-driven RC.
  stack.setTestRig(true);
  stack.reset(/*seed=*/0xC0FFEE);
  const bool armed = stack.arm(/*timeoutMs=*/3000);
  qInfo() << "arm result:" << armed << "state:" << stack.lastState();
  QVERIFY2(armed, "firmware did not reach ARMED from C++-driven RC");

  // A SET_PID over the C++ navlink path must not disturb the link (still ARMED,
  // telemetry still flowing).
  stack.clearSamples();
  stack.setPid(/*rate*/ 1, /*roll*/ 0, 0.007f, 0.002f, 0.0005f, 0.0f);
  QTest::qWait(300);
  QVERIFY(!stack.snapshot().empty());

  stack.disarm();
  stack.stop();
}

QTEST_GUILESS_MAIN(TstSitlStack)
#include "tst_sitl_stack.moc"
