#include <QtTest>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include "Rollout.h"
#include "Space.h"
#include "SitlStack.h"

// LIVE bring-up test for the C++ SitlStack (the no-Python autotune driver).
// It spawns the real vsim_d + vayu_sitl, so it only runs when the binaries are
// pointed to via env vars; otherwise it skips. Run it with the sim built:
//
//   VAYU_VSIM_BIN=<repo>/build_vsim/vsim_d \
//   VAYU_SITL_BIN=<repo>/sim/host/build_sitl/vayu_sitl \
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
  // Unique per process so a leftover daemon from a prior run can't hold the
  // FIFO lock and starve this one (the GUI uses a pid-based suffix too).
  cfg.suffix = QStringLiteral("_cpptest%1").arg(QCoreApplication::applicationPid());
  cfg.quiet = true;

  // Optional: load the user's LIVE (rotated) physics geometry + un-rotated mixer
  // geometry from JSON to reproduce the in-app autotune divergence headlessly.
  auto loadGeom = [](const QString &path, vsim_ctl_geometry_t *g) -> bool {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return false;
    const QJsonObject o = QJsonDocument::fromJson(f.readAll()).object();
    g->mass = o["mass"].toDouble();
    const QJsonArray I = o["inertia"].toArray();
    for (int i = 0; i < 9 && i < I.size(); ++i) g->inertia[i] = I[i].toDouble();
    const QJsonArray ms = o["motors"].toArray();
    for (int i = 0; i < 4 && i < ms.size(); ++i) {
      const QJsonObject m = ms[i].toObject();
      const QJsonArray p = m["pos"].toArray(), a = m["axis"].toArray();
      for (int k = 0; k < 3; ++k) { g->motors[i].pos[k] = p[k].toDouble(); g->motors[i].axis[k] = a[k].toDouble(); }
      g->motors[i].spin = float(m["spin"].toInt());
      g->motors[i].k_thrust = m["k_thrust"].toDouble();
      g->motors[i].k_moment = m["k_moment"].toDouble();
      g->motors[i].max_omega = m["max_omega"].toDouble();
    }
    return true;
  };
  const QString geomPath = qEnvironmentVariable("VAYU_TEST_GEOM");
  if (!geomPath.isEmpty() && loadGeom(geomPath, &cfg.geometry)) {
    cfg.hasGeometry = true;  // firmware mix derives from this (physics frame)
    qInfo() << "loaded physics geometry from" << geomPath;
  }

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

  // One full rollout with the seed gains: must score (finite, not divergence).
  autotune::Space space(/*tuneYaw=*/false);
  autotune::RolloutParams rp;
  rp.seed = 0xC0FFEE;
  const std::optional<double> cost = autotune::runRollout(
      stack, space.names(), space.seed(), /*tuneYaw=*/false, rp);
  qInfo() << "seed-gain rollout cost:"
          << (cost ? QString::number(*cost) : QStringLiteral("nullopt"));
  QVERIFY2(cost.has_value(), "rollout produced no scorable result");
  QVERIFY2(*cost < autotune::kBig, "seed gains diverged on the rig");

  stack.disarm();
  stack.stop();
}

QTEST_GUILESS_MAIN(TstSitlStack)
#include "tst_sitl_stack.moc"
