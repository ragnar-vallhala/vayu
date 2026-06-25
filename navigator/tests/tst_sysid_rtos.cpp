#include <QtTest>
#include <cmath>

#include "RtosEval.h"
#include "SysId.h"

using namespace autotune;

// End-to-end System-ID against the REAL vayu_sitl_rtos binary: fly an
// identification chirp, fit the rate-loop plant, design gains, and verify the
// designed gains track in a doublet. Opt-in (the binary is a separate build):
//   cmake -S sim/host -B build_sitl_rtos -DVAYU_SITL_RTOS_BUILD=ON
//   cmake --build build_sitl_rtos --target vayu_sitl_rtos
//   VAYU_RTOS_BIN=<repo>/build_sitl_rtos/vayu_sitl_rtos ctest -R tst_sysid_rtos
class TstSysIdRtos : public QObject {
  Q_OBJECT

  QString m_bin;

private slots:
  void initTestCase() {
    m_bin = qEnvironmentVariable("VAYU_RTOS_BIN");
    if (m_bin.isEmpty())
      QSKIP("set VAYU_RTOS_BIN to the vayu_sitl_rtos binary to run this test");
    if (!QFile::exists(m_bin))
      QSKIP("VAYU_RTOS_BIN does not exist");
  }

  void identifiesPlausiblePlant();
};

void TstSysIdRtos::identifiesPlausiblePlant() {
  RtosEval rtos(m_bin, QStringLiteral("_sidtest"));
  QVERIFY(rtos.binExists());
  // Rich chirp, soft rig, long hold — the conditions the worker uses.
  rtos.setExcitation(/*stepUs=*/1750, /*tetherK=*/30.0, /*holdS=*/2.0,
                     /*retS=*/0.4, /*settleS=*/0.6, /*waveform=*/1,
                     /*chirpF0=*/0.5, /*chirpF1=*/18.0);

  RtosEval::RawRate raw;
  QString err;
  QVERIFY2(rtos.captureRate(RtosEval::Gains{}, /*seed=*/12345, raw, &err),
           qPrintable(err));
  QVERIFY(raw.rollU.size() > 100);
  QCOMPARE(raw.rollU.size(), raw.rollOmega.size());

  const Plant pr = identifyPlant(raw.rollU, raw.rollOmega, raw.dt);
  const Plant pp = identifyPlant(raw.pitchU, raw.pitchOmega, raw.dt);
  QVERIFY2(pr.ok, "roll plant fit failed");
  // Roll is the cleaner axis (excited first, no carryover) — demand a real fit.
  QVERIFY2(pr.r2 > 0.5, qPrintable(QStringLiteral("roll R^2=%1").arg(pr.r2)));
  QVERIFY(pr.K > 0.0);
  QVERIFY2(pr.tau > 0.0005 && pr.tau < 0.1,
           qPrintable(QStringLiteral("tau=%1 ms").arg(pr.tau * 1000)));

  const Plant prp = averagePlants(pr, pp);
  const double wc = chooseCrossover(prp, 0.33);
  const DesignGains g = designGains(prp, wc);
  qInfo("roll K=%.0f tau=%.1fms R2=%.2f | wc=%.1f -> rate_kp=%.6g ki=%.6g "
        "kd=%.6g angle_kp=%.3g",
        pr.K, pr.tau * 1000, pr.r2, wc, g.rate_kp, g.rate_ki, g.rate_kd,
        g.angle_kp);
  // Designed gains land in the known-good band (analysis: floor 5e-4, buzz ~1e-2).
  QVERIFY2(g.rate_kp > 1e-5 && g.rate_kp < 2e-2,
           qPrintable(QStringLiteral("rate_kp=%1 out of band").arg(g.rate_kp)));
  QVERIFY(g.angle_kp > 0.5 && g.angle_kp < 12.0);
  QVERIFY(std::isfinite(g.rate_ki) && std::isfinite(g.rate_kd));

  // The designed gains must actually fly: a verification doublet scores finite
  // and does not diverge.
  rtos.setExcitation(1800, 30.0, 1.0, 0.7, 0.6, /*waveform=*/0, 0.5, 18.0);
  RtosEval::Gains vg;
  vg.rate_kp = g.rate_kp; vg.rate_ki = g.rate_ki; vg.rate_kd = g.rate_kd;
  vg.angle_kp = g.angle_kp;
  const RtosResult vr = rtos.rollout(vg, 12345, /*tuneYaw=*/false);
  QVERIFY2(vr.ok, qPrintable(vr.error));
  QVERIFY2(!vr.diverged, "designed gains diverged in the verification doublet");
}

QTEST_MAIN(TstSysIdRtos)
#include "tst_sysid_rtos.moc"
