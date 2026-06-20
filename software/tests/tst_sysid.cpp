#include <QtTest>
#include <cmath>

#include "SysId.h"

using namespace autotune;

// System-ID plant fit + analytic gain design. The fit is validated against a
// SYNTHETIC plant with known (K, tau): simulate omega/u = K/(s(tau s+1)) with a
// rich input, then check identifyPlant recovers K and tau. The design formulas
// are checked for the loop-shaping invariants (PD zero on the actuator pole,
// crossover at wc).
class TstSysId : public QObject {
  Q_OBJECT

  // Simulate the rate plant omega/u = K/(s(tau s+1)) under input u[].
  // Discrete: accel is a first-order lag of u (pole exp(-dt/tau), DC gain K),
  // omega integrates accel. Returns omega[] aligned to u[].
  static QVector<double> simPlant(const QVector<double> &u, double K,
                                  double tau, double dt) {
    const double a = std::exp(-dt / tau);
    const double b = K * (1.0 - a);  // so DC gain accel/u = b/(1-a) = K
    QVector<double> omega(u.size(), 0.0);
    double accel = 0.0, w = 0.0;
    for (int i = 0; i < u.size(); ++i) {
      omega[i] = w;
      accel = a * accel + b * u[i];  // u[i] drives accel[i+1]
      w += accel * dt;
    }
    return omega;
  }

  // Deterministic rich excitation (a chirp-ish sum of sines), no RNG.
  static QVector<double> excite(int n, double dt) {
    QVector<double> u(n);
    for (int i = 0; i < n; ++i) {
      const double t = i * dt;
      u[i] = 0.02 * (std::sin(2 * M_PI * 1.5 * t) +
                     0.6 * std::sin(2 * M_PI * 7.0 * t) +
                     0.4 * std::sin(2 * M_PI * 13.0 * t));
    }
    return u;
  }

private slots:
  void recoversKnownPlant();
  void rejectsShortData();
  void designPlacesZeroOnActuatorPole();
  void crossoverCapsRateKp();
  void averageOfIdenticalIsItself();
};

void TstSysId::recoversKnownPlant() {
  const double dt = 0.001, K = 50000.0, tau = 0.010;  // 10 ms lag
  const auto u = excite(3000, dt);
  const auto omega = simPlant(u, K, tau, dt);

  const Plant p = identifyPlant(u, omega, dt);
  QVERIFY(p.ok);
  QVERIFY(p.r2 > 0.99);                       // clean synthetic -> near-perfect
  QVERIFY2(std::fabs(p.tau - tau) / tau < 0.05,
           qPrintable(QStringLiteral("tau=%1 want %2").arg(p.tau).arg(tau)));
  QVERIFY2(std::fabs(p.K - K) / K < 0.05,
           qPrintable(QStringLiteral("K=%1 want %2").arg(p.K).arg(K)));
}

void TstSysId::rejectsShortData() {
  QVector<double> u(10, 0.01), omega(10, 0.0);
  QVERIFY(!identifyPlant(u, omega, 0.001).ok);  // too few samples
  QVERIFY(!identifyPlant({}, {}, 0.001).ok);
}

void TstSysId::designPlacesZeroOnActuatorPole() {
  Plant p;
  p.ok = true; p.K = 50000.0; p.tau = 0.010;
  const double wc = 20.0;
  const DesignGains g = designGains(p, wc);
  // rate_kp = wc/K
  QVERIFY(std::fabs(g.rate_kp - wc / p.K) < 1e-12);
  // PD zero on the actuator pole: kd/kp == tau
  QVERIFY(std::fabs(g.rate_kd / g.rate_kp - p.tau) < 1e-9);
  // integral knee a decade below wc
  QVERIFY(std::fabs(g.rate_ki - 0.1 * wc * g.rate_kp) < 1e-15);
  // cascade separation
  QVERIFY(std::fabs(g.angle_kp - 0.25 * wc) < 1e-12);
}

void TstSysId::crossoverCapsRateKp() {
  Plant p;
  p.ok = true; p.K = 50000.0; p.tau = 0.002;  // fast actuator -> high bwFrac/tau
  // Unbounded wc = 0.33/tau = 165 -> rate_kp = 165/50000 = 0.0033 (under cap).
  const double wcFree = chooseCrossover(p, 0.33, /*kpMax=*/1.0);
  QVERIFY(std::fabs(wcFree - 0.33 / p.tau) < 1e-6);
  // Tight cap forces wc down so rate_kp == kpMax exactly.
  const double kpMax = 0.001;
  const double wcCapped = chooseCrossover(p, 0.33, kpMax);
  const DesignGains g = designGains(p, wcCapped);
  QVERIFY2(g.rate_kp <= kpMax * 1.0001,
           qPrintable(QStringLiteral("rate_kp=%1 > cap %2").arg(g.rate_kp).arg(kpMax)));
  QVERIFY(std::fabs(g.rate_kp - kpMax) < 1e-9);
}

void TstSysId::averageOfIdenticalIsItself() {
  Plant a; a.ok = true; a.K = 40000; a.tau = 0.008; a.r2 = 0.9;
  const Plant avg = averagePlants(a, a);
  QVERIFY(avg.ok);
  QVERIFY(std::fabs(avg.K - a.K) < 1e-6);
  QVERIFY(std::fabs(avg.tau - a.tau) < 1e-9);
  // One bad axis -> falls back to the good one.
  Plant bad;  // !ok
  const Plant only = averagePlants(a, bad);
  QVERIFY(std::fabs(only.K - a.K) < 1e-6);
}

QTEST_MAIN(TstSysId)
#include "tst_sysid.moc"
