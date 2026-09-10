#include "SysId.h"

#include <cmath>

namespace autotune {

double Plant::actuatorBwHz() const {
  return (ok && tau > 1e-9) ? 1.0 / tau / (2.0 * M_PI) : 0.0;
}

Plant identifyPlant(const QVector<double> &u, const QVector<double> &omega,
                    double dt) {
  Plant p;
  const int n = omega.size();
  if (n < 50 || u.size() != n || dt <= 1e-9)
    return p; // too short to fit a lag from

  // accel[i] = (omega[i+1]-omega[i])/dt, i in [0, n-2]. Differencing factors out
  // the rigid-body integrator (omega = integral of accel), leaving the lone
  // first-order actuator lag to fit — robust against the integrator drift that a
  // two-pole fit on omega would chase.
  const int m = n - 1;
  QVector<double> accel(m);
  for (int i = 0; i < m; ++i)
    accel[i] = (omega[i + 1] - omega[i]) / dt;

  // Least-squares ARX: accel[k] = a*accel[k-1] + b*u[k]. (u[k] is the input over
  // the step that produced accel[k] = the change from omega[k] to omega[k+1].)
  double s11 = 0, s12 = 0, s22 = 0, sy1 = 0, sy2 = 0;
  int cnt = 0;
  for (int k = 1; k < m; ++k) {
    const double x1 = accel[k - 1], x2 = u[k], y = accel[k];
    s11 += x1 * x1;
    s12 += x1 * x2;
    s22 += x2 * x2;
    sy1 += y * x1;
    sy2 += y * x2;
    ++cnt;
  }
  const double det = s11 * s22 - s12 * s12;
  if (cnt < 20 || std::fabs(det) < 1e-30)
    return p;
  const double a = (sy1 * s22 - sy2 * s12) / det;
  const double b = (-sy1 * s12 + sy2 * s11) / det;
  if (!(a > 0.0 && a < 1.0))
    return p; // non-physical: no stable first-order lag in the data

  // One-step prediction R^2 on accel (fit quality the caller can gate on).
  double mean = 0;
  for (int k = 1; k < m; ++k)
    mean += accel[k];
  mean /= (m - 1);
  double ssRes = 0, ssTot = 0;
  for (int k = 1; k < m; ++k) {
    const double pred = a * accel[k - 1] + b * u[k];
    ssRes += (accel[k] - pred) * (accel[k] - pred);
    ssTot += (accel[k] - mean) * (accel[k] - mean);
  }

  p.ok = true;
  p.a = a;
  p.b = b;
  p.tau = -dt / std::log(a);
  p.K = b / (1.0 - a); // DC gain accel/u  ->  omega/u = K/(s(tau s+1))
  p.r2 = ssTot > 1e-30 ? 1.0 - ssRes / ssTot : 0.0;
  p.n = cnt;
  return p;
}

Plant averagePlants(const Plant &a, const Plant &b) {
  if (a.ok && !b.ok)
    return a;
  if (b.ok && !a.ok)
    return b;
  if (!a.ok && !b.ok)
    return Plant{};
  // R^2-weighted (floor the weight so a 0-R^2 fit still counts a little).
  const double wa = std::max(0.05, a.r2), wb = std::max(0.05, b.r2);
  Plant p;
  p.ok = true;
  p.K = (wa * a.K + wb * b.K) / (wa + wb);
  p.tau = (wa * a.tau + wb * b.tau) / (wa + wb);
  p.r2 = (wa * a.r2 + wb * b.r2) / (wa + wb);
  p.n = a.n + b.n;
  // a/b (discrete) are not meaningfully averageable; recover a representative
  // pole from the averaged tau is not needed downstream (design uses K/tau).
  return p;
}

double chooseCrossover(const Plant &p, double bwFrac, double kpMax) {
  if (!p.ok || p.tau <= 1e-9 || p.K <= 0.0)
    return 0.0;
  double wc = bwFrac / p.tau; // a fraction of the actuator bandwidth
  if (wc / p.K > kpMax)       // buzz cap: keep rate_kp under the knee
    wc = kpMax * p.K;
  return wc;
}

DesignGains designGains(const Plant &p, double wc) {
  DesignGains g;
  if (!p.ok || p.K <= 0.0 || wc <= 0.0)
    return g;
  g.wc = wc;
  g.rate_kp = wc / p.K;
  g.rate_kd = g.rate_kp * p.tau;
  g.rate_ki = 0.1 * wc * g.rate_kp;
  g.angle_kp = 0.25 * wc;
  return g;
}

} // namespace autotune
