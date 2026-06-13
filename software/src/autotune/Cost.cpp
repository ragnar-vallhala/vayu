#include "Cost.h"

#include <algorithm>
#include <cmath>

namespace autotune {

double chatter(const std::vector<double> &xs) {
  if (xs.size() < 2)
    return 0.0;
  double s = 0.0;
  for (size_t i = 1; i < xs.size(); ++i)
    s += std::fabs(xs[i] - xs[i - 1]);
  return s / double(xs.size());
}

namespace {

// Pull the (sp, curr, out) trace for one angle axis (0=roll, 1=pitch).
void angleTrace(const std::vector<Sample> &s, int axis, std::vector<double> &sp,
                std::vector<double> &cur, std::vector<double> &out) {
  sp.clear();
  cur.clear();
  out.clear();
  for (const Sample &k : s) {
    if (axis == 0) {
      sp.push_back(k.rollAngleSp);
      cur.push_back(k.rollAngleCurr);
      out.push_back(k.rollOut);
    } else {
      sp.push_back(k.pitchAngleSp);
      cur.push_back(k.pitchAngleCurr);
      out.push_back(k.pitchOut);
    }
  }
}

double meanAbsErr(const std::vector<double> &sp, const std::vector<double> &cur) {
  double s = 0.0;
  for (size_t i = 0; i < sp.size(); ++i)
    s += std::fabs(sp[i] - cur[i]);
  return s / double(sp.size());
}

double maxAbs(const std::vector<double> &v) {
  double m = 0.0;
  for (double x : v)
    m = std::max(m, std::fabs(x));
  return m;
}

}  // namespace

std::optional<double> axisCost(const std::vector<Sample> &samples, int axis) {
  std::vector<double> sp, cur, out;
  angleTrace(samples, axis, sp, cur, out);
  if (sp.size() < 5)
    return std::nullopt;  // telemetry starved -> retry, not a divergence

  for (double c : cur)
    if (!std::isfinite(c) || std::fabs(c) > 80.0)
      return kBig;  // genuine divergence / failsafe

  const double iae = meanAbsErr(sp, cur);
  const double target = std::max(1.0, maxAbs(sp));
  const double overshoot = std::max(0.0, maxAbs(cur) - target) / target;
  const double chatterPen =
      kChatterScale * std::max(0.0, chatter(out) - kChatterThresh);
  return iae + 3.0 * overshoot + chatterPen;
}

std::optional<double> yawRateCost(const std::vector<Sample> &samples) {
  std::vector<double> sp, cur, out;
  for (const Sample &k : samples) {
    sp.push_back(k.yawRateSp);
    cur.push_back(k.yawRateCurr);
    out.push_back(k.yawOut);
  }
  if (sp.size() < 5)
    return std::nullopt;

  for (double c : cur)
    if (!std::isfinite(c) || std::fabs(c) > 2000.0)
      return kBig;  // gyro saturating / divergence

  const double iae = meanAbsErr(sp, cur);
  const double target = std::max(20.0, maxAbs(sp));
  const double iaeN = (iae / target) * 21.0;  // fraction-of-command -> angle scale
  const double overshoot = std::max(0.0, maxAbs(cur) - target) / target;
  const double chatterPen =
      kChatterScale * std::max(0.0, chatter(out) - kChatterThresh);
  return iaeN + 3.0 * overshoot + chatterPen;
}

}  // namespace autotune
