#include "Rollout.h"

#include <chrono>
#include <thread>
#include <unordered_map>

#include "Cost.h"

namespace autotune {

namespace {
void sleepS(double s) {
  std::this_thread::sleep_for(
      std::chrono::milliseconds(qint64(s * 1000.0 + 0.5)));
}
}  // namespace

void applyGains(SitlStack &stack, const std::vector<std::string> &names,
                const Vec &x, bool tuneYaw) {
  std::unordered_map<std::string, double> v;
  for (size_t i = 0; i < names.size() && i < x.size(); ++i)
    v[names[i]] = x[i];
  auto val = [&](const char *k) {
    auto it = v.find(k);
    return it == v.end() ? 0.0f : float(it->second);
  };

  const float rateKp = val("rate_kp"), rateKi = val("rate_ki"),
              rateKd = val("rate_kd"), angleKp = val("angle_kp"),
              gyroLpf = val("gyro_lpf");
  for (int axis = 0; axis <= 1; ++axis) {  // roll, pitch
    stack.setPid(/*rate*/ 1, axis, rateKp, rateKi, rateKd, 0.0f);
    sleepS(0.02);
    stack.setPid(/*angle*/ 0, axis, angleKp, 0.0f, 0.0f, 0.0f);
    sleepS(0.02);
    stack.setGyroLpf(axis, gyroLpf);
    sleepS(0.02);
  }
  if (tuneYaw) {  // yaw RATE loop only
    stack.setPid(1, 2, val("yaw_rate_kp"), val("yaw_rate_ki"),
                 val("yaw_rate_kd"), 0.0f);
    sleepS(0.02);
    stack.setGyroLpf(2, val("yaw_gyro_lpf"));
    sleepS(0.02);
  }
  sleepS(0.05);
}

namespace {

// One excitation attempt. Returns nullopt on arm-fail / starved telemetry
// (retry), kBig on divergence, else the summed per-axis cost.
std::optional<double> exciteOnce(SitlStack &stack, bool tuneYaw,
                                 const RolloutParams &p) {
  stack.reset(p.seed);
  stack.setTestRig(true, float(p.tetherK));
  stack.setRc(1500, 1500, /*thr*/ -1, 1500, /*arm*/ -1, /*ch6*/ 1000);
  stack.clearSamples();
  stack.waitLevel();
  if (!stack.arm())
    return std::nullopt;  // arm not confirmed -> retry
  stack.setRc(-1, -1, /*thr*/ p.hover);
  sleepS(p.settle);

  struct Axis {
    int rc;  // RC channel index for setRc
    int idx;
  };
  // roll=ch0, pitch=ch1, yaw=ch3.
  std::vector<int> axes = {0, 1};
  if (tuneYaw)
    axes.push_back(3);

  double cost = 0.0;
  for (int ch : axes) {
    stack.clearSamples();
    if (ch == 0) stack.setRc(p.stepUs);
    else if (ch == 1) stack.setRc(-1, p.stepUs);
    else stack.setRc(-1, -1, -1, p.stepUs);
    sleepS(p.hold);
    if (ch == 0) stack.setRc(1500);
    else if (ch == 1) stack.setRc(-1, 1500);
    else stack.setRc(-1, -1, -1, 1500);
    sleepS(p.ret);

    const std::vector<Sample> snap = stack.snapshot();
    std::optional<double> c =
        (ch == 3) ? yawRateCost(snap) : axisCost(snap, ch == 1 ? 1 : 0);
    if (!c.has_value()) {  // starved window -> the whole rollout retries
      stack.disarm();
      return std::nullopt;
    }
    cost += *c;
  }
  stack.disarm();
  return cost;
}

}  // namespace

std::optional<double> runRollout(SitlStack &stack,
                                 const std::vector<std::string> &names,
                                 const Vec &x, bool tuneYaw,
                                 const RolloutParams &p) {
  applyGains(stack, names, x, tuneYaw);
  for (int attempt = 0; attempt <= p.maxRetries; ++attempt) {
    std::optional<double> c = exciteOnce(stack, tuneYaw, p);
    if (c.has_value())
      return c;
    // Harness hiccup (arm-fail / starved): recover and retry.
    stack.disarm();
    stack.reset(p.seed);
    stack.clearSamples();
    stack.waitLevel(6.0, 3000);
    sleepS(0.2);
  }
  return std::nullopt;  // no scorable rollout -> engine treats as divergence
}

}  // namespace autotune
