#include "Rollout.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <unordered_map>

#include "Cost.h"

namespace autotune {

namespace {
// Rig rollouts pin translation, so the spawn height is dynamically irrelevant
// to the attitude search — EXCEPT that spawning ~5 cm above ground lets the
// ground-contact righting force (world ground_right_gain) kick the craft into a
// divergent spin (every rollout -> kBig), badly so on a low-inertia airframe.
// Spawn the rig well above ground so ground contact never triggers and the
// search is immune to the world's ground gains. Mirrors autotune.py RIG_RESET_Z.
constexpr float kRigResetZ = -2.0f;  // NED: 2 m up

// Pre-excitation: at sp=0 a settled craft sits within waitLevel's 6°. If it
// exceeds this over the brief pre-excite window it's spinning from a reset/arm
// transient (a vsim glitch), not the gains -> retry. Well above settled tilt,
// well below a divergence (80°). See the pre-excitation check in exciteOnce.
constexpr double kPreExciteTiltMax = 45.0;  // deg

void sleepS(double s) {
  std::this_thread::sleep_for(
      std::chrono::milliseconds(qint64(s * 1000.0 + 0.5)));
}

// True once the autotune Stop has been requested.
bool cancelled(const std::atomic<bool> *cancel) {
  return cancel && cancel->load(std::memory_order_relaxed);
}

// Cancel-aware sleep: wakes within ~10 ms of a Stop instead of blocking the full
// excitation/settle window, so autotune Stop is effectively instant.
void sleepSC(double s, const std::atomic<bool> *cancel) {
  qint64 remain = qint64(s * 1000.0 + 0.5);
  while (remain > 0 && !cancelled(cancel)) {
    const qint64 chunk = remain < 10 ? remain : 10;
    std::this_thread::sleep_for(std::chrono::milliseconds(chunk));
    remain -= chunk;
  }
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
// Fire the configured waveform on stick channel `ch` (0 roll / 1 pitch /
// 3 yaw), then return to centre + settle. Step = doublet; Chirp = swept sine
// f0→f1 over `hold`, sampled at 50 Hz.
void exciteAxis(SitlStack &stack, int ch, const RolloutParams &p,
                const std::atomic<bool> *cancel) {
  auto setCh = [&](int us) {
    if (ch == 0) stack.setRc(us);
    else if (ch == 1) stack.setRc(-1, us);
    else stack.setRc(-1, -1, -1, us);
  };
  if (p.excite == Excitation::Chirp) {
    const double amp = double(p.stepUs) - 1500.0;  // peak deflection µs
    const double T = (p.hold > 1e-3) ? p.hold : 1.0;
    const double dt = 0.02;  // 50 Hz update
    for (double t = 0.0; t < T && !cancelled(cancel); t += dt) {
      // Linear chirp: instantaneous f ramps f0→f1; phase is its integral.
      const double phase =
          2.0 * M_PI * (p.chirpF0 * t + (p.chirpF1 - p.chirpF0) * t * t / (2.0 * T));
      setCh(int(1500.0 + amp * std::sin(phase)));
      sleepSC(dt, cancel);
    }
    setCh(1500);
    sleepSC(p.ret, cancel);
  } else {  // Step doublet
    setCh(p.stepUs);
    sleepSC(p.hold, cancel);
    setCh(1500);
    sleepSC(p.ret, cancel);
  }
}

std::optional<double> exciteOnce(SitlStack &stack, bool tuneYaw,
                                 const RolloutParams &p,
                                 std::vector<Sample> *outResponse,
                                 const std::atomic<bool> *cancel) {
  stack.reset(p.seed, kRigResetZ);
  stack.setTestRig(true, float(p.tetherK), kRigResetZ);
  stack.setRc(1500, 1500, /*thr*/ -1, 1500, /*arm*/ -1, /*ch6*/ 1000);
  stack.clearSamples();
  stack.waitLevel(6.0, 2500, cancel);
  if (cancelled(cancel)) return std::nullopt;
  if (!stack.arm()) {
    std::fprintf(stderr, "[rollout] FAIL: arm not confirmed (no ARMED state in telemetry)\n");
    return std::nullopt;  // arm not confirmed -> retry
  }
  stack.setRc(-1, -1, /*thr*/ p.hover);
  sleepSC(p.settle, cancel);

  // Reject a PRE-EXCITATION transient. vsim intermittently injects a large body
  // rate at the reset/arm/hover transition (physically impossible from motors),
  // which runs away regardless of gains and would score as a phantom kBig. It's
  // rare per rollout, but at repeats>1 nearly every eval catches one and the
  // averaged cost pins at kBig -> the search "never converges". If the craft is
  // not level BEFORE we excite (sp=0, so it should be ~0°), it's a harness/
  // physics glitch, not the gains -> retry. (The Sample carries angle, not
  // roll/pitch rate, so a spinning craft is caught as a large off-level angle
  // over the brief check window.) Mirrors autotune.py PRE_EXCITE_SPIN_MAX.
  stack.clearSamples();
  sleepSC(0.1, cancel);
  for (const Sample &s : stack.snapshot()) {
    if (std::fabs(s.rollAngleCurr) > kPreExciteTiltMax ||
        std::fabs(s.pitchAngleCurr) > kPreExciteTiltMax) {
      std::fprintf(stderr,
                   "[rollout] FAIL: pre-excite spin (armed+level then ran away "
                   "BEFORE excitation): roll=%.1f pitch=%.1f deg\n",
                   s.rollAngleCurr, s.pitchAngleCurr);
      return std::nullopt;  // transient -> runRollout resets + retries
    }
  }

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
    if (cancelled(cancel)) {
      stack.disarm();
      return std::nullopt;
    }
    stack.clearSamples();
    exciteAxis(stack, ch, p, cancel);

    const std::vector<Sample> snap = stack.snapshot();
    // Capture the roll-axis window for the live response plot.
    if (outResponse && ch == 0) *outResponse = snap;
    std::optional<double> c =
        (ch == 3) ? yawRateCost(snap) : axisCost(snap, ch == 1 ? 1 : 0);
    if (!c.has_value()) {  // starved window -> the whole rollout retries
      std::fprintf(stderr,
                   "[rollout] FAIL: starved/unscorable window on ch=%d "
                   "(%zu samples)\n", ch, snap.size());
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
                                 const RolloutParams &p,
                                 std::vector<Sample> *outResponse,
                                 const std::atomic<bool> *cancel) {
  applyGains(stack, names, x, tuneYaw);
  for (int attempt = 0; attempt <= p.maxRetries && !cancelled(cancel);
       ++attempt) {
    std::optional<double> c = exciteOnce(stack, tuneYaw, p, outResponse, cancel);
    if (c.has_value())
      return c;
    if (cancelled(cancel))
      break;
    // Harness hiccup (arm-fail / starved): recover and retry.
    stack.disarm();
    stack.reset(p.seed, kRigResetZ);   // off the ground (see kRigResetZ)
    stack.clearSamples();
    stack.waitLevel(6.0, 3000, cancel);
    sleepSC(0.2, cancel);
  }
  return std::nullopt;  // no scorable rollout -> engine treats as divergence
}

}  // namespace autotune
