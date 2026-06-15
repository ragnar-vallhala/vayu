#pragma once

#include <atomic>
#include <cstdint>  // int32_t before <stdlib.h> (glibc quirk)
#include <optional>
#include <string>
#include <vector>

#include "Cost.h"       // Sample (live response window)
#include "Optimizer.h"  // Vec
#include "SitlStack.h"

// One autotune rollout against a live SITL stack — a C++ port of
// tools/autotune/autotune.py rollout()/_excite_once()/apply_gains(): push the
// candidate gains, arm on the tuning rig, fire a step doublet per axis, and
// score the control-loop telemetry window with Cost. Returns the scalar cost,
// or nullopt when the rollout couldn't score (arm never confirmed / telemetry
// starved) so the caller retries instead of recording a phantom divergence.
namespace autotune {

// Excitation waveform fired on each axis: a single doublet (step) or a swept
// sinusoid (chirp f0→f1 over `hold`), which probes a band of frequencies.
enum class Excitation { Step, Chirp };

struct RolloutParams {
  int stepUs = 1800;     // excitation amplitude (peak stick µs; 1500 = centre)
  double hold = 1.0;     // s at the step / chirp sweep duration
  double ret = 0.7;      // s settle window after release (chatter shows here)
  double settle = 0.6;   // s after spin-up before exciting
  int hover = 1500;      // hover throttle stick
  double tetherK = 0.0;  // >0 = soft rig (estimator-aware)
  quint32 seed = 0;      // deterministic sensor-noise seed
  int maxRetries = 2;
  Excitation excite = Excitation::Step;
  double chirpF0 = 1.0;   // Hz, chirp sweep start
  double chirpF1 = 12.0;  // Hz, chirp sweep end
};

// Apply the optimizer vector x (named per Space) to the stack: rate + angle-P
// loops and gyro LPF for roll/pitch, plus the yaw rate loop when tuned.
void applyGains(SitlStack &stack, const std::vector<std::string> &names,
                const Vec &x, bool tuneYaw);

// Apply gains then run one scored excitation (with internal retries). When
// outResponse is non-null it receives the roll-axis excitation window (for the
// live response plot).
// `cancel` (optional): when it flips true the rollout aborts within ~10 ms and
// returns nullopt, so autotune Stop is effectively instant.
std::optional<double> runRollout(SitlStack &stack,
                                 const std::vector<std::string> &names,
                                 const Vec &x, bool tuneYaw,
                                 const RolloutParams &p,
                                 std::vector<Sample> *outResponse = nullptr,
                                 const std::atomic<bool> *cancel = nullptr);

}  // namespace autotune
