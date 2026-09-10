#include "PropAudio.h"

#ifdef HAVE_PULSE_SIMPLE
#include <pulse/error.h>
#include <pulse/simple.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace {
constexpr int kRate = 44100;
constexpr int kFrames = 441;         // ~10 ms per write (paces the loop)
constexpr float kFullOmega = 900.0f; // rad/s mapped to full loudness
} // namespace

PropAudio::PropAudio() {
  for (auto &o : omega_)
    o.store(0.0f, std::memory_order_relaxed);
  alive_.store(true, std::memory_order_release);
  thread_ = std::thread([this] { run(); });
}

PropAudio::~PropAudio() {
  alive_.store(false, std::memory_order_release);
  if (thread_.joinable())
    thread_.join();
}

void PropAudio::setEnabled(bool on) {
  enabled_.store(on, std::memory_order_release);
}

void PropAudio::setMotors(const std::array<float, 4> &w) {
  for (int i = 0; i < 4; ++i)
    omega_[i].store(w[i], std::memory_order_relaxed);
}

void PropAudio::run() {
#ifdef HAVE_PULSE_SIMPLE
  pa_simple *s = nullptr;
  double phase[4] = {0, 0, 0, 0};
  float master = 0.0f;       // smoothed master gain (anti-click)
  uint32_t rng = 0x1234567u; // cheap LCG for the noise floor
  int16_t buf[kFrames];

  while (alive_.load(std::memory_order_acquire)) {
    if (!enabled_.load(std::memory_order_acquire)) {
      // Release the device while muted so we don't hold an audio sink idle.
      if (s) {
        pa_simple_free(s);
        s = nullptr;
        master = 0.0f;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      continue;
    }
    if (!s) {
      pa_sample_spec ss;
      ss.format = PA_SAMPLE_S16LE;
      ss.rate = kRate;
      ss.channels = 1;
      s = pa_simple_new(nullptr, "Vayu GCS", PA_STREAM_PLAYBACK, nullptr,
                        "props", &ss, nullptr, nullptr, nullptr);
      if (!s) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        continue;
      }
    }

    float w[4];
    float wmax = 0.0f;
    for (int i = 0; i < 4; ++i) {
      w[i] = omega_[i].load(std::memory_order_relaxed);
      if (w[i] < 0.0f)
        w[i] = 0.0f;
      if (w[i] > wmax)
        wmax = w[i];
    }
    const float targetGain = std::min(1.0f, wmax / kFullOmega);

    for (int f = 0; f < kFrames; ++f) {
      master += (targetGain - master) * 0.0008f; // ~28 ms time constant
      float v = 0.0f;
      for (int i = 0; i < 4; ++i) {
        // Blade-pass frequency: 2 blades => 2 rev/s = omega/pi [Hz].
        const double hz = static_cast<double>(w[i]) / M_PI;
        phase[i] += 2.0 * M_PI * hz / kRate;
        if (phase[i] > 2.0 * M_PI)
          phase[i] -= 2.0 * M_PI;
        const float wgt = w[i] / (wmax + 1e-3f); // faster motor = louder
        v += static_cast<float>(std::sin(phase[i]) +
                                0.4 * std::sin(2.0 * phase[i])) *
             wgt;
      }
      v *= 0.22f;
      // Broadband whoosh.
      rng = rng * 1664525u + 1013904223u;
      const float noise = (static_cast<int>(rng >> 9) / 4194304.0f) - 1.0f;
      v += noise * 0.18f;

      float out = v * master;
      out = std::clamp(out, -1.0f, 1.0f);
      buf[f] = static_cast<int16_t>(out * 28000.0f);
    }
    pa_simple_write(s, buf, sizeof(buf), nullptr); // blocks => real-time pace
  }
  if (s)
    pa_simple_free(s);
#else
  while (alive_.load(std::memory_order_acquire))
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
#endif
}
