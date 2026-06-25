#include "ChimeAudio.h"

#ifdef HAVE_PULSE_SIMPLE
#include <pulse/error.h>
#include <pulse/simple.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace {
constexpr int kRate = 44100;

#ifdef HAVE_PULSE_SIMPLE
// Append a `freq`-Hz tone for `ms` with a short raised-cosine fade in/out so the
// note doesn't click. amp is 0..1.
void appendTone(std::vector<int16_t>& buf, double freq, int ms, float amp) {
  const int n = kRate * ms / 1000;
  const int fade = std::min(n / 2, kRate / 500);  // ~2 ms ramp
  for (int i = 0; i < n; ++i) {
    double env = 1.0;
    if (i < fade)
      env = double(i) / fade;
    else if (i >= n - fade)
      env = double(n - i) / fade;
    const double s = std::sin(2.0 * M_PI * freq * i / kRate);
    buf.push_back(static_cast<int16_t>(s * env * amp * 30000.0));
  }
}

void appendSilence(std::vector<int16_t>& buf, int ms) {
  buf.insert(buf.end(), static_cast<size_t>(kRate * ms / 1000), 0);
}

// Build the PCM for one chime. Arm rises, disarm falls, failsafe is an urgent
// repeated beep.
std::vector<int16_t> synth(ChimeAudio::Kind k) {
  std::vector<int16_t> b;
  switch (k) {
    case ChimeAudio::Kind::Arm:
      appendTone(b, 660.0, 110, 0.5f);
      appendTone(b, 990.0, 150, 0.5f);
      break;
    case ChimeAudio::Kind::Disarm:
      appendTone(b, 990.0, 110, 0.5f);
      appendTone(b, 660.0, 150, 0.5f);
      break;
    case ChimeAudio::Kind::Failsafe:
      for (int i = 0; i < 3; ++i) {
        appendTone(b, 880.0, 90, 0.65f);
        appendSilence(b, 70);
      }
      break;
  }
  return b;
}
#endif  // HAVE_PULSE_SIMPLE
}  // namespace

ChimeAudio::ChimeAudio() {
  alive_.store(true, std::memory_order_release);
  thread_ = std::thread([this] { run(); });
}

ChimeAudio::~ChimeAudio() {
  alive_.store(false, std::memory_order_release);
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void ChimeAudio::setEnabled(bool on) {
  enabled_.store(on, std::memory_order_release);
}

void ChimeAudio::play(Kind k) {
  if (!enabled_.load(std::memory_order_acquire)) return;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (queue_.size() > 4) return;  // don't pile up if something spams events
    queue_.push_back(k);
  }
  cv_.notify_one();
}

void ChimeAudio::run() {
  while (alive_.load(std::memory_order_acquire)) {
    Kind k;
    {
      std::unique_lock<std::mutex> lk(mtx_);
      cv_.wait(lk, [this] {
        return !queue_.empty() || !alive_.load(std::memory_order_acquire);
      });
      if (!alive_.load(std::memory_order_acquire)) return;
      k = queue_.front();
      queue_.pop_front();
    }
#ifdef HAVE_PULSE_SIMPLE
    if (!enabled_.load(std::memory_order_acquire)) continue;
    const std::vector<int16_t> buf = synth(k);
    if (buf.empty()) continue;
    pa_sample_spec ss;
    ss.format = PA_SAMPLE_S16LE;
    ss.rate = kRate;
    ss.channels = 1;
    pa_simple* s = pa_simple_new(nullptr, "Vayu GCS", PA_STREAM_PLAYBACK, nullptr,
                                 "alert", &ss, nullptr, nullptr, nullptr);
    if (!s) continue;
    pa_simple_write(s, buf.data(), buf.size() * sizeof(int16_t), nullptr);
    pa_simple_drain(s, nullptr);
    pa_simple_free(s);
#endif
  }
}
