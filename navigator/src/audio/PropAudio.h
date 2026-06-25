#pragma once

#include <array>
#include <atomic>
#include <thread>

// PropAudio — real-time propeller-noise synthesizer driven by the four motor
// angular speeds. Each motor contributes a blade-pass tone (pitch ∝ rpm) over
// a broadband whoosh, with overall loudness ∝ how hard the props are spinning,
// so the sim "spools up" as throttle rises. Runs on its own thread and streams
// S16 mono PCM to PulseAudio.
//
// PulseAudio is wired in only when libpulse-simple is found at configure time
// (HAVE_PULSE_SIMPLE); otherwise this is a silent stub so the build and the UI
// toggle still work. setMotors() is safe to call from the GUI thread.
class PropAudio {
 public:
  PropAudio();
  ~PropAudio();

  PropAudio(const PropAudio&) = delete;
  PropAudio& operator=(const PropAudio&) = delete;

  void setEnabled(bool on);
  void setMotors(const std::array<float, 4>& omega_rads);

 private:
  void run();

  std::atomic<bool>  alive_{false};
  std::atomic<bool>  enabled_{false};
  std::atomic<float> omega_[4];
  std::thread        thread_;
};
