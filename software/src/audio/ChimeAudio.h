#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

// ChimeAudio — one-shot alert chimes (arm / disarm / failsafe) synthesized and
// streamed to PulseAudio, mirroring PropAudio's S16-mono / pa_simple approach.
// Runs on its own worker thread; play() is safe to call from the GUI thread and
// returns immediately (the tone is rendered + written on the worker).
//
// Like PropAudio, the PulseAudio path is compiled in only when libpulse-simple
// is found (HAVE_PULSE_SIMPLE); otherwise this is a silent stub so the build and
// the Settings toggle still work.
class ChimeAudio {
 public:
  enum class Kind { Arm, Disarm, Failsafe };

  ChimeAudio();
  ~ChimeAudio();

  ChimeAudio(const ChimeAudio&) = delete;
  ChimeAudio& operator=(const ChimeAudio&) = delete;

  void setEnabled(bool on);
  void play(Kind k);  // no-op while disabled

 private:
  void run();

  std::atomic<bool> alive_{false};
  std::atomic<bool> enabled_{false};
  std::mutex mtx_;
  std::condition_variable cv_;
  std::deque<Kind> queue_;
  std::thread thread_;
};
