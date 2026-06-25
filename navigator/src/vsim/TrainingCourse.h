#pragma once

#include <QVector>
#include <QVector3D>

namespace vsim {

// A single glowing halo gate the pilot must fly through. The course is laid
// out in the NED world frame (z down, up = -z); ground/floor is z=0.
struct RingGate {
  QVector3D center;   // gate centre, NED world metres
  QVector3D normal;   // unit; the intended direction of travel through the ring
  float     radius;   // metres
};

// Procedurally generates a sequence of ring gates of increasing difficulty
// (smaller, farther apart, more lateral/vertical swing as you progress) and
// tracks the pilot's progress through them. Pure gameplay logic — no Qt
// widgets, no GL. The renderer draws gates_/activeIndex_; SimulatorWidget feeds
// it the live drone position each frame via advance().
class TrainingCourse {
 public:
  enum Difficulty { Off = 0, Easy = 1, Medium = 2, Hard = 3 };

  // (Re)build the gate list for a difficulty and reset progress. Off clears it.
  void generate(Difficulty d);
  // Back to the first gate without rebuilding the layout.
  void resetProgress();

  bool active()   const { return diff_ != Off && !gates_.empty(); }
  bool finished() const { return finished_; }

  // Feed the live drone position each sim frame. Returns true on the frame a
  // gate is first cleared (so the caller can play a cue / log), and advances
  // the active gate. Uses segment-vs-plane crossing so fast passes still count.
  bool advance(const QVector3D& dronePos);

  const QVector<RingGate>& gates() const { return gates_; }
  int  activeIndex() const { return activeIndex_; }   // gate to aim for next
  int  passedCount() const { return activeIndex_; }
  int  total()       const { return int(gates_.size()); }
  Difficulty difficulty() const { return diff_; }

 private:
  Difficulty        diff_ = Off;
  QVector<RingGate> gates_;
  int       activeIndex_ = 0;
  bool      finished_    = false;
  bool      havePrev_    = false;
  QVector3D prevPos_;
};

}  // namespace vsim
