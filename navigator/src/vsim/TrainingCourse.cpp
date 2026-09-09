#include "TrainingCourse.h"

#include <algorithm>
#include <cmath>

namespace vsim {

namespace {
// Per-difficulty course shape. The course eases the pilot in and gets harder
// as it goes: the first gates are big and far apart (forgiving), then they
// shrink and close in on each other for tighter manoeuvres later. Difficulty
// sets how big/forgiving the start is and how aggressively it tightens.
struct DiffParams {
  int count;          // number of gates
  float spacing;      // forward (north) step to the FIRST gate, metres
  float spacingDecay; // step shrinks by this per gate (gates draw closer)
  float minStep;      // floor on the forward step, metres
  float baseRadius;   // first gate radius, metres (generous = easy)
  float shrink;       // radius removed per gate (rings get smaller)
  float baseAlt;      // altitude of the first gate, metres above ground
  float climb;        // altitude gained per gate, metres (up = -z)
  float latAmp;       // lateral (east) swing amplitude, metres
};

DiffParams paramsFor(TrainingCourse::Difficulty d) {
  // The course flies high off the deck (no ground-skimming rings), starts with
  // the first gate a long way out, and tightens gradually over many gates so
  // neither the ring size nor the spacing collapses in just a handful of gates.
  // spacingDecay/shrink are sized to ease from the opening value down to the
  // floor over (count-1) gates.
  switch (d) {
  // First-timers: big rings, far apart, almost straight, high and gentle.
  // 20 gates: first at 100 m, spacing eases 100 m -> 5 m, size eases gradually.
  case TrainingCourse::Easy:
    return {20, 100.0f, 5.0f, 5.0f, 3.6f, 0.16f, 22.0f, 0.8f, 1.5f};
  case TrainingCourse::Medium:
    return {24, 100.0f, 4.5f, 5.0f, 3.0f, 0.12f, 22.0f, 1.0f, 4.0f};
  case TrainingCourse::Hard:
    return {30, 90.0f, 3.2f, 4.0f, 2.4f, 0.07f, 20.0f, 1.3f, 7.0f};
  default:
    return {0, 0, 0, 0, 0, 0, 0, 0, 0};
  }
}
constexpr float kMinRadius = 0.6f;
} // namespace

void TrainingCourse::generate(Difficulty d) {
  diff_ = d;
  gates_.clear();
  resetProgress();
  if (d == Off)
    return;

  const DiffParams p = paramsFor(d);

  // Lay the centres out along +north (x), climbing in altitude (-z) with an
  // east (y) S-curve that widens as the course goes on. Floor is z=0; the
  // drone spawns there and climbs through the gates.
  QVector<QVector3D> centres;
  centres.reserve(p.count);
  float x = 0.0f;
  for (int i = 0; i < p.count; ++i) {
    // Gates start far apart and draw closer as the course goes on (tighter
    // manoeuvres later), never tighter than minStep.
    const float step = std::max(p.minStep, p.spacing - p.spacingDecay * i);
    x += step;
    // Lateral swing grows with progress; alternate sides for an S-weave.
    const float swing =
        p.latAmp * (0.4f + 0.6f * float(i) / std::max(1, p.count - 1));
    const float y = swing * std::sin(float(i) * 1.15f);
    // Fly high off the deck and climb gently, with a sensible ceiling.
    const float alt = p.baseAlt + p.climb * i; // metres above ground
    const float z = -std::min(alt, 50.0f);     // NED up is negative
    centres.push_back(QVector3D(x, y, z));
  }

  for (int i = 0; i < p.count; ++i) {
    RingGate g;
    g.center = centres[i];
    // Ring faces along the path tangent so the pilot flies through it head-on:
    // direction from the previous centre (or spawn) toward the next.
    const QVector3D prev = (i > 0) ? centres[i - 1] : QVector3D(0, 0, 0);
    const QVector3D next = (i + 1 < p.count) ? centres[i + 1] : centres[i];
    QVector3D tangent = (next - prev);
    if (tangent.lengthSquared() < 1e-6f)
      tangent = QVector3D(1, 0, 0);
    g.normal = tangent.normalized();
    g.radius = std::max(kMinRadius, p.baseRadius - p.shrink * i);
    gates_.push_back(g);
  }
}

void TrainingCourse::resetProgress() {
  activeIndex_ = 0;
  finished_ = false;
  havePrev_ = false;
}

bool TrainingCourse::advance(const QVector3D &dronePos) {
  if (!active() || finished_) {
    prevPos_ = dronePos;
    havePrev_ = true;
    return false;
  }
  if (!havePrev_) {
    prevPos_ = dronePos;
    havePrev_ = true;
    return false;
  }

  const RingGate &g = gates_[activeIndex_];
  const QVector3D from = prevPos_;
  prevPos_ = dronePos;

  // Signed distance of the previous and current position to the gate plane
  // (positive on the +normal / exit side).
  const float d0 = QVector3D::dotProduct(from - g.center, g.normal);
  const float d1 = QVector3D::dotProduct(dronePos - g.center, g.normal);

  bool passed = false;
  // Crossed the plane forward (entry side -> exit side) since last frame?
  if (d0 < 0.0f && d1 >= 0.0f) {
    const float denom = (d0 - d1);
    const float t = (denom != 0.0f) ? d0 / denom : 0.0f; // in [0,1]
    const QVector3D hit = from + t * (dronePos - from);  // point on the plane
    // Inside the ring iff the in-plane offset from the centre is within radius.
    const QVector3D fromCentre = hit - g.center;
    const QVector3D inPlane =
        fromCentre - QVector3D::dotProduct(fromCentre, g.normal) * g.normal;
    if (inPlane.length() <= g.radius)
      passed = true;
  }

  if (passed) {
    ++activeIndex_;
    if (activeIndex_ >= gates_.size()) {
      activeIndex_ = gates_.size(); // one past the end
      finished_ = true;
    }
  }
  return passed;
}

} // namespace vsim
