#pragma once

#include <QString>
#include <QVector>

#include "Types.h"  // CalibUpdateType

// Step state machine for the gated calibration wizard (FR / feature-matrix:
// "Calibration wizard (gated, fig-8)"). The firmware drives the flow by sending
// CALIB_UPDATE_<orientation> instructions; this models the expected ordered
// steps for a calibration mode and tracks which are done / current / pending so
// the UI can show a guided checklist + progress + a per-step animation. Pure
// (no widgets) so the flow logic is unit-tested headless.
enum class CalibMode {
  Gyro,        // hold still
  AccelBias,   // single level placement (legacy; firmware no longer bias-only)
  Accel6Axis,  // full 3x3: six faces + six edges/corners (pose-tolerant)
  Mag,         // free figure-8 rotation
};

struct CalibStep {
  CalibUpdateType orient;  // firmware instruction that activates this step
  QString label;           // short title (e.g. "Nose up")
  QString hint;            // what the operator should do
};

class CalibrationWizard {
public:
  void begin(CalibMode mode);
  void reset();

  CalibMode mode() const { return m_mode; }
  const QVector<CalibStep> &steps() const { return m_steps; }
  int stepCount() const { return int(m_steps.size()); }
  int currentIndex() const { return m_current; }  // -1 until the first prompt
  int doneCount() const { return m_done; }
  bool isActive() const { return m_active; }
  bool isComplete() const { return m_complete; }
  double progress() const;  // 0..1; 1.0 once complete

  // A firmware CALIB_UPDATE_<orient> instruction arrived: the prompted pose
  // becomes current and every pose instructed before it is marked done.
  // Advancement follows ARRIVAL order, not the catalog order, so the firmware
  // and GCS can never disagree on the sequence. Non-step statuses (e.g.
  // Progress) and poses this mode doesn't list are ignored for advancement.
  void onInstruction(CalibUpdateType orient);
  // Calibration finished successfully — mark every step done.
  void markComplete();

private:
  static QVector<CalibStep> stepsFor(CalibMode mode);

  CalibMode m_mode = CalibMode::Gyro;
  QVector<CalibStep> m_steps;
  int m_current = -1;
  int m_done = 0;
  bool m_active = false;
  bool m_complete = false;
};
