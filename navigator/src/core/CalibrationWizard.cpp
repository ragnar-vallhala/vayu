#include "CalibrationWizard.h"

QVector<CalibStep> CalibrationWizard::stepsFor(CalibMode mode) {
  switch (mode) {
  case CalibMode::Gyro:
    return {{CalibUpdateType::Progress, QStringLiteral("Hold still"),
             QStringLiteral("Keep the vehicle motionless and level.")}};
  case CalibMode::AccelBias:
    return {{CalibUpdateType::Upright, QStringLiteral("Level"),
             QStringLiteral("Place the vehicle level and still.")}};
  case CalibMode::Accel6Axis:
    // The SIX FACES — matching the firmware's active accel fit, the closed-form
    // 6-side solve (ACCEL_CALIB_METHOD == ACCEL_CALIB_SIXPOINT in
    // firmware/include/variables.h), which prompts the faces only. The edge /
    // corner holds are NOT listed: they are prompted solely by the ELLIPSOID
    // fit, whose off-diagonal misalignment terms need the shared-gravity poses
    // to become observable. Listing them under SIXPOINT stranded six rows
    // permanently pending and stalled the bar at 50% until the firmware's
    // terminal COMPLETE snapped it to 100%.
    //
    // The firmware does not advertise its fit method on the wire, so this list
    // tracks that compile-time switch by hand: flipping variables.h back to
    // ELLIPSOID means restoring the six Edge1..Edge6 steps here.
    //
    // Listed in the firmware's emission order so the initial (all-pending)
    // checklist reads in the real sequence. Advancement is order-agnostic
    // regardless (see onInstruction), so this order is display-only. Poses are
    // advisory (the fit classifies each hold by its dominant axis), so "roughly
    // this orientation, held still" is all the operator needs.
    return {
        {CalibUpdateType::Upright, QStringLiteral("Level"),
         QStringLiteral("Set level, upright.")},
        {CalibUpdateType::UpsideDown, QStringLiteral("Upside down"),
         QStringLiteral("Flip fully inverted.")},
        {CalibUpdateType::NoseUp, QStringLiteral("Nose up"),
         QStringLiteral("Stand the vehicle on its tail, nose pointing up.")},
        {CalibUpdateType::NoseDown, QStringLiteral("Nose down"),
         QStringLiteral("Nose pointing straight down.")},
        {CalibUpdateType::RightDown, QStringLiteral("Right side down"),
         QStringLiteral("Roll onto the right side.")},
        {CalibUpdateType::LeftDown, QStringLiteral("Left side down"),
         QStringLiteral("Roll onto the left side.")},
    };
  case CalibMode::Mag:
    return {{CalibUpdateType::FreeRot, QStringLiteral("Figure-8"),
             QStringLiteral("Rotate the vehicle slowly through a figure-8, "
                            "covering every axis.")}};
  case CalibMode::BoardLevel:
    return {{CalibUpdateType::BoardLevel, QStringLiteral("Frame level"),
             QStringLiteral("Set the FRAME level (props plane horizontal) and "
                            "hold still — corrects a tilted FC mount.")}};
  }
  return {};
}

void CalibrationWizard::begin(CalibMode mode) {
  m_mode = mode;
  m_steps = stepsFor(mode);
  m_current = -1;
  m_done = 0;
  m_active = true;
  m_complete = false;
}

void CalibrationWizard::reset() {
  m_steps.clear();
  m_current = -1;
  m_done = 0;
  m_active = false;
  m_complete = false;
}

double CalibrationWizard::progress() const {
  if (m_complete)
    return 1.0;
  if (m_steps.isEmpty())
    return 0.0;
  return double(m_done) / double(m_steps.size());
}

void CalibrationWizard::onInstruction(CalibUpdateType orient) {
  // Find the prompted orientation in this mode's catalog. A status we don't
  // model (e.g. Progress, or a pose this mode doesn't list) advances nothing.
  int j = -1;
  for (int i = 0; i < m_steps.size(); ++i) {
    if (m_steps[i].orient == orient) {
      j = i;
      break;
    }
  }
  if (j < 0)
    return;

  // Advance by ARRIVAL order, not by the catalog position: the firmware owns the
  // orientation sequence and the GCS must not assume it. (The old code set
  // m_done = catalog index, so e.g. UPSIDE_DOWN — last in the GCS list but 2nd
  // in firmware order — looked "almost done" and the next pose appeared to move
  // the bar backwards.) Bring the prompted pose into the next live slot so the
  // checklist reflects the true order and progress stays monotonic.
  if (j <= m_current)
    return; // a pose already visited was re-prompted — never go backwards
  const int nextSlot = m_current + 1; // m_current starts at -1 -> first slot 0
  if (j != nextSlot)
    m_steps.swapItemsAt(nextSlot, j);
  m_current = nextSlot;
  m_done = nextSlot; // every step ahead of the current one is complete
  m_active = true;
}

void CalibrationWizard::markComplete() {
  m_done = int(m_steps.size());
  m_current = m_steps.isEmpty() ? -1 : int(m_steps.size()) - 1;
  m_complete = true;
  m_active = false;
}
