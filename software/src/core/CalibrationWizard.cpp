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
      return {
          {CalibUpdateType::NoseUp, QStringLiteral("Nose up"),
           QStringLiteral("Stand the vehicle on its tail, nose pointing up.")},
          {CalibUpdateType::NoseDown, QStringLiteral("Nose down"),
           QStringLiteral("Nose pointing straight down.")},
          {CalibUpdateType::RightDown, QStringLiteral("Right side down"),
           QStringLiteral("Roll onto the right side.")},
          {CalibUpdateType::LeftDown, QStringLiteral("Left side down"),
           QStringLiteral("Roll onto the left side.")},
          {CalibUpdateType::Upright, QStringLiteral("Level"),
           QStringLiteral("Set level, upright.")},
          {CalibUpdateType::UpsideDown, QStringLiteral("Upside down"),
           QStringLiteral("Flip fully inverted.")},
      };
    case CalibMode::Mag:
      return {{CalibUpdateType::FreeRot, QStringLiteral("Figure-8"),
               QStringLiteral("Rotate the vehicle slowly through a figure-8, "
                              "covering every axis.")}};
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
  for (int i = 0; i < m_steps.size(); ++i) {
    if (m_steps[i].orient == orient) {
      m_current = i;
      m_done = i;  // every step before the current one is complete
      m_active = true;
      return;
    }
  }
  // Not an orientation step (e.g. Progress) — leave the wizard position alone.
}

void CalibrationWizard::markComplete() {
  m_done = int(m_steps.size());
  m_current = m_steps.isEmpty() ? -1 : int(m_steps.size()) - 1;
  m_complete = true;
  m_active = false;
}
