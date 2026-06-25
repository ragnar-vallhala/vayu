#pragma once

#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>

// One CMD_SET_PID apply: (controller, axis) + gains. controller 0=angle / 1=rate,
// axis 0=roll / 1=pitch / 2=yaw. Mirrors the firmware pid_config schema.
struct PidSetCmd {
  int controller;
  int axis;
  float kp, ki, kd, kff;
};

// Map an autotuner result — the `params` name list and a `best_x` vector from
// tools/autotune (Space) — to the exact CMD_SET_PID applies that apply_gains()
// performs: the rate loop (ctrl 1) and angle-P loop (ctrl 0) for roll & pitch,
// plus the yaw rate loop (ctrl 1, axis 2) when --yaw was tuned. Missing params
// are skipped. Pure (no Qt widgets) so AT-1's apply path is unit-testable.
inline QVector<PidSetCmd> autotuneGainsToCommands(const QStringList &names,
                                                  const QVector<double> &bestX) {
  QHash<QString, double> v;
  for (int i = 0; i < names.size() && i < bestX.size(); ++i)
    v.insert(names[i], bestX[i]);

  const auto has = [&](const QString &k) { return v.contains(k); };
  const auto val = [&](const QString &k) {
    return float(v.value(k, 0.0));
  };

  QVector<PidSetCmd> cmds;
  for (int axis = 0; axis <= 1; ++axis) {  // roll, pitch
    if (has("rate_kp"))
      cmds.push_back({1, axis, val("rate_kp"), val("rate_ki"), val("rate_kd"),
                      0.0f});
    if (has("angle_kp"))
      cmds.push_back({0, axis, val("angle_kp"), 0.0f, 0.0f, 0.0f});
  }
  if (has("yaw_rate_kp"))  // yaw is rate-controlled only
    cmds.push_back({1, 2, val("yaw_rate_kp"), val("yaw_rate_ki"),
                    val("yaw_rate_kd"), 0.0f});
  return cmds;
}
