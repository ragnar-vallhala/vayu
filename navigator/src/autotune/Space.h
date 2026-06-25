#pragma once

#include <cstdint>  // int32_t before <stdlib.h> (glibc quirk)
#include <string>
#include <vector>

#include "Optimizer.h"  // Vec, Bounds

// The autotune parameter space — a C++ port of tools/autotune/autotune.py's
// Space (_BASE [+ _YAW]). Maps the optimizer's vector to named gains; the
// bounds reflect the measured plant (rate loop capped below the buzz knee,
// angle_kp wide). Pure — no Qt/sim — so it's unit-testable.
namespace autotune {

struct Param {
  std::string name;
  double lo, hi, seed;
};

class Space {
public:
  // fastRtos: restrict the space to the gains the in-process vayu_sitl_rtos
  // backend can actually set (rate kp/ki/kd, angle_kp [, yaw_rate_kp]) — it has
  // no env hook for gyro_lpf or the yaw ki/kd/lpf, so tuning them there would
  // burn budget on dimensions its cost is blind to. See AutotuneWorker::runRtos.
  explicit Space(bool tuneYaw, bool fastRtos = false);

  bool tuneYaw() const { return m_tuneYaw; }
  int dim() const { return int(m_params.size()); }
  std::vector<std::string> names() const;
  Bounds bounds() const;
  Vec seed() const;

private:
  bool m_tuneYaw;
  std::vector<Param> m_params;
};

}  // namespace autotune
