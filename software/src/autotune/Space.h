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
  explicit Space(bool tuneYaw);

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
