#include "MassProperties.h"

#include <cmath>

namespace vsim {

namespace {
// Eberly's per-axis subexpression helper: from the three vertex
// coordinates along one axis, produce the projection/face integrals.
inline void subexpr(float w0, float w1, float w2, float& f1, float& f2,
                    float& f3, float& g0, float& g1, float& g2) {
  const float t0 = w0 + w1;
  f1 = t0 + w2;
  const float t1 = w0 * w0;
  const float t2 = t1 + w1 * t0;
  f2 = t2 + w2 * f1;
  f3 = w0 * t1 + w1 * t2 + w2 * f2;
  g0 = f2 + w0 * (f1 + w0);
  g1 = f2 + w1 * (f1 + w1);
  g2 = f2 + w2 * (f1 + w2);
}
}  // namespace

MassProperties computeMassProperties(const std::vector<QVector3D>& p,
                                     float targetMass) {
  MassProperties out;
  if (p.size() < 3) return out;

  // integral order: [0]=1, [1..3]=x,y,z, [4..6]=x^2,y^2,z^2,
  // [7..9]=xy,yz,zx (the volume integrals of those monomials).
  double integ[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  const size_t triCount = p.size() / 3;
  for (size_t t = 0; t < triCount; ++t) {
    const QVector3D& v0 = p[3 * t + 0];
    const QVector3D& v1 = p[3 * t + 1];
    const QVector3D& v2 = p[3 * t + 2];

    const float x0 = v0.x(), y0 = v0.y(), z0 = v0.z();
    const float x1 = v1.x(), y1 = v1.y(), z1 = v1.z();
    const float x2 = v2.x(), y2 = v2.y(), z2 = v2.z();

    // Edges from v0, and their cross product (twice the face area normal).
    const float a1 = x1 - x0, b1 = y1 - y0, c1 = z1 - z0;
    const float a2 = x2 - x0, b2 = y2 - y0, c2 = z2 - z0;
    const float d0 = b1 * c2 - b2 * c1;
    const float d1 = a2 * c1 - a1 * c2;
    const float d2 = a1 * b2 - a2 * b1;

    float f1x, f2x, f3x, g0x, g1x, g2x;
    float f1y, f2y, f3y, g0y, g1y, g2y;
    float f1z, f2z, f3z, g0z, g1z, g2z;
    subexpr(x0, x1, x2, f1x, f2x, f3x, g0x, g1x, g2x);
    subexpr(y0, y1, y2, f1y, f2y, f3y, g0y, g1y, g2y);
    subexpr(z0, z1, z2, f1z, f2z, f3z, g0z, g1z, g2z);

    integ[0] += d0 * f1x;
    integ[1] += d0 * f2x;
    integ[2] += d1 * f2y;
    integ[3] += d2 * f2z;
    integ[4] += d0 * f3x;
    integ[5] += d1 * f3y;
    integ[6] += d2 * f3z;
    integ[7] += d0 * (y0 * g0x + y1 * g1x + y2 * g2x);
    integ[8] += d1 * (z0 * g0y + z1 * g1y + z2 * g2y);
    integ[9] += d2 * (x0 * g0z + x1 * g1z + x2 * g2z);
  }

  integ[0] *= 1.0 / 6.0;
  integ[1] *= 1.0 / 24.0;
  integ[2] *= 1.0 / 24.0;
  integ[3] *= 1.0 / 24.0;
  integ[4] *= 1.0 / 60.0;
  integ[5] *= 1.0 / 60.0;
  integ[6] *= 1.0 / 60.0;
  integ[7] *= 1.0 / 120.0;
  integ[8] *= 1.0 / 120.0;
  integ[9] *= 1.0 / 120.0;

  // integ[0] is the signed volume (density 1). Inside-out winding flips
  // its sign and that of every other integral; correct by negating all.
  if (integ[0] < 0.0) {
    for (double& v : integ) v = -v;
  }
  const double volume = integ[0];
  if (volume < 1e-12) return out;  // degenerate / non-closed

  // Center of mass (density-independent).
  const double cx = integ[1] / volume;
  const double cy = integ[2] / volume;
  const double cz = integ[3] / volume;

  // Inertia about the origin at density 1, then parallel-axis shift to
  // the CoM. Diagonal: Ixx = ∫(y²+z²); products carry the inertia-matrix
  // sign (ixy = -∫xy).
  double ixx = integ[5] + integ[6];
  double iyy = integ[4] + integ[6];
  double izz = integ[4] + integ[5];
  double ixy = -integ[7];
  double iyz = -integ[8];
  double izx = -integ[9];

  ixx -= volume * (cy * cy + cz * cz);
  iyy -= volume * (cz * cz + cx * cx);
  izz -= volume * (cx * cx + cy * cy);
  ixy += volume * cx * cy;
  iyz += volume * cy * cz;
  izx += volume * cz * cx;

  // Scale density so the tessellated solid weighs targetMass.
  const double density = (targetMass > 0.0f) ? (targetMass / volume) : 0.0;

  out.volume = static_cast<float>(volume);
  out.mass = targetMass;
  out.com = QVector3D(static_cast<float>(cx), static_cast<float>(cy),
                      static_cast<float>(cz));
  out.ixx = static_cast<float>(ixx * density);
  out.iyy = static_cast<float>(iyy * density);
  out.izz = static_cast<float>(izz * density);
  out.ixy = static_cast<float>(ixy * density);
  out.ixz = static_cast<float>(izx * density);
  out.iyz = static_cast<float>(iyz * density);
  out.valid = true;
  return out;
}

}  // namespace vsim
