/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// vsim_math.h — minimal Vec3 / Quat for the vsim_d daemon.
//
// Hand-rolled instead of pulling in glm/Eigen/Qt so the daemon stays a
// tiny, dep-free executable. Surface mimics the QVector3D / QQuaternion
// methods the in-process port was using (x()/y()/z(), scalar(),
// rotatedVector, conjugated, normalize) so the physics .cpp files port
// across with mechanical search-and-replace.
//
// Conventions:
//   Quat layout is (w, x, y, z) — scalar first — matching what the
//   firmware-side `attitude_t` and the wire protocol both use.
//   Quaternions multiply Hamilton-style: q1 * q2 means "first apply
//   the rotation q2, then q1". rotatedVector(v) applies the quaternion
//   to a 3-vector, i.e. v' = q * v * q^-1.
#ifndef VSIM_MATH_H
#define VSIM_MATH_H

#include <cmath>

namespace vsim {

struct Vec3 {
  float v[3];

  Vec3() : v{0.0f, 0.0f, 0.0f} {}
  Vec3(float x, float y, float z) : v{x, y, z} {}

  float x() const { return v[0]; }
  float y() const { return v[1]; }
  float z() const { return v[2]; }

  void setX(float x) { v[0] = x; }
  void setY(float y) { v[1] = y; }
  void setZ(float z) { v[2] = z; }

  Vec3 operator+(const Vec3 &o) const {
    return {v[0] + o.v[0], v[1] + o.v[1], v[2] + o.v[2]};
  }
  Vec3 operator-(const Vec3 &o) const {
    return {v[0] - o.v[0], v[1] - o.v[1], v[2] - o.v[2]};
  }
  Vec3 operator*(float s) const { return {v[0] * s, v[1] * s, v[2] * s}; }
  Vec3 operator/(float s) const { return {v[0] / s, v[1] / s, v[2] / s}; }

  Vec3 &operator+=(const Vec3 &o) {
    v[0] += o.v[0];
    v[1] += o.v[1];
    v[2] += o.v[2];
    return *this;
  }
  Vec3 &operator-=(const Vec3 &o) {
    v[0] -= o.v[0];
    v[1] -= o.v[1];
    v[2] -= o.v[2];
    return *this;
  }

  static Vec3 crossProduct(const Vec3 &a, const Vec3 &b) {
    return {a.v[1] * b.v[2] - a.v[2] * b.v[1],
            a.v[2] * b.v[0] - a.v[0] * b.v[2],
            a.v[0] * b.v[1] - a.v[1] * b.v[0]};
  }
};

struct Quat {
  float w, xv, yv, zv;

  Quat() : w(1.0f), xv(0.0f), yv(0.0f), zv(0.0f) {}
  Quat(float w_, float x_, float y_, float z_)
      : w(w_), xv(x_), yv(y_), zv(z_) {}

  float scalar() const { return w; }
  float x() const { return xv; }
  float y() const { return yv; }
  float z() const { return zv; }

  // Hamilton product. (q1 * q2) applies q2 first, then q1.
  Quat operator*(const Quat &q) const {
    return Quat(w * q.w - xv * q.xv - yv * q.yv - zv * q.zv,
                w * q.xv + xv * q.w + yv * q.zv - zv * q.yv,
                w * q.yv - xv * q.zv + yv * q.w + zv * q.xv,
                w * q.zv + xv * q.yv - yv * q.xv + zv * q.w);
  }

  Quat conjugated() const { return Quat(w, -xv, -yv, -zv); }

  void normalize() {
    float n = std::sqrt(w * w + xv * xv + yv * yv + zv * zv);
    if (n > 0.0f) {
      w /= n;
      xv /= n;
      yv /= n;
      zv /= n;
    }
  }

  // v' = q * v * q^-1, with v promoted to a pure quaternion (0, v).
  // Closed-form expansion avoids two Quat*Quat multiplications.
  Vec3 rotatedVector(const Vec3 &v) const {
    // u = (xv, yv, zv); s = w
    Vec3 u(xv, yv, zv);
    Vec3 t = Vec3::crossProduct(u, v) * 2.0f;
    return v + t * w + Vec3::crossProduct(u, t);
  }
};

// Mat3 — row-major 3x3, the minimum needed to carry a full (possibly
// non-diagonal) inertia tensor through the rigid-body integrator:
// I*omega for the gyroscopic term and I^-1 for the angular accel.
// Hand-rolled, like Vec3/Quat, to keep the daemon dep-free.
struct Mat3 {
  // m[row*3 + col]
  float m[9];

  Mat3() : m{0, 0, 0, 0, 0, 0, 0, 0, 0} {}
  Mat3(float m00, float m01, float m02, float m10, float m11, float m12,
       float m20, float m21, float m22)
      : m{m00, m01, m02, m10, m11, m12, m20, m21, m22} {}

  static Mat3 identity() { return Mat3(1, 0, 0, 0, 1, 0, 0, 0, 1); }

  static Mat3 diagonal(float dx, float dy, float dz) {
    return Mat3(dx, 0, 0, 0, dy, 0, 0, 0, dz);
  }

  // Symmetric tensor from the six unique components (as a physical
  // inertia tensor: products of inertia enter off-diagonal with the
  // usual negative sign already folded into the caller's values).
  static Mat3 symmetric(float xx, float yy, float zz, float xy, float xz,
                        float yz) {
    return Mat3(xx, xy, xz, xy, yy, yz, xz, yz, zz);
  }

  float at(int r, int c) const { return m[r * 3 + c]; }

  Vec3 operator*(const Vec3 &v) const {
    return Vec3(m[0] * v.x() + m[1] * v.y() + m[2] * v.z(),
                m[3] * v.x() + m[4] * v.y() + m[5] * v.z(),
                m[6] * v.x() + m[7] * v.y() + m[8] * v.z());
  }

  Mat3 operator*(const Mat3 &o) const {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        r.m[i * 3 + j] = m[i * 3 + 0] * o.m[0 * 3 + j] +
                         m[i * 3 + 1] * o.m[1 * 3 + j] +
                         m[i * 3 + 2] * o.m[2 * 3 + j];
    return r;
  }

  float determinant() const {
    return m[0] * (m[4] * m[8] - m[5] * m[7]) -
           m[1] * (m[3] * m[8] - m[5] * m[6]) +
           m[2] * (m[3] * m[7] - m[4] * m[6]);
  }

  // Closed-form inverse via the adjugate. If the matrix is singular
  // (det ~ 0) returns identity scaled large so the resulting angular
  // accel is huge-but-finite rather than NaN -- a wildly-wrong
  // inertia is a config error the caller should never feed us, and a
  // NaN would silently poison the whole integrator.
  Mat3 inverse() const {
    const float det = determinant();
    if (std::fabs(det) < 1e-20f)
      return Mat3::identity();
    const float invd = 1.0f / det;
    Mat3 r;
    r.m[0] = (m[4] * m[8] - m[5] * m[7]) * invd;
    r.m[1] = (m[2] * m[7] - m[1] * m[8]) * invd;
    r.m[2] = (m[1] * m[5] - m[2] * m[4]) * invd;
    r.m[3] = (m[5] * m[6] - m[3] * m[8]) * invd;
    r.m[4] = (m[0] * m[8] - m[2] * m[6]) * invd;
    r.m[5] = (m[2] * m[3] - m[0] * m[5]) * invd;
    r.m[6] = (m[3] * m[7] - m[4] * m[6]) * invd;
    r.m[7] = (m[1] * m[6] - m[0] * m[7]) * invd;
    r.m[8] = (m[0] * m[4] - m[1] * m[3]) * invd;
    return r;
  }
};

} // namespace vsim

#endif // VSIM_MATH_H
