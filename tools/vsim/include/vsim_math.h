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

    Vec3 operator+(const Vec3& o) const { return {v[0]+o.v[0], v[1]+o.v[1], v[2]+o.v[2]}; }
    Vec3 operator-(const Vec3& o) const { return {v[0]-o.v[0], v[1]-o.v[1], v[2]-o.v[2]}; }
    Vec3 operator*(float s)       const { return {v[0]*s,     v[1]*s,     v[2]*s    }; }
    Vec3 operator/(float s)       const { return {v[0]/s,     v[1]/s,     v[2]/s    }; }

    Vec3& operator+=(const Vec3& o) { v[0]+=o.v[0]; v[1]+=o.v[1]; v[2]+=o.v[2]; return *this; }
    Vec3& operator-=(const Vec3& o) { v[0]-=o.v[0]; v[1]-=o.v[1]; v[2]-=o.v[2]; return *this; }

    static Vec3 crossProduct(const Vec3& a, const Vec3& b) {
        return { a.v[1]*b.v[2] - a.v[2]*b.v[1],
                 a.v[2]*b.v[0] - a.v[0]*b.v[2],
                 a.v[0]*b.v[1] - a.v[1]*b.v[0] };
    }
};

struct Quat {
    float w, xv, yv, zv;

    Quat() : w(1.0f), xv(0.0f), yv(0.0f), zv(0.0f) {}
    Quat(float w_, float x_, float y_, float z_) : w(w_), xv(x_), yv(y_), zv(z_) {}

    float scalar() const { return w; }
    float x()      const { return xv; }
    float y()      const { return yv; }
    float z()      const { return zv; }

    // Hamilton product. (q1 * q2) applies q2 first, then q1.
    Quat operator*(const Quat& q) const {
        return Quat(
            w*q.w  - xv*q.xv - yv*q.yv - zv*q.zv,
            w*q.xv + xv*q.w  + yv*q.zv - zv*q.yv,
            w*q.yv - xv*q.zv + yv*q.w  + zv*q.xv,
            w*q.zv + xv*q.yv - yv*q.xv + zv*q.w
        );
    }

    Quat conjugated() const { return Quat(w, -xv, -yv, -zv); }

    void normalize() {
        float n = std::sqrt(w*w + xv*xv + yv*yv + zv*zv);
        if (n > 0.0f) { w/=n; xv/=n; yv/=n; zv/=n; }
    }

    // v' = q * v * q^-1, with v promoted to a pure quaternion (0, v).
    // Closed-form expansion avoids two Quat*Quat multiplications.
    Vec3 rotatedVector(const Vec3& v) const {
        // u = (xv, yv, zv); s = w
        Vec3 u(xv, yv, zv);
        Vec3 t = Vec3::crossProduct(u, v) * 2.0f;
        return v + t * w + Vec3::crossProduct(u, t);
    }
};

}  // namespace vsim

#endif  // VSIM_MATH_H
