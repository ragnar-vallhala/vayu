#ifndef CALIB_ELLIPSOID_H
#define CALIB_ELLIPSOID_H

/* Sensor-agnostic ellipsoid fit for sphere-constrained 3-axis sensors
 * (magnetometer hard/soft-iron; accelerometer offset + scale + misalignment).
 * Pure fixed-size float linear algebra — no heap, no RTOS, no sensor deps —
 * so it is shared by every calibration target and is host-unit-testable.
 *
 * Usage: the caller accumulates the 9-parameter quadric normal equations over
 * its samples. For each sample (x,y,z) that should lie on a sphere, form
 *   r = [x^2, y^2, z^2, 2yz, 2xz, 2xy, 2x, 2y, 2z]
 * and add r to t[9] and the outer product r*r^T to S[81] (row-major), with an
 * implicit target of 1. Scale the raw samples by 1/radius first (e.g. 1/g for
 * accel, 1/|B| for mag) so the matrix entries (which span x^4 .. x) stay
 * well-conditioned in float32.
 *
 * calib_fit_ellipsoid() then recovers, for the model  x^T Q x + 2 u^T x = 1:
 *   offset c = -Q^-1 u               (the ellipsoid centre — hard iron / bias)
 *   soft   M = detQ^(-1/6) * Q^(1/2) (maps the ellipsoid back onto a sphere)
 * The corrected vector is  M*(raw_scaled - offset)  with unit-ish norm; the
 * caller scales offset back by radius and applies M directly to (raw - offset).
 */

/* S[81], t[9] are CONSUMED (destroyed) by the solve. offset[3] and soft[9]
 * (row-major 3x3) receive the result. Returns 0 on success; -1 if the system
 * is singular or Q is not positive-definite (degenerate / planar data) — in
 * which case the caller should keep the previous calibration. */
int calib_fit_ellipsoid(float S[81], float t[9], float offset[3], float soft[9]);

#endif /* CALIB_ELLIPSOID_H */
