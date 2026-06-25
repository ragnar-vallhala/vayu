/**
 * @file ekf.h
 * @brief Tunables for the attitude error-state EKF (MEKF).
 *
 * The public entry points (ekf_init / ekf_reset / m_ekf_filter) and the
 * SF_EKF* selector live in est.h alongside the other fusion filters; this
 * header holds only the noise/initial-covariance tuning so it can be
 * overridden per build (define before including) without touching est.h.
 *
 * The filter is a multiplicative/error-state EKF over a nominal quaternion:
 *   6-state  (SF_EKF):            attitude error (3) + gyro bias (3)
 *   9-state  (SF_EKF_ACCEL_BIAS): the above       (6) + accel bias (3)
 * Accelerometer pins tilt (gravity direction), magnetometer pins heading
 * (yaw-only scalar update) — mirroring the decoupling the Mahony filter uses
 * so a noisy mag can never tip roll/pitch.
 *
 * NOTE on the 9-state accel bias: a constant accel bias and a small tilt are
 * physically indistinguishable while the airframe is STATIC (both just shift
 * the measured gravity vector). The bias only becomes observable under motion
 * that samples gravity against several orientations — so it is meaningfully
 * learned only in flight. On the ground the 9-state therefore behaves like the
 * 6-state with a near-zero, slowly-adapting bias; prefer SF_EKF (6-state)
 * unless you specifically want in-flight accel-bias learning.
 */
#ifndef VAYU_EST_EKF_H
#define VAYU_EST_EKF_H

/* Gravity magnitude (m/s^2) — used by the 9-state accel-bias measurement and
 * by both variants' accel-trust gate. */
#ifndef EKF_GRAVITY
#define EKF_GRAVITY 9.80665f
#endif

/* Process noise (per-second variances; the predict step scales by dt). */
#ifndef EKF_Q_ATT
#define EKF_Q_ATT 1.0e-4f /* attitude, driven by gyro white noise (rad^2/s) */
#endif
#ifndef EKF_Q_BG
#define EKF_Q_BG 1.0e-8f /* gyro-bias random walk ((rad/s)^2/s) */
#endif
#ifndef EKF_Q_BA
#define EKF_Q_BA 1.0e-4f /* accel-bias random walk ((m/s^2)^2/s) */
#endif

/* Measurement noise. */
#ifndef EKF_R_ACC_DIR
#define EKF_R_ACC_DIR 2.5e-2f /* 6-state: unit gravity-direction variance */
#endif
#ifndef EKF_R_ACC
#define EKF_R_ACC 9.0e-2f /* 9-state: accel variance (m/s^2)^2 */
#endif
#ifndef EKF_R_MAG_YAW
#define EKF_R_MAG_YAW 1.0e-2f /* heading measurement variance (rad^2) */
#endif

/* Initial covariance (diagonal). Roll/pitch are seeded from the first accel
 * sample so attitude P0 mainly covers the unknown initial heading. */
#ifndef EKF_P0_ATT
#define EKF_P0_ATT 2.5e-1f
#endif
#ifndef EKF_P0_BG
#define EKF_P0_BG 2.5e-3f
#endif
#ifndef EKF_P0_BA
#define EKF_P0_BA 2.5e-1f
#endif

/* Accel-trust gate: skip the accel correction when |a| deviates from gravity
 * by more than this (m/s^2), i.e. the body is accelerating and the accel is no
 * longer a clean gravity reference. */
#ifndef EKF_ACC_GATE
#define EKF_ACC_GATE 1.5f
#endif

#endif /* VAYU_EST_EKF_H */
