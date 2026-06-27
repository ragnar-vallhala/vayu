/* mixer.h — N-motor control allocation with sequential desaturation.
 *
 * A unit-testable allocator that turns a desired wrench (roll/pitch/yaw torque
 * plus collective thrust) into per-motor commands:
 *
 *   1. nominal allocation:  motor = B+ · wrench        (geometry pseudo-inverse)
 *   2. sequential desaturation (airmode): when the demand does not fit [0,1],
 *      preserve roll/pitch torque by first moving collective thrust to make
 *      room, then sacrificing yaw — rather than scaling the whole attitude
 *      differential down.
 *   3. clamp to [0,1] and apply the idle floor.
 *
 * Pure C, no firmware/HAL deps, so it builds and tests on the host.
 * Geometry is fixed at config time; the pseudo-inverse is computed once in
 * mixer_set_geometry(), never per control cycle.
 */
#ifndef VAYU_CONTROL_MIXER_H
#define VAYU_CONTROL_MIXER_H

#include <stdint.h>
#include <stdbool.h>

#ifndef MIXER_MAX_MOTORS
#define MIXER_MAX_MOTORS 8
#endif

/* Wrench axes. THRUST is the collective; ROLL/PITCH/YAW are torque demands. */
enum { MIX_ROLL = 0, MIX_PITCH = 1, MIX_YAW = 2, MIX_THRUST = 3, MIX_NW = 4 };

typedef enum {
  /* Scale the whole roll/pitch/yaw differential uniformly so the worst motor
   * fits, holding the commanded collective. Attitude authority is reduced when
   * the limit bites. */
  MIXER_AIRMODE_DISABLED = 0,
  /* Preserve roll/pitch: desaturate by moving collective thrust, then yaw. */
  MIXER_AIRMODE_RP = 1,
  /* Preserve roll/pitch and yaw where headroom allows. */
  MIXER_AIRMODE_RPY = 2,
} mixer_airmode_t;

typedef struct {
  uint8_t n;                               /* motor count (<= MIXER_MAX_MOTORS) */
  float B[MIX_NW][MIXER_MAX_MOTORS];       /* effectiveness: wrench = B · motor */
  float Bpinv[MIXER_MAX_MOTORS][MIX_NW];   /* allocation:   motor = Bpinv · wrench */
  float fwd_scale;                         /* B·Bpinv = (1/fwd_scale)·I; realized = fwd_scale·B·motor */
  float idle_floor;                        /* per-motor minimum while armed */
  mixer_airmode_t airmode;
} mixer_t;

/* Build the effectiveness matrix from quad/multirotor geometry and compute the
 * normalised pseudo-inverse. pos_x/pos_y are arm positions (sign is what
 * matters), spin is +1 (CW) / -1 (CCW). Returns false on degenerate geometry.
 *
 * Normalisation: Bpinv is scaled so the THRUST column passes the collective
 * through ~1:1 per motor; for a symmetric quad-X this gives the mix
 * out_i = thr + roll*sign + pitch*sign + yaw*spin, i.e. unit per-axis gain. */
bool mixer_set_geometry(mixer_t *mx, const float *pos_x, const float *pos_y,
                        const int *spin, uint8_t n);

void mixer_set_airmode(mixer_t *mx, mixer_airmode_t mode);
void mixer_set_idle_floor(mixer_t *mx, float idle);

/* Allocate a wrench to motor commands in [0,1].
 *   w        : desired [roll, pitch, yaw, thrust]
 *   motor    : output, length mx->n, each in [idle_floor, 1]
 *   realized : optional (may be NULL) — the wrench actually delivered after
 *              clipping (= B · motor), for INDI/PID anti-windup feedback.
 */
void mixer_allocate(const mixer_t *mx, const float w[MIX_NW],
                    float *motor, float realized[MIX_NW]);

#endif /* VAYU_CONTROL_MIXER_H */
