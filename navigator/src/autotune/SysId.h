#pragma once

#include <QVector>

// SysId — analytic PID design from a measured rate-loop response.
//
// The cost-based search (even with a rate-tracking term) can settle into the
// degenerate "don't move" minimum because angle tracking is dominated by the
// outer angle_kp, so a near-zero rate_kp is never penalised (see
// AUTOTUNE-ROLLPITCH-ANALYSIS Part B). This sidesteps the cost entirely: fit a
// physical plant model from a chirp/doublet response, then compute the gains by
// loop-shaping. There is no objective to game, so the inner loop gets a real,
// stiff gain by construction.
//
// Plant (rate loop, per axis): omega/u = K / (s (tau s + 1)) — the rigid-body
// integrator (torque -> rate) in series with the motor/actuator + gyro-filter
// lag. The integrator pole is PHYSICAL (known a priori), so we factor it out and
// fit only the first-order lag from u -> angular acceleration (d omega/dt); that
// is far more robust than fitting two poles from noisy closed-loop data.
//
// Firmware rate PID is parallel form with derivative-on-measurement
// (src/control/pid.c v_pid_update): C(s) = Kp + Ki/s + Kd s. The PD zero is
// placed on the actuator pole (Kd = Kp tau), which cancels the lag and leaves
// L(s) = Kp K / s — a clean integrator open loop. Then:
//   rate_kp  = wc / K           (sets the crossover: |L(j wc)| = 1)
//   rate_kd  = rate_kp * tau     (zero cancels the actuator pole -> ~90 deg PM)
//   rate_ki  = 0.1 * wc * rate_kp (integral knee a decade below wc; little PM cost)
//   angle_kp = 0.25 * wc         (cascade: outer loop ~4x slower than inner xover)
// wc (rad/s) is the single design knob, auto-picked below the actuator bandwidth.
namespace autotune {

struct Plant {
  bool   ok  = false;
  double K   = 0.0;   // DC gain of u -> angular accel  [ (rad/s^2) / u ]
  double tau = 0.0;   // actuator/filter lag time constant [s]
  double a   = 0.0;   // ARX pole (discrete), tau = -dt/ln(a)
  double b   = 0.0;   // ARX input gain (discrete)
  double r2  = 0.0;   // one-step prediction R^2 on accel (fit quality, 0..1)
  int    n   = 0;     // samples used
  double actuatorBwHz() const;  // (1/tau)/(2 pi); 0 if !ok
};

struct DesignGains {
  double rate_kp = 0.0, rate_ki = 0.0, rate_kd = 0.0, angle_kp = 0.0;
  double wc = 0.0;  // crossover actually used [rad/s] (may be buzz-capped)
};

// Fit the plant from a rate-loop input/output series (u = rate-PID output,
// omega = measured rate, dt = step [s]). Returns ok=false if the data is too
// short or the fit is non-physical (pole outside (0,1) -> no stable lag).
Plant identifyPlant(const QVector<double> &u, const QVector<double> &omega,
                    double dt);

// Average two same-axis plants (roll/pitch are physically identical on a
// symmetric quad; averaging the K/tau is more robust than trusting one noisy
// fit). Weights by fit R^2. Either may be !ok.
Plant averagePlants(const Plant &a, const Plant &b);

// Auto-pick the crossover: a fraction of the actuator bandwidth (kept well below
// 1/tau so the loop never chases the lag pole), then capped so rate_kp stays
// under the buzz knee. bwFrac in (0,1); kpMax is the rate_kp ceiling.
double chooseCrossover(const Plant &p, double bwFrac = 0.33,
                       double kpMax = 0.012);

// Loop-shape the gains for crossover wc against plant p (formulas above).
DesignGains designGains(const Plant &p, double wc);

}  // namespace autotune
