#pragma once

#include <QProcessEnvironment>
#include <QString>
#include <QVector>

// RtosEval — C++ port of tools/autotune/rtos_eval.py.
//
// Drives the in-process real-vaios SITL backend `vayu_sitl_rtos` (Phase 4,
// firmware/docs/plans/sitl-lockstep-sim.md) as a one-shot subprocess per gain set: it
// arms + flies a seeded roll/pitch/yaw step-doublet with the env-set PID gains
// and prints a `#RTOS-TUNE` line scoring each axis's rate-loop tracking
// correlation corr(rate_sp, rate_curr). Each rollout is ~0.04 s wall (~70x
// realtime) and **bit-deterministic** for a given (seed, gains) — so the GCS
// autotune search can use it as a fast, reproducible `eval(x)` instead of the
// realtime FIFO SitlStack.
//
// Unlike SitlStack this backend has NO geometry input: it tunes the firmware's
// reference quad (the binary links its own in-process vsim physics). The gain
// knobs it honours are rate_kp/ki/kd, angle_kp and yaw_rate_kp; any other Space
// param (gyro_lpf, yaw_rate_ki/kd, ...) has no env hook and is ignored.
namespace autotune {

struct RtosResult {
  bool   ok    = false;   // process ran and a cost was scored
  bool   starved = false; // a window was unscorable (axisCost nullopt) — like the
                          // realtime path's starved retry, NOT a divergence
  bool   diverged = false;// a per-axis cost hit kBig (|angle| > 80°)
  // EXACT realtime cost: axisCost(roll) + axisCost(pitch) [+ yawRateCost(yaw)],
  // the SAME Cost.cpp the SITL path runs (incl. its kBig divergence guard).
  double cost  = 0.0;
  double rollCost = 0.0, pitchCost = 0.0, yawCost = 0.0;  // angle-loop components
  // Roll/pitch RATE-loop components (AngleRate mode only; 0 in Angle mode).
  double rollRateCost = 0.0, pitchRateCost = 0.0;
  // Rate-tracking correlations from the #RTOS-TUNE line — kept for the log only
  // (the cost is the angle-IAE metric above, not these).
  double roll  = 0.0, pitch = 0.0, yaw = 0.0;
  double speedup = 0.0;   // sim-seconds / wall-seconds for this rollout
  // Roll-axis excitation window (angle setpoint vs measured, deg) for the live
  // response plot — the same trace the realtime path feeds it.
  QVector<double> respSp, respMeas;
  QString error;          // populated when !ok
};

class RtosEval {
public:
  // Cost function for a rollout:
  //   Angle      — the exact realtime cost: angle-loop IAE + overshoot + chatter
  //                for roll/pitch (+ yaw rate loop), with the kBig divergence
  //                guard. Scores the OUTER loop (what the realtime tuner uses).
  //   AngleRate  — Angle, PLUS a roll/pitch RATE-loop tracking term (mirrors the
  //                yaw rate cost) so the search also values a tracking,
  //                non-buzzy INNER loop (the analysis's Part-D rate term).
  enum class CostMode { Angle, AngleRate };

  // Gains pushed to the doublet (a value < 0 means "leave unset" → the binary
  // uses the firmware default / persisted pid.bin for that gain).
  struct Gains {
    double rate_kp = -1, rate_ki = -1, rate_kd = -1;
    double angle_kp = -1, yaw_rate_kp = -1;
  };

  // bin: path to vayu_sitl_rtos. suffix: VSIM_FIFO_SUFFIX isolation tag so a
  // concurrent world-tab/other sim can't collide on /tmp advert paths.
  RtosEval(QString bin, QString suffix) : m_bin(std::move(bin)), m_suffix(std::move(suffix)) {}
  ~RtosEval();

  bool binExists() const;

  void setCostMode(CostMode m) { m_costMode = m; }

  // Tune the loaded airframe instead of the reference quad: serialize a
  // vsim_ctl_geometry_t blob (mass + inertia + per-rotor layout, exactly the
  // bytes SitlStack pushes to vsim_d) to a temp file that every rollout points
  // the binary at via VAYU_RTOS_GEOMETRY. The backend applies it to both the
  // in-process physics and the firmware mix. Call once before rollout().
  // Returns false (and leaves the reference quad active) if the file can't be
  // written. `geomBytes` must be sizeof(vsim_ctl_geometry_t).
  bool setGeometry(const QByteArray &geomBytes, QString *err = nullptr);

  // Match the realtime doublet's excitation so the fast backend flies the same
  // maneuver the GCS configured (RolloutParams). The hold MUST be long enough
  // for the craft to reach steady state — a short hold makes the tracking cost
  // reward a near-motionless craft. holdS/retS/settleS in seconds.
  // waveform: 0 = step doublet, 1 = chirp (swept sine chirpF0→chirpF1 over hold).
  void setExcitation(int stepUs, double tetherK, double holdS, double retS,
                     double settleS, int waveform, double chirpF0,
                     double chirpF1) {
    m_stepUs = stepUs;
    m_tetherK = tetherK;
    m_holdS = holdS;
    m_retS = retS;
    m_settleS = settleS;
    m_waveform = waveform;
    m_chirpF0 = chirpF0;
    m_chirpF1 = chirpF1;
    m_haveExcite = true;
  }

  // Run one arm+doublet rollout and score it with the EXACT realtime cost
  // (axisCost/yawRateCost over the per-axis Sample windows the binary emits).
  // tuneYaw adds the yaw rate-loop term, mirroring the SITL path. Blocks until
  // the subprocess exits (or timeout).
  RtosResult rollout(const Gains &g, quint32 seed, bool tuneYaw,
                     int timeoutMs = 60000) const;

  // Raw rate-loop input/output series for System-ID (per axis): u = rate-PID
  // output, omega = measured rate, sp = rate setpoint, dt = step [s]. Run one
  // rollout with gains `g` (usually firmware-default — pass a default Gains — to
  // identify the open-ish plant) and the configured excitation (set a chirp via
  // setExcitation for a rich fit). Returns false (err set) on a harness failure.
  struct RawRate {
    QVector<double> rollU, rollOmega, rollSp;
    QVector<double> pitchU, pitchOmega, pitchSp;
    QVector<double> yawU, yawOmega, yawSp;  // yaw is rate-commanded (no angle loop)
    double dt = 0.001;
  };
  bool captureRate(const Gains &g, quint32 seed, RawRate &out,
                   QString *err = nullptr, int timeoutMs = 60000) const;

private:
  // Build the subprocess environment for one rollout (scenario, seed, isolation,
  // geometry, excitation, gains). Shared by rollout() and captureRate().
  QProcessEnvironment buildEnv(const Gains &g, quint32 seed,
                               const QString &winPath) const;

  QString m_bin;
  QString m_suffix;
  QString m_geomPath;  // temp vsim_ctl_geometry_t file (empty = reference quad)
  // Excitation (set via setExcitation; defaults match the backend's own).
  bool   m_haveExcite = false;
  int    m_stepUs = 1800;
  double m_tetherK = 30.0, m_holdS = 1.0, m_retS = 0.7, m_settleS = 0.6;
  int    m_waveform = 0;  // 0 = step, 1 = chirp
  double m_chirpF0 = 1.0, m_chirpF1 = 12.0;
  CostMode m_costMode = CostMode::Angle;
};

}  // namespace autotune
