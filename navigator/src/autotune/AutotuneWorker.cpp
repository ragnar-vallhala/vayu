#include "AutotuneWorker.h"

#include <algorithm>
#include <cmath>

#include "Cost.h"       // autotune::kBig (divergence threshold)
#include "Optimizer.h"  // autotune::Vec
#include "RtosEval.h"   // fast in-process backend
#include "Space.h"
#include "SysId.h"      // analytic plant-fit + gain design

AutotuneWorker::AutotuneWorker(Params p, QObject *parent)
    : QObject(parent), m_p(std::move(p)) {}

void AutotuneWorker::cancel() {
  m_cancel = true;
  std::lock_guard<std::mutex> lk(m_engMtx);
  if (m_engine)
    m_engine->cancel();
}

void AutotuneWorker::run() {
  if (m_p.sysId) {
    runSysId();
    return;
  }
  if (m_p.fastRtos) {
    runRtos();
    return;
  }
  SitlStack stack(m_p.sitl);
  emit log(QStringLiteral("launching SITL stack (vsim_d + vayu_sitl) ..."));
  QString err;
  if (!stack.start(&err)) {
    emit failed(err);
    emit done();
    return;
  }
  emit log(QStringLiteral("SITL up — tuning %1 params with '%2' (budget %3)")
               .arg(autotune::Space(m_p.tuneYaw).dim())
               .arg(m_p.optimizer)
               .arg(m_p.budget));

  const autotune::Space space(m_p.tuneYaw);
  const std::vector<std::string> names = space.names();

  AutotuneEngine engine(
      m_p.tuneYaw, m_p.optimizer, m_p.budget, m_p.optSeed,
      [&](const QVector<double> &xq) -> std::optional<double> {
        autotune::Vec x;
        x.reserve(xq.size());
        for (double v : xq)
          x.push_back(v);
        // Average `repeats` rollouts over distinct noise realizations
        // (sim_seed + i), mirroring autotune.py: include divergences (kBig) in
        // the mean, drop only harness failures (nullopt). This smooths the
        // noisy SITL cost so the search isn't misled by one unlucky rollout.
        double sum = 0.0;
        int scored = 0;
        const int reps = std::max(1, m_p.repeats);
        std::vector<autotune::Sample> resp;  // last rollout's roll-axis window
        for (int i = 0; i < reps && !m_cancel.load(); ++i) {
          autotune::RolloutParams rp = m_p.rollout;
          rp.seed = m_p.rollout.seed + quint32(i);
          // Capture the response window only on the final repeat (for the plot).
          std::vector<autotune::Sample> *out = (i == reps - 1) ? &resp : nullptr;
          // &m_cancel makes the rollout abort within ~10 ms of Stop.
          if (auto c = autotune::runRollout(stack, names, x, m_p.tuneYaw, rp, out,
                                            &m_cancel)) {
            sum += *c;
            ++scored;
          }
        }
        // Push the latest excitation window to the live response plot.
        if (!resp.empty()) {
          QVector<double> sp, meas;
          sp.reserve(int(resp.size()));
          meas.reserve(int(resp.size()));
          for (const autotune::Sample &s : resp) {
            sp.push_back(s.rollAngleSp);
            meas.push_back(s.rollAngleCurr);
          }
          emit responseWindow(sp, meas);
        }
        // State + POSE log so the operator (and offline diagnosis) can see WHAT
        // each eval did, not just the cost: the gains tried, the mean cost, and
        // the peak attitude the firmware reported during the roll-axis window
        // (rollAngleCurr/pitchAngleCurr are the estimated pose the controller
        // sees and what diverges; yawRateCurr is the yaw body rate). A cost
        // >= kBig means the craft tumbled past 80° on some axis.
        {
          double mxRoll = 0.0, mxPitch = 0.0, mxYawRate = 0.0;
          for (const autotune::Sample &s : resp) {
            mxRoll = std::max(mxRoll, std::fabs(s.rollAngleCurr));
            mxPitch = std::max(mxPitch, std::fabs(s.pitchAngleCurr));
            mxYawRate = std::max(mxYawRate, std::fabs(s.yawRateCurr));
          }
          QStringList gs;
          for (int gi = 0; gi < int(names.size()) && gi < int(x.size()); ++gi)
            gs << QStringLiteral("%1=%2")
                      .arg(QString::fromStdString(names[size_t(gi)]))
                      .arg(x[size_t(gi)], 0, 'g', 4);
          const bool diverged = (scored == 0) || (sum / std::max(1, scored)) >= autotune::kBig;
          const QString costStr =
              (scored == 0) ? QStringLiteral("n/a (harness fail)")
                            : QString::number(sum / scored, 'f', 2);
          emit log(QStringLiteral("  eval [%1]  cost=%2  pose: max|roll|=%3° "
                                  "max|pitch|=%4° peak|yawRate|=%5°/s%6")
                       .arg(gs.join(QStringLiteral(", ")))
                       .arg(costStr)
                       .arg(mxRoll, 0, 'f', 1)
                       .arg(mxPitch, 0, 'f', 1)
                       .arg(mxYawRate, 0, 'f', 1)
                       .arg(diverged ? QStringLiteral("  *** DIVERGED (>80°)")
                                     : QString()));
        }
        if (scored == 0)
          return std::nullopt;  // every attempt was a harness failure
        return sum / scored;
      });

  // Re-emit the engine's signals (this worker lives in the worker thread, so
  // they queue across to the GUI).
  connect(&engine, &AutotuneEngine::evaluated, this, &AutotuneWorker::evaluated);
  connect(&engine, &AutotuneEngine::finished, this, &AutotuneWorker::finished);

  {
    std::lock_guard<std::mutex> lk(m_engMtx);
    m_engine = &engine;
    if (m_cancel)
      engine.cancel();
  }
  engine.run();  // blocking: optimizer -> rollout -> stack
  {
    std::lock_guard<std::mutex> lk(m_engMtx);
    m_engine = nullptr;
  }

  stack.stop();
  emit log(QStringLiteral("SITL stack stopped."));
  emit done();
}

void AutotuneWorker::runRtos() {
  const autotune::Space space(m_p.tuneYaw, /*fastRtos=*/true);
  const std::vector<std::string> names = space.names();

  autotune::RtosEval rtos(m_p.rtosBin, m_p.sitl.suffix);
  if (!rtos.binExists()) {
    emit failed(QStringLiteral("fast backend not found: %1\n(build it with "
                               "-DVAYU_SITL_RTOS_BUILD=ON)")
                    .arg(m_p.rtosBin));
    emit done();
    return;
  }
  rtos.setCostMode(m_p.rtosRateCost ? autotune::RtosEval::CostMode::AngleRate
                                    : autotune::RtosEval::CostMode::Angle);
  emit log(m_p.rtosRateCost
               ? QStringLiteral("FAST autotune — in-process vayu_sitl_rtos "
                                "(deterministic, ~70x). Cost = angle tracking + "
                                "roll/pitch RATE-loop tracking (values the inner "
                                "loop), kBig on divergence.")
               : QStringLiteral("FAST autotune — in-process vayu_sitl_rtos "
                                "(deterministic, ~70x). Same cost as the realtime "
                                "tuner (angle IAE + overshoot + chatter, kBig on "
                                "divergence) from a seeded doublet."));
  // Fly the same doublet the realtime path uses (amplitude, soft-rig tether, and
  // — critically — a hold long enough to reach steady state, so the tracking
  // cost rewards responsiveness rather than a motionless craft).
  rtos.setExcitation(
      m_p.rollout.stepUs, m_p.rollout.tetherK, m_p.rollout.hold, m_p.rollout.ret,
      m_p.rollout.settle,
      m_p.rollout.excite == autotune::Excitation::Chirp ? 1 : 0,
      m_p.rollout.chirpF0, m_p.rollout.chirpF1);
  // Tune the loaded airframe: hand the same vsim_ctl_geometry_t SitlStack pushes
  // to vsim_d to the fast backend, which applies it to physics + firmware mix.
  if (m_p.sitl.hasGeometry) {
    const QByteArray blob(reinterpret_cast<const char *>(&m_p.sitl.geometry),
                          int(sizeof(m_p.sitl.geometry)));
    QString gerr;
    if (rtos.setGeometry(blob, &gerr))
      emit log(QStringLiteral("applied vehicle geometry (physics + firmware mix)."));
    else
      emit log(QStringLiteral("warning: %1 — tuning the reference quad.").arg(gerr));
  } else {
    emit log(QStringLiteral("note: no vehicle geometry — tuning the reference quad."));
  }
  emit log(QStringLiteral("tuning %1 params with '%2' (budget %3)")
               .arg(names.size())
               .arg(m_p.optimizer)
               .arg(m_p.budget));

  AutotuneEngine engine(
      m_p.tuneYaw, m_p.optimizer, m_p.budget, m_p.optSeed,
      [&](const QVector<double> &xq) -> std::optional<double> {
        // Map the Space param vector (by name) onto the RTOS gain knobs. Params
        // the backend has no env hook for (gyro_lpf, yaw_rate_ki/kd, yaw_gyro_lpf)
        // are simply not pushed — the binary keeps the firmware default for them.
        autotune::RtosEval::Gains g;
        for (int i = 0; i < int(names.size()) && i < xq.size(); ++i) {
          const std::string &n = names[size_t(i)];
          const double v = xq[i];
          if (n == "rate_kp") g.rate_kp = v;
          else if (n == "rate_ki") g.rate_ki = v;
          else if (n == "rate_kd") g.rate_kd = v;
          else if (n == "angle_kp") g.angle_kp = v;
          else if (n == "yaw_rate_kp") g.yaw_rate_kp = v;
        }

        // Average `repeats` deterministic rollouts over distinct sensor-noise
        // seeds (sim_seed + i), mirroring the SITL path: include divergences
        // (kBig) in the mean so a gain that tumbles on any realization is
        // penalised, and drop only harness failures (starved / no-score). Each
        // rollout is ~0.04 s, so repeats stay cheap.
        double sum = 0.0;
        int scored = 0;
        const int reps = std::max(1, m_p.repeats);
        autotune::RtosResult last;
        bool anyDiverged = false;
        for (int i = 0; i < reps && !m_cancel.load(); ++i) {
          const autotune::RtosResult r =
              rtos.rollout(g, m_p.rollout.seed + quint32(i), m_p.tuneYaw);
          if (!r.ok || r.starved) {
            last = r;
            continue;  // harness failure / starved window — skip this realization
          }
          last = r;
          // EXACT realtime cost (axisCost + yawRateCost, computed in RtosEval via
          // the SAME Cost.cpp): angle-IAE + overshoot + chatter, with kBig on
          // |angle| > 80° divergence — so a tumbling tune scores ~1e6, never low.
          sum += r.cost;
          anyDiverged = anyDiverged || r.diverged;
          ++scored;
        }

        // Push the latest roll-axis excitation window to the live response plot
        // (same trace + signal the realtime path uses).
        if (!last.respSp.isEmpty())
          emit responseWindow(last.respSp, last.respMeas);

        // Per-eval log: gains tried, mean cost, per-axis split + speedup.
        QStringList gs;
        for (int gi = 0; gi < int(names.size()) && gi < xq.size(); ++gi)
          gs << QStringLiteral("%1=%2")
                    .arg(QString::fromStdString(names[size_t(gi)]))
                    .arg(xq[gi], 0, 'g', 4);
        if (scored == 0) {
          emit log(QStringLiteral("  eval [%1]  cost=n/a (backend fail: %2)")
                       .arg(gs.join(QStringLiteral(", ")),
                            last.error.isEmpty() ? QStringLiteral("?")
                                                 : last.error.section('\n', 0, 0)));
          return std::nullopt;
        }
        const double mean = sum / scored;
        const bool diverged = anyDiverged || mean >= autotune::kBig;
        // In rate-cost mode, also show the inner-loop term split.
        const QString rateStr =
            m_p.rtosRateCost
                ? QStringLiteral(" rate[roll=%1 pitch=%2]")
                      .arg(last.rollRateCost, 0, 'f', 2)
                      .arg(last.pitchRateCost, 0, 'f', 2)
                : QString();
        emit log(QStringLiteral("  eval [%1]  cost=%2  angle: roll=%3 pitch=%4%5%6"
                                "  (%7x)%8")
                     .arg(gs.join(QStringLiteral(", ")))
                     .arg(diverged ? QStringLiteral("DIVERGED")
                                   : QString::number(mean, 'f', 3))
                     .arg(last.rollCost, 0, 'f', 2)
                     .arg(last.pitchCost, 0, 'f', 2)
                     .arg(m_p.tuneYaw
                              ? QStringLiteral(" yaw=%1").arg(last.yawCost, 0, 'f', 2)
                              : QString())
                     .arg(rateStr)
                     .arg(last.speedup, 0, 'f', 0)
                     .arg(diverged ? QStringLiteral("  *** DIVERGED (>80°)")
                                   : QString()));
        return mean;
      },
      /*fastRtos=*/true);

  connect(&engine, &AutotuneEngine::evaluated, this, &AutotuneWorker::evaluated);
  connect(&engine, &AutotuneEngine::finished, this, &AutotuneWorker::finished);

  {
    std::lock_guard<std::mutex> lk(m_engMtx);
    m_engine = &engine;
    if (m_cancel)
      engine.cancel();
  }
  engine.run();  // blocking: optimizer -> rtos rollout
  {
    std::lock_guard<std::mutex> lk(m_engMtx);
    m_engine = nullptr;
  }
  emit log(QStringLiteral("fast autotune finished."));
  emit done();
}

void AutotuneWorker::runSysId() {
  using autotune::DesignGains;
  using autotune::Plant;

  autotune::RtosEval rtos(m_p.rtosBin, m_p.sitl.suffix);
  if (!rtos.binExists()) {
    emit failed(QStringLiteral("fast backend not found: %1\n(build it with "
                               "-DVAYU_SITL_RTOS_BUILD=ON)")
                    .arg(m_p.rtosBin));
    emit done();
    return;
  }
  emit log(QStringLiteral(
      "SYSTEM-ID autotune — fit a plant model from one chirp, then compute the "
      "gains analytically (loop-shaping). No optimizer search, no tracking cost "
      "to game: the inner loop gets a real gain by construction."));

  // A rich, broadband chirp makes the rate-loop input persistently exciting so
  // the plant fit is well-conditioned. Honour the configured chirp band/timing
  // but force the chirp waveform regardless of the cost-mode setting.
  const double f0 = m_p.rollout.chirpF0 > 0 ? m_p.rollout.chirpF0 : 0.5;
  const double f1 = m_p.rollout.chirpF1 > f0 ? m_p.rollout.chirpF1 : 18.0;
  const double hold = std::max(1.5, m_p.rollout.hold);  // long enough to fit a lag
  rtos.setExcitation(m_p.rollout.stepUs, m_p.rollout.tetherK, hold,
                     m_p.rollout.ret, m_p.rollout.settle, /*waveform=*/1, f0, f1);

  // Tune the loaded airframe (physics + firmware mix), as the fast/realtime paths.
  if (m_p.sitl.hasGeometry) {
    const QByteArray blob(reinterpret_cast<const char *>(&m_p.sitl.geometry),
                          int(sizeof(m_p.sitl.geometry)));
    QString gerr;
    if (rtos.setGeometry(blob, &gerr))
      emit log(QStringLiteral("applied vehicle geometry (physics + firmware mix)."));
    else
      emit log(QStringLiteral("warning: %1 — identifying the reference quad.").arg(gerr));
  } else {
    emit log(QStringLiteral("note: no vehicle geometry — identifying the reference quad."));
  }

  // One chirp with the firmware-default gains (a stable seed) -> raw rate-loop
  // input/output per axis. Default Gains leave every knob at the firmware value.
  emit log(QStringLiteral("flying identification chirp (%1->%2 Hz) ...")
               .arg(f0, 0, 'g', 2)
               .arg(f1, 0, 'g', 2));
  autotune::RtosEval::RawRate raw;
  QString err;
  if (!rtos.captureRate(autotune::RtosEval::Gains{}, m_p.rollout.seed, raw, &err)) {
    emit failed(QStringLiteral("identification chirp failed: %1").arg(err));
    emit done();
    return;
  }
  if (m_cancel) { emit done(); return; }

  // Fit a plant per axis. Roll and pitch are physically identical on a symmetric
  // quad, so average their fits (R^2-weighted) for a robust shared estimate; the
  // back-to-back chirp contaminates whichever axis flies second, and averaging
  // damps that. Yaw is a separate plant (rate-commanded, no angle loop).
  const Plant pr = autotune::identifyPlant(raw.rollU, raw.rollOmega, raw.dt);
  const Plant pp = autotune::identifyPlant(raw.pitchU, raw.pitchOmega, raw.dt);
  const Plant prp = autotune::averagePlants(pr, pp);
  if (!prp.ok) {
    emit failed(QStringLiteral(
        "plant identification failed (roll R^2=%1, pitch R^2=%2) — the chirp did "
        "not excite the rate loop. Try a wider chirp band or a larger amplitude.")
                    .arg(pr.ok ? pr.r2 : 0.0, 0, 'f', 2)
                    .arg(pp.ok ? pp.r2 : 0.0, 0, 'f', 2));
    emit done();
    return;
  }
  emit log(QStringLiteral("roll/pitch plant: K=%1, tau=%2 ms, actuator BW=%3 Hz "
                          "(fit R^2 roll=%4 pitch=%5)")
               .arg(prp.K, 0, 'g', 4)
               .arg(prp.tau * 1000.0, 0, 'f', 1)
               .arg(prp.actuatorBwHz(), 0, 'f', 1)
               .arg(pr.ok ? pr.r2 : 0.0, 0, 'f', 2)
               .arg(pp.ok ? pp.r2 : 0.0, 0, 'f', 2));

  // Loop-shape the roll/pitch gains.
  const double wc = autotune::chooseCrossover(prp, m_p.sysIdBwFrac);
  const DesignGains rpg = autotune::designGains(prp, wc);
  emit log(QStringLiteral("design: crossover wc=%1 rad/s (%2 Hz, %3 of actuator BW) "
                          "-> rate_kp=%4 rate_ki=%5 rate_kd=%6 angle_kp=%7")
               .arg(wc, 0, 'f', 1)
               .arg(wc / (2.0 * M_PI), 0, 'f', 1)
               .arg(m_p.sysIdBwFrac, 0, 'g', 2)
               .arg(rpg.rate_kp, 0, 'g', 4)
               .arg(rpg.rate_ki, 0, 'g', 4)
               .arg(rpg.rate_kd, 0, 'g', 4)
               .arg(rpg.angle_kp, 0, 'g', 4));

  // Yaw (optional): same fit/design on the yaw rate loop; only yaw_rate_kp is
  // exposed by the fast Space, so the yaw kd/ki are computed but unused.
  double yawRateKp = -1.0;  // <0 -> not in the applied set
  if (m_p.tuneYaw) {
    const Plant py = autotune::identifyPlant(raw.yawU, raw.yawOmega, raw.dt);
    if (py.ok) {
      const double wcy = autotune::chooseCrossover(py, m_p.sysIdBwFrac);
      yawRateKp = autotune::designGains(py, wcy).rate_kp;
      emit log(QStringLiteral("yaw plant: K=%1, tau=%2 ms (R^2=%3) -> yaw_rate_kp=%4")
                   .arg(py.K, 0, 'g', 4)
                   .arg(py.tau * 1000.0, 0, 'f', 1)
                   .arg(py.r2, 0, 'f', 2)
                   .arg(yawRateKp, 0, 'g', 4));
    } else {
      emit log(QStringLiteral("warning: yaw plant fit failed — leaving yaw_rate_kp "
                              "at the firmware default."));
    }
  }

  // Map the designed gains onto the fast-Space param vector (by name) so the UI's
  // gains table + "apply" path consume them exactly like a search result.
  const autotune::Space space(m_p.tuneYaw, /*fastRtos=*/true);
  const std::vector<std::string> names = space.names();
  QVector<double> bestX(int(names.size()), 0.0);
  for (int i = 0; i < int(names.size()); ++i) {
    const std::string &n = names[size_t(i)];
    if (n == "rate_kp") bestX[i] = rpg.rate_kp;
    else if (n == "rate_ki") bestX[i] = rpg.rate_ki;
    else if (n == "rate_kd") bestX[i] = rpg.rate_kd;
    else if (n == "angle_kp") bestX[i] = rpg.angle_kp;
    else if (n == "yaw_rate_kp") bestX[i] = (yawRateKp >= 0 ? yawRateKp : 0.0);
  }

  // Verify: fly a step doublet with the designed gains and report the realtime
  // cost (so the user sees the analytic tune actually tracks, not just numbers).
  autotune::RtosEval::Gains vg;
  vg.rate_kp = rpg.rate_kp; vg.rate_ki = rpg.rate_ki; vg.rate_kd = rpg.rate_kd;
  vg.angle_kp = rpg.angle_kp;
  if (yawRateKp >= 0) vg.yaw_rate_kp = yawRateKp;
  rtos.setCostMode(autotune::RtosEval::CostMode::Angle);
  rtos.setExcitation(m_p.rollout.stepUs, m_p.rollout.tetherK,
                     std::max(1.0, m_p.rollout.hold), m_p.rollout.ret,
                     m_p.rollout.settle, /*waveform=*/0, f0, f1);
  double verifyCost = 0.0;
  if (!m_cancel) {
    const autotune::RtosResult vr =
        rtos.rollout(vg, m_p.rollout.seed, m_p.tuneYaw);
    if (vr.ok && !vr.starved) {
      verifyCost = vr.cost;
      if (!vr.respSp.isEmpty())
        emit responseWindow(vr.respSp, vr.respMeas);
      emit log(QStringLiteral(
                   "verification doublet: cost=%1  (roll=%2 pitch=%3%4)%5")
                   .arg(vr.diverged ? QStringLiteral("DIVERGED")
                                    : QString::number(vr.cost, 'f', 3))
                   .arg(vr.rollCost, 0, 'f', 2)
                   .arg(vr.pitchCost, 0, 'f', 2)
                   .arg(m_p.tuneYaw ? QStringLiteral(" yaw=%1").arg(vr.yawCost, 0, 'f', 2)
                                    : QString())
                   .arg(vr.diverged ? QStringLiteral("  *** DIVERGED (>80°)")
                                    : QString()));
    } else {
      emit log(QStringLiteral("verification doublet did not score (%1)")
                   .arg(vr.error.isEmpty() ? QStringLiteral("starved") : vr.error));
    }
  }

  QStringList qnames;
  for (const std::string &n : names)
    qnames << QString::fromStdString(n);
  emit finished(bestX, qnames, verifyCost);
  emit log(QStringLiteral("system-ID autotune finished."));
  emit done();
}
