#include "RtosEval.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QProcessEnvironment>
#include <QRegularExpression>

#include <cstring>
#include <vector>

#include "Cost.h" // autotune::Sample, axisCost, yawRateCost, kBig

namespace autotune {

namespace {
constexpr int kTuneSampleFloats = 13; // must match TUNE_SAMPLE_FLOATS (binary)
// Weight of the roll/pitch rate-loop term in AngleRate mode. yawRateCost is
// already normalised to the angle-cost magnitude (~5-20), so unit weight puts
// the inner-loop tracking on par with the outer-loop angle tracking.
constexpr double kRateWeight = 1.0;

// Parse the binary windows file (tune_win_write in host_rtos_main.c, magic
// 'TNT2'): [magic u32][n_axes u32], then per axis [count u32] and count * 13
// floats. Fills the angle windows (angleWin[0..2], autotune::Sample field order)
// and the roll/pitch RATE windows mapped onto a Sample's YAW slots so the exact
// yawRateCost() can score them. Returns false on a short/garbled/old-version file.
bool readWindows(const QByteArray &d, std::vector<Sample> angleWin[3],
                 std::vector<Sample> &rollRateWin,
                 std::vector<Sample> &pitchRateWin) {
  const char *p = d.constData();
  qsizetype n = d.size(), off = 0;
  auto rd = [&](void *dst, qsizetype bytes) -> bool {
    if (off + bytes > n)
      return false;
    std::memcpy(dst, p + off, size_t(bytes));
    off += bytes;
    return true;
  };
  quint32 magic = 0, naxes = 0;
  if (!rd(&magic, 4) || !rd(&naxes, 4) || magic != 0x32544e54u || naxes != 3)
    return false;
  rollRateWin.clear();
  pitchRateWin.clear();
  for (int ax = 0; ax < 3; ++ax) {
    quint32 cnt = 0;
    if (!rd(&cnt, 4))
      return false;
    angleWin[ax].clear();
    angleWin[ax].reserve(cnt);
    for (quint32 i = 0; i < cnt; ++i) {
      float f[kTuneSampleFloats];
      if (!rd(f, sizeof f))
        return false;
      Sample s;
      s.rollAngleSp = f[0];
      s.rollAngleCurr = f[1];
      s.rollOut = f[2];
      s.pitchAngleSp = f[3];
      s.pitchAngleCurr = f[4];
      s.pitchOut = f[5];
      s.yawRateSp = f[6];
      s.yawRateCurr = f[7];
      s.yawOut = f[8];
      angleWin[ax].push_back(s);
      // Map roll/pitch rate (f9..12) into a Sample's yaw-rate slots so the exact
      // yawRateCost() scores the inner loop (same fn, no per-axis duplicate).
      if (ax == 0) {
        Sample rr;
        rr.yawRateSp = f[9];
        rr.yawRateCurr = f[10];
        rr.yawOut = f[2];
        rollRateWin.push_back(rr);
      } else if (ax == 1) {
        Sample pr;
        pr.yawRateSp = f[11];
        pr.yawRateCurr = f[12];
        pr.yawOut = f[5];
        pitchRateWin.push_back(pr);
      }
    }
  }
  return true;
}
} // namespace

RtosEval::~RtosEval() {
  if (!m_geomPath.isEmpty())
    QFile::remove(m_geomPath);
}

bool RtosEval::binExists() const { return QFileInfo::exists(m_bin); }

bool RtosEval::setGeometry(const QByteArray &geomBytes, QString *err) {
  const QString path = QDir::temp().absoluteFilePath(
      QStringLiteral("vayu_rtos_geom%1.bin").arg(m_suffix));
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate) ||
      f.write(geomBytes) != geomBytes.size()) {
    if (err)
      *err = QStringLiteral("could not write geometry to %1").arg(path);
    return false;
  }
  f.close();
  m_geomPath = path;
  return true;
}

QProcessEnvironment RtosEval::buildEnv(const Gains &g, quint32 seed,
                                       const QString &winPath) const {
  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  env.insert(QStringLiteral("VAYU_RTOS_SCENARIO"), QStringLiteral("doublet"));
  env.insert(QStringLiteral("VAYU_RTOS_SEED"), QString::number(seed));
  env.insert(QStringLiteral("VAYU_RTOS_TUNE_OUT"), winPath);
  // Isolate /tmp advert paths from any other sim (world tab, a second tune).
  env.insert(QStringLiteral("VSIM_FIFO_SUFFIX"),
             QStringLiteral("_rtoseval%1").arg(m_suffix));
  // Tune the loaded airframe (physics + firmware mix) when geometry was set.
  if (!m_geomPath.isEmpty())
    env.insert(QStringLiteral("VAYU_RTOS_GEOMETRY"), m_geomPath);
  // Fly the same doublet the realtime path uses (amplitude/rig/timing).
  if (m_haveExcite) {
    env.insert(QStringLiteral("VAYU_RTOS_STEP_US"), QString::number(m_stepUs));
    env.insert(QStringLiteral("VAYU_RTOS_TETHER"),
               QString::number(m_tetherK, 'g', 6));
    env.insert(QStringLiteral("VAYU_RTOS_HOLD_MS"),
               QString::number(qRound(m_holdS * 1000.0)));
    env.insert(QStringLiteral("VAYU_RTOS_RET_MS"),
               QString::number(qRound(m_retS * 1000.0)));
    env.insert(QStringLiteral("VAYU_RTOS_SETTLE_MS"),
               QString::number(qRound(m_settleS * 1000.0)));
    env.insert(QStringLiteral("VAYU_RTOS_WAVEFORM"),
               QString::number(m_waveform));
    env.insert(QStringLiteral("VAYU_RTOS_CHIRP_F0"),
               QString::number(m_chirpF0, 'g', 6));
    env.insert(QStringLiteral("VAYU_RTOS_CHIRP_F1"),
               QString::number(m_chirpF1, 'g', 6));
  }
  // The binary reads a negative/absent env var as "use firmware default"
  // (apply_gains_from_env in host_rtos_main.c), so only push the ones we set.
  auto put = [&](const char *key, double v) {
    if (v >= 0.0)
      env.insert(QString::fromLatin1(key), QString::number(v, 'g', 9));
  };
  put("VAYU_RATE_KP", g.rate_kp);
  put("VAYU_RATE_KI", g.rate_ki);
  put("VAYU_RATE_KD", g.rate_kd);
  put("VAYU_ANGLE_KP", g.angle_kp);
  put("VAYU_YAW_RATE_KP", g.yaw_rate_kp);
  return env;
}

RtosResult RtosEval::rollout(const Gains &g, quint32 seed, bool tuneYaw,
                             int timeoutMs) const {
  RtosResult r;

  // Per-rollout temp file the backend writes the per-axis Sample windows to.
  const QString winPath = QDir::temp().absoluteFilePath(
      QStringLiteral("vayu_rtos_win%1_%2.bin").arg(m_suffix).arg(seed));
  QFile::remove(winPath); // stale-guard: never score a previous run's windows

  const QProcessEnvironment env = buildEnv(g, seed, winPath);

  QProcess proc;
  proc.setProcessEnvironment(env);
  proc.setProcessChannelMode(QProcess::SeparateChannels);
  proc.start(m_bin, {});
  if (!proc.waitForStarted(4000)) {
    r.error = QStringLiteral("failed to start %1").arg(m_bin);
    return r;
  }
  if (!proc.waitForFinished(timeoutMs)) {
    proc.kill();
    proc.waitForFinished(1000);
    r.error = QStringLiteral("rollout timed out (%1 ms)").arg(timeoutMs);
    return r;
  }

  const QString out = QString::fromUtf8(proc.readAllStandardOutput());
  // #RTOS-TUNE seed=.. roll_corr=.. pitch_corr=.. yaw_corr=.. rate_kp=.. ...
  static const QRegularExpression re(QStringLiteral(
      "#RTOS-TUNE seed=(\\d+) roll_corr=(\\S+) pitch_corr=(\\S+) "
      "yaw_corr=(\\S+) rate_kp=\\S+ angle_kp=\\S+ wall=\\S+ "
      "speedup=(\\S+)"));
  const QRegularExpressionMatch m = re.match(out);
  if (!m.hasMatch()) {
    const QString err = QString::fromUtf8(proc.readAllStandardError());
    r.error = QStringLiteral("no #RTOS-TUNE line from %1\n%2")
                  .arg(m_bin, err.right(600));
    return r;
  }
  r.roll = m.captured(2).toDouble();
  r.pitch = m.captured(3).toDouble();
  r.yaw = m.captured(4).toDouble();
  r.speedup = m.captured(5).toDouble();

  // Score over the per-axis Sample windows.
  QFile wf(winPath);
  std::vector<Sample> win[3], rollRate, pitchRate;
  if (!wf.open(QIODevice::ReadOnly) ||
      !readWindows(wf.readAll(), win, rollRate, pitchRate)) {
    wf.close();
    QFile::remove(winPath);
    r.error = QStringLiteral("missing/garbled tune windows (%1)").arg(winPath);
    return r;
  }
  wf.close();
  QFile::remove(winPath);

  // Roll-axis trace for the live response plot (angle sp vs measured, deg).
  r.respSp.reserve(int(win[0].size()));
  r.respMeas.reserve(int(win[0].size()));
  for (const Sample &s : win[0]) {
    r.respSp.push_back(s.rollAngleSp);
    r.respMeas.push_back(s.rollAngleCurr);
  }

  // Angle-loop cost (the exact realtime cost) — always part of the total.
  const std::optional<double> rc = axisCost(win[0], 0);
  const std::optional<double> pc = axisCost(win[1], 1);
  const std::optional<double> yc =
      tuneYaw ? yawRateCost(win[2]) : std::optional<double>(0.0);
  // Roll/pitch RATE-loop cost (AngleRate mode): the same yawRateCost applied to
  // the inner loop, so the search also rewards a tracking, non-buzzy rate loop.
  std::optional<double> rrc(0.0), prc(0.0);
  if (m_costMode == CostMode::AngleRate) {
    rrc = yawRateCost(rollRate);
    prc = yawRateCost(pitchRate);
  }
  if (!rc || !pc || !yc || !rrc || !prc) {
    // <5 samples in a window — telemetry-starved, exactly the realtime path's
    // "retry, not a divergence" case. The caller drops this realization.
    r.ok = true;
    r.starved = true;
    return r;
  }
  r.rollCost = *rc;
  r.pitchCost = *pc;
  r.yawCost = tuneYaw ? *yc : 0.0;
  r.rollRateCost = *rrc;
  r.pitchRateCost = *prc;
  r.cost = *rc + *pc + (tuneYaw ? *yc : 0.0);
  if (m_costMode == CostMode::AngleRate)
    r.cost += kRateWeight * (*rrc + *prc);
  r.diverged = (*rc >= kBig) || (*pc >= kBig) || (tuneYaw && *yc >= kBig) ||
               (*rrc >= kBig) || (*prc >= kBig);
  r.ok = true;
  return r;
}

bool RtosEval::captureRate(const Gains &g, quint32 seed, RawRate &out,
                           QString *err, int timeoutMs) const {
  out = RawRate{};
  const QString winPath = QDir::temp().absoluteFilePath(
      QStringLiteral("vayu_rtos_sid%1_%2.bin").arg(m_suffix).arg(seed));
  QFile::remove(winPath);

  QProcess proc;
  proc.setProcessEnvironment(buildEnv(g, seed, winPath));
  proc.setProcessChannelMode(QProcess::SeparateChannels);
  proc.start(m_bin, {});
  if (!proc.waitForStarted(4000)) {
    if (err)
      *err = QStringLiteral("failed to start %1").arg(m_bin);
    return false;
  }
  if (!proc.waitForFinished(timeoutMs)) {
    proc.kill();
    proc.waitForFinished(1000);
    if (err)
      *err = QStringLiteral("capture timed out (%1 ms)").arg(timeoutMs);
    return false;
  }

  QFile wf(winPath);
  std::vector<Sample> win[3], rollRate, pitchRate;
  if (!wf.open(QIODevice::ReadOnly) ||
      !readWindows(wf.readAll(), win, rollRate, pitchRate)) {
    wf.close();
    QFile::remove(winPath);
    if (err) {
      const QString e = QString::fromUtf8(proc.readAllStandardError());
      *err = QStringLiteral("missing/garbled tune windows (%1)\n%2")
                 .arg(winPath, e.right(400));
    }
    return false;
  }
  wf.close();
  QFile::remove(winPath);

  // Extract the rate-loop input/output series the System-ID fit needs. In a
  // Sample mapped from a rate window (readWindows), yawOut is the rate-PID output
  // u (roll_out/pitch_out) and yawRateCurr is the measured rate omega; yawRateSp
  // is the rate setpoint (kept for diagnostics). dt = 1 ms/step (the binary's
  // SITL_IMU_FEED_HZ). The roll/pitch chirp windows each cover that axis's hold.
  auto fill = [](const std::vector<Sample> &w, QVector<double> &u,
                 QVector<double> &omega, QVector<double> &sp) {
    u.reserve(int(w.size()));
    omega.reserve(int(w.size()));
    sp.reserve(int(w.size()));
    for (const Sample &s : w) {
      u.push_back(s.yawOut);
      omega.push_back(s.yawRateCurr);
      sp.push_back(s.yawRateSp);
    }
  };
  fill(rollRate, out.rollU, out.rollOmega, out.rollSp);
  fill(pitchRate, out.pitchU, out.pitchOmega, out.pitchSp);
  // Yaw is rate-commanded, so its "angle" window win[2] already holds the rate
  // loop directly (yawRateSp/yawRateCurr/yawOut from f6/f7/f8).
  fill(win[2], out.yawU, out.yawOmega, out.yawSp);
  out.dt = 0.001;
  return !out.rollU.isEmpty();
}

} // namespace autotune
