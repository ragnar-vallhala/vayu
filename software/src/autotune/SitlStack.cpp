#include "SitlStack.h"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>

#include <QElapsedTimer>
#include <QFile>
#include <QProcess>
#include <QSet>

#include "CommandCodec.h"
#include "DroneProtocol.h"
#include "Types.h"
#include "vsim_proto.h"

namespace {
constexpr size_t kMaxSamples = 20000;

void makeRaw(int fd) {
  termios a{};
  if (tcgetattr(fd, &a) != 0)
    return;
  a.c_lflag &= ~(ICANON | ECHO | ISIG);
  a.c_iflag &= ~(ICRNL | INLCR);
  a.c_oflag &= ~OPOST;
  tcsetattr(fd, TCSANOW, &a);
}

bool waitPath(const QString &path, int timeoutMs) {
  QElapsedTimer t;
  t.start();
  while (!QFile::exists(path)) {
    if (t.elapsed() > timeoutMs)
      return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return true;
}
}  // namespace

SitlStack::SitlStack(Config cfg, QObject *parent)
    : QObject(parent), m_cfg(std::move(cfg)) {}

SitlStack::~SitlStack() { stop(); }

bool SitlStack::openRcPty(QString *err, QString *rcSlavePathOut) {
  // The firmware calls tcgetattr() on its RC path, so it must be a real tty:
  // open a PTY master, hand the firmware the slave path, write RC to the master.
  m_rcMaster = posix_openpt(O_RDWR | O_NOCTTY);
  if (m_rcMaster < 0 || grantpt(m_rcMaster) != 0 || unlockpt(m_rcMaster) != 0) {
    if (err) *err = "RC PTY allocation failed";
    return false;
  }
  const char *slave = ptsname(m_rcMaster);
  if (!slave) {
    if (err) *err = "ptsname failed";
    return false;
  }
  *rcSlavePathOut = QString::fromLocal8Bit(slave);
  makeRaw(m_rcMaster);
  return true;
}

bool SitlStack::openCtlFifo(QString *err) {
  const QString ctl = QStringLiteral("/tmp/vsim_ctl%1").arg(m_cfg.suffix);
  if (!waitPath(ctl, 5000)) {
    if (err) *err = "timed out waiting for " + ctl;
    return false;
  }
  m_ctlFd = ::open(ctl.toLocal8Bit().constData(), O_RDWR | O_NONBLOCK);
  if (m_ctlFd < 0) {
    if (err) *err = "open ctl FIFO failed: " + ctl;
    return false;
  }
  return true;
}

bool SitlStack::openUart2Pty(QString *err) {
  // The firmware advertises its UART2 PTY slave path in a small text file.
  const QString advert =
      QStringLiteral("/tmp/vayu_uart2_pty%1").arg(m_cfg.suffix);
  if (!waitPath(advert, 6000)) {
    if (err) *err = "timed out waiting for " + advert;
    return false;
  }
  QFile f(advert);
  if (!f.open(QIODevice::ReadOnly)) {
    if (err) *err = "open advert failed: " + advert;
    return false;
  }
  const QString slave = QString::fromUtf8(f.readAll()).trimmed();
  f.close();
  if (!waitPath(slave, 5000)) {
    if (err) *err = "uart2 slave never appeared: " + slave;
    return false;
  }
  m_ptyFd = ::open(slave.toLocal8Bit().constData(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (m_ptyFd < 0) {
    if (err) *err = "open uart2 pty failed: " + slave;
    return false;
  }
  makeRaw(m_ptyFd);
  return true;
}

bool SitlStack::start(QString *err) {
  QString rcSlave;
  if (!openRcPty(err, &rcSlave))
    return false;

  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  env.insert("VSIM_FIFO_SUFFIX", m_cfg.suffix);
  env.insert("VAYU_UART_RC_PATH", rcSlave);

  auto spawn = [&](const QString &bin) -> QProcess * {
    auto *p = new QProcess(this);
    p->setProcessEnvironment(env);
    if (m_cfg.quiet) {
      p->setStandardOutputFile(QProcess::nullDevice());
      p->setStandardErrorFile(QProcess::nullDevice());
    }
    p->start(bin, {});
    if (!p->waitForStarted(4000)) {
      delete p;
      return nullptr;
    }
    return p;
  };

  // vsim_d first: it creates the ctl/pwm/imu/pose FIFOs the firmware opens.
  m_vsim = spawn(m_cfg.vsimBin);
  if (!m_vsim) {
    if (err) *err = "failed to start vsim_d: " + m_cfg.vsimBin;
    return false;
  }
  if (!openCtlFifo(err))
    return false;
  sendCtlRates();

  // firmware host: opens the RC PTY, creates its UART2 PTY, boots to STANDBY.
  m_sitl = spawn(m_cfg.sitlBin);
  if (!m_sitl) {
    if (err) *err = "failed to start vayu_sitl: " + m_cfg.sitlBin;
    return false;
  }

  m_stop = false;
  m_rcWriter = std::thread([this] { rcWriterLoop(); });

  if (!openUart2Pty(err))
    return false;
  m_reader = std::thread([this] { readerLoop(); });

  // Gate on telemetry actually flowing + a reported state.
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < 8000) {
    if (!snapshot().empty() && !lastState().isEmpty()) {
      m_running = true;
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (err) *err = "firmware never produced telemetry (cold-start timeout)";
  return false;
}

void SitlStack::stop() {
  m_stop = true;
  if (m_reader.joinable())
    m_reader.join();
  if (m_rcWriter.joinable())
    m_rcWriter.join();
  for (int *fd : {&m_ctlFd, &m_ptyFd, &m_rcMaster}) {
    if (*fd >= 0) {
      ::close(*fd);
      *fd = -1;
    }
  }
  for (QProcess *p : {m_sitl, m_vsim}) {
    if (p) {
      p->terminate();
      if (!p->waitForFinished(2000))
        p->kill();
    }
  }
  m_sitl = m_vsim = nullptr;
  m_running = false;
}

// --- vsim_d control frames (mirror SimWorker) ------------------------------

void SitlStack::sendCtlRates() {
  if (m_ctlFd < 0) return;
  vsim_ctl_frame_t f{};
  f.hdr.magic = VSIM_MAGIC;
  f.hdr.version = VSIM_PROTO_VERSION;
  f.hdr.type = VSIM_FRAME_CTL;
  f.hdr.payload_bytes = sizeof(f) - sizeof(vsim_hdr_t);
  f.subtype = VSIM_CTL_SET_RATES;
  vsim_ctl_rates_t b{};
  b.imu_hz = quint32(m_cfg.imuHz);
  b.physics_hz = quint32(m_cfg.physicsHz);
  b.pose_hz = quint32(m_cfg.poseHz);
  std::memcpy(f.body, &b, sizeof(b));
  ::write(m_ctlFd, &f, sizeof(f));
}

void SitlStack::sendCtlReset(quint32 seed) {
  if (m_ctlFd < 0) return;
  vsim_ctl_frame_t f{};
  f.hdr.magic = VSIM_MAGIC;
  f.hdr.version = VSIM_PROTO_VERSION;
  f.hdr.type = VSIM_FRAME_CTL;
  f.hdr.payload_bytes = sizeof(f) - sizeof(vsim_hdr_t);
  f.subtype = VSIM_CTL_RESET;
  vsim_ctl_reset_t b{};
  b.pos_w[2] = -0.05f;
  b.quat_wxyz[0] = 1.0f;
  b.seed = seed;
  std::memcpy(f.body, &b, sizeof(b));
  ::write(m_ctlFd, &f, sizeof(f));
}

void SitlStack::sendCtlTestRig(bool on, float tetherK) {
  if (m_ctlFd < 0) return;
  vsim_ctl_frame_t f{};
  f.hdr.magic = VSIM_MAGIC;
  f.hdr.version = VSIM_PROTO_VERSION;
  f.hdr.type = VSIM_FRAME_CTL;
  f.hdr.payload_bytes = sizeof(f) - sizeof(vsim_hdr_t);
  f.subtype = VSIM_CTL_SET_TESTRIG;
  vsim_ctl_testrig_t b{};
  b.enable = on ? 1 : 0;
  b.pos[2] = -0.05f;
  b.tether_k = tetherK;
  std::memcpy(f.body, &b, sizeof(b));
  ::write(m_ctlFd, &f, sizeof(f));
}

void SitlStack::reset(quint32 seed) { sendCtlReset(seed); }
void SitlStack::setTestRig(bool on, float tetherK) {
  sendCtlTestRig(on, tetherK);
}

// --- navlink commands ------------------------------------------------------

void SitlStack::writeNavlink(const QByteArray &frame) {
  std::lock_guard<std::mutex> lk(m_writeMtx);
  if (m_ptyFd < 0) return;
  const char *p = frame.constData();
  qsizetype left = frame.size();
  QElapsedTimer t;
  t.start();
  while (left > 0) {
    ssize_t n = ::write(m_ptyFd, p, size_t(left));
    if (n > 0) {
      p += n;
      left -= n;
    } else if (t.elapsed() > 500) {
      break;  // give up on a stuck write rather than spin forever
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void SitlStack::setPid(int controller, int axis, float kp, float ki, float kd,
                       float kff) {
  writeNavlink(CommandCodec::encodeSetPid(controller, axis, kp, ki, kd, kff));
}
void SitlStack::setGyroLpf(int axis, float rc) {
  writeNavlink(CommandCodec::encodeSetGyroLpf(axis, rc));
}
void SitlStack::setFlightMode(int mode) {
  writeNavlink(CommandCodec::encodeSetFlightMode(mode));
}

// --- RC --------------------------------------------------------------------

void SitlStack::setRc(int roll, int pitch, int thr, int yaw, int arm, int ch6) {
  std::lock_guard<std::mutex> lk(m_rcMtx);
  if (roll >= 0) m_rc[0] = roll;
  if (pitch >= 0) m_rc[1] = pitch;
  if (thr >= 0) m_rc[2] = thr;
  if (yaw >= 0) m_rc[3] = yaw;
  if (arm >= 0) m_rc[4] = arm;
  if (ch6 >= 0) m_rc[5] = ch6;
}

void SitlStack::rcWriterLoop() {
  while (!m_stop) {
    QByteArray line;
    {
      std::lock_guard<std::mutex> lk(m_rcMtx);
      line = QByteArray::number(m_rc[0]);
      for (int i = 1; i < 6; ++i)
        line += "," + QByteArray::number(m_rc[i]);
      line += "\n";
    }
    if (m_rcMaster >= 0)
      (void)::write(m_rcMaster, line.constData(), size_t(line.size()));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));  // ~50 Hz
  }
}

// --- telemetry reader ------------------------------------------------------

void SitlStack::onControlLoop(const autotune::Sample &s) {
  std::lock_guard<std::mutex> lk(m_sampMtx);
  m_samples.push_back(s);
  if (m_samples.size() > kMaxSamples)
    m_samples.pop_front();
}

void SitlStack::onStatus(const QString &state) {
  // statusReceived carries the system-state NAME for sys-state packets, but
  // other 0x6 status origins fall through DroneProtocol's raw-string path —
  // ignore those so they don't clobber the tracked state with garbage.
  static const QSet<QString> kStates = {
      "UNINITIALIZED", "INIT",     "STANDBY",    "PREARM",     "ARMED",
      "IN_AIR",        "FAILSAFE", "TERMINATED", "CALIBRATING"};
  if (!kStates.contains(state))
    return;
  std::lock_guard<std::mutex> lk(m_stateMtx);
  m_lastState = state;
}

void SitlStack::readerLoop() {
  // DroneProtocol decodes navlink telemetry; its signals fire synchronously on
  // this thread (functor connections), so no event loop is needed here.
  DroneProtocol proto;
  QObject::connect(&proto, &DroneProtocol::controlLoopDataReceived,
                   [this](const ControlLoopData &d) {
                     autotune::Sample s;
                     s.rollAngleSp = d.roll_angle_setpoint;
                     s.rollAngleCurr = d.roll_angle_current;
                     s.rollOut = d.roll_output;
                     s.pitchAngleSp = d.pitch_angle_setpoint;
                     s.pitchAngleCurr = d.pitch_angle_current;
                     s.pitchOut = d.pitch_output;
                     s.yawRateSp = d.yaw_rate_setpoint;
                     s.yawRateCurr = d.yaw_rate_current;
                     s.yawOut = d.yaw_output;
                     onControlLoop(s);
                   });
  QObject::connect(&proto, &DroneProtocol::statusReceived,
                   [this](const QString &st) { onStatus(st); });

  char buf[4096];
  while (!m_stop) {
    pollfd pfd{m_ptyFd, POLLIN, 0};
    if (::poll(&pfd, 1, 50) <= 0)
      continue;
    const ssize_t n = ::read(m_ptyFd, buf, sizeof(buf));
    if (n > 0)
      proto.processData(QByteArray(buf, int(n)));
    else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

// --- arm / state -----------------------------------------------------------

QString SitlStack::lastState() const {
  std::lock_guard<std::mutex> lk(m_stateMtx);
  return m_lastState;
}

bool SitlStack::waitState(const QString &name, int timeoutMs) {
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < timeoutMs) {
    if (lastState().compare(name, Qt::CaseInsensitive) == 0)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

bool SitlStack::arm(int timeoutMs) {
  // Arm switch low first to clear any latched FAILSAFE, settle into STANDBY.
  setRc(1500, 1500, 1000, 1500, /*arm=*/1000);
  waitState("STANDBY", 1500);
  setRc(1500, 1500, 1000, 1500, /*arm=*/2000);  // throttle low + arm high
  return waitState("ARMED", timeoutMs);
}

void SitlStack::disarm() { setRc(-1, -1, 1000, -1, /*arm=*/1000); }

void SitlStack::clearSamples() {
  std::lock_guard<std::mutex> lk(m_sampMtx);
  m_samples.clear();
}

std::vector<autotune::Sample> SitlStack::snapshot() const {
  std::lock_guard<std::mutex> lk(m_sampMtx);
  return {m_samples.begin(), m_samples.end()};
}
