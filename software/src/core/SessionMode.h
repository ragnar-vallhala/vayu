#pragma once

#include <QMetaType>
#include <QObject>

// Whole-GCS session mode. In Replay the UI is fed from a recorded log and
// every outbound path is refused — one enum is the read-only authority, so the
// guarantee is auditable by grepping the single guard rather than reasoning
// about per-widget disabling (docs/roadmap/gcs-log-replay.md).
enum class SessionMode { Live, Replay };

// Holds the session mode and answers the one question every command-emitting
// site asks before transmitting: is tx allowed right now? Emits changed() so
// the toolbar / LIVE pill can react. The Simulator is intentionally NOT gated
// by this — it is a separate live process, not a consumer of the link.
class SessionState : public QObject {
  Q_OBJECT

public:
  using QObject::QObject;

  SessionMode mode() const { return m_mode; }
  bool isReplay() const { return m_mode == SessionMode::Replay; }
  // Transmission is allowed only in Live mode.
  bool txAllowed() const { return m_mode == SessionMode::Live; }

  void setMode(SessionMode m) {
    if (m == m_mode)
      return;
    m_mode = m;
    emit changed(m_mode);
  }

signals:
  void changed(SessionMode mode);

private:
  SessionMode m_mode = SessionMode::Live;
};

Q_DECLARE_METATYPE(SessionMode)
