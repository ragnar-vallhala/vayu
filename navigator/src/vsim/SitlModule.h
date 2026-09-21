#pragma once

#include <QString>
#include <QStringList>

#include "vayu_sitl_abi.h"

// Runtime loader for libvayu_sitl -- the firmware compiled for the host.
//
// Navigator does not link the firmware. It loads the module at runtime and
// calls through the vtable in vayu_sitl_abi.h, so the GCS and the firmware
// build and ship independently, and a firmware crash is a failed load or a
// dead simulator rather than a dead GCS.
//
// Loading is lazy and never fatal: if the module is missing or its ABI does
// not match, available() stays false and the app runs with the simulator
// disabled -- the same degraded state as a -DNAVIGATOR_SITL=OFF build. Callers
// must check available() before api().
//
// One instance per process, because the engine holds global state and boots
// once (see vayu_sitl_abi.h). Hence a singleton rather than an ownable object.
class SitlModule {
public:
  // Loads on first call; subsequent calls return the same result without
  // retrying, so a missing module costs one failed load, not one per caller.
  static SitlModule &instance();

  bool available() const { return api_ != nullptr; }

  // Valid only when available(). Null otherwise, so a caller that skips the
  // check crashes here rather than somewhere further downstream.
  const vayu_sitl_api_t *api() const { return api_; }

  // Human-readable reason the load failed, for the UI and the log. Empty when
  // the module loaded.
  QString error() const { return error_; }

  // Absolute path of the module actually loaded; empty if none was.
  QString path() const { return path_; }

  // The paths that were tried, in order, for a "why can't it find it" message.
  static QStringList searchPaths();

private:
  SitlModule();

  const vayu_sitl_api_t *api_ = nullptr;
  QString error_;
  QString path_;
};
