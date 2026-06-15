#pragma once

#include "core/Types.h"  // AttitudeData, ...
#include <QByteArray>
#include <functional>

// The single home for the GCS's NavLink v2 *receive* handler table. This is the
// only translation unit that includes the generated codec for decoding, so the
// rest of the GCS never sees a navlink_* type — decoded leaves arrive as plain
// GCS structs through the std::function hooks below. Assign a hook from whatever
// module owns that data; any decoded leaf without a hook falls to onDefault
// (which just logs). Adding a message = one thunk + one hook + one ctor line in
// NavlinkRouter.cpp. See navlink/INTEGRATION.md.
class NavlinkRouter {
public:
  NavlinkRouter();
  ~NavlinkRouter();
  NavlinkRouter(const NavlinkRouter &) = delete;
  NavlinkRouter &operator=(const NavlinkRouter &) = delete;

  // Push one whole v2 frame (sync..CRC) through the parser; hooks fire inline.
  void feed(const QByteArray &v2Frame);

  // Per-leaf hooks — set by the owning GCS module (default: unset -> onDefault).
  std::function<void(const AttitudeData &)> onAttitude;
  // Fallback for every decoded leaf without a specific hook above.
  std::function<void(uint32_t msgid, int payloadLen)> onDefault;

private:
  struct Impl;
  Impl *d_;
};
