#include "NavlinkRouter.h"

#include <QtGlobal>

extern "C" {
#include "navlink_msgs.h"  // the generated codec — included ONLY here (RX side)
}

namespace {
constexpr float kRad2Deg = 57.29577951308232f;

// One thunk per wired leaf: cast ctx back to the router and call its hook,
// converting the v2 message into the GCS's own struct first.
void thunkAttitude(void *ctx, const navlink_frame_hdr_t *,
                   const navlink_attitude_euler_t *m) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (r->onAttitude) {
    AttitudeData a;  // v2 is radians; AttitudeData is degrees (core/Types.h)
    a.roll = m->roll * kRad2Deg;
    a.pitch = m->pitch * kRad2Deg;
    a.yaw = m->yaw * kRad2Deg;
    r->onAttitude(a);
  }
}

void thunkDefault(void *ctx, const navlink_frame_hdr_t *, uint32_t msgid,
                  const uint8_t *, size_t len) {
  auto *r = static_cast<NavlinkRouter *>(ctx);
  if (r->onDefault)
    r->onDefault(msgid, static_cast<int>(len));
}
}  // namespace

struct NavlinkRouter::Impl {
  navlink_parser_t parser;
  navlink_handlers_t handlers;
};

NavlinkRouter::NavlinkRouter() : d_(new Impl) {
  navlink_parser_init(&d_->parser);
  d_->handlers = {};               // zero every slot
  d_->handlers.ctx = this;         // shared ctx: the router itself
  d_->handlers.on_default = thunkDefault;
  d_->handlers.on_attitude_euler = thunkAttitude;
  // Sensible default so an unhandled leaf is never silent, even if the owner
  // didn't override onDefault.
  onDefault = [](uint32_t msgid, int len) {
    qInfo("[navlink] unhandled v2 msgid %u (%d B)", msgid, len);
  };
}

NavlinkRouter::~NavlinkRouter() { delete d_; }

void NavlinkRouter::feed(const QByteArray &f) {
  navlink_parser_push(&d_->parser, &d_->handlers,
                      reinterpret_cast<const uint8_t *>(f.constData()),
                      static_cast<size_t>(f.size()));
}
