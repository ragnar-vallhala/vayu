/*
 * test_sitl_module.c -- proves the SITL module is loadable and its vtable wired.
 *
 * test_gcs_engine_api.cpp already covers what the engine DOES; this covers the
 * part that is new and easy to get silently wrong: that libvayu_sitl.so loads
 * with no link-time knowledge of the firmware, exports exactly one symbol, and
 * hands back a table whose entries actually point at the engine.
 *
 * It deliberately links NOTHING from the firmware -- only libdl and the two SDK
 * headers -- so if the module ever stops being self-contained (an unresolved
 * symbol it expected the host to provide), this fails to load and says so.
 */
#include "vayu_sitl_abi.h"

/* The wire codec -- the protocol, not the firmware. A real ground station
 * links exactly this, which is the point: the test talks to the module the way
 * a GCS does. */
#include "navlink_msgs.h"

#include <assert.h>
#include <dlfcn.h>
#include <stdio.h>
#include <string.h>

static size_t g_telem_bytes = 0;

/* Parser for the firmware's replies, so the test can see what it made of an
 * injected command rather than guessing from byte counts. */
static navlink_parser_t g_parser;
static navlink_handlers_t g_handlers;
static int g_acks;
static uint8_t g_last_ack_result;
static uint32_t g_last_ack_command;

static void on_ack(void *ctx, const navlink_frame_hdr_t *hdr,
                   const navlink_command_ack_t *msg) {
  (void)ctx;
  (void)hdr;
  ++g_acks;
  g_last_ack_result = msg->result;
  g_last_ack_command = msg->command;
}

static void on_uart2(void *user, const uint8_t *data, size_t n) {
  (void)user;
  g_telem_bytes += n;
  navlink_parser_push(&g_parser, &g_handlers, data, n);
}

/* One CMD_SET_PID frame, built exactly as a GCS builds it. */
static size_t make_set_pid(uint8_t *out) {
  navlink_cmd_set_pid_t m;
  memset(&m, 0, sizeof m);
  m.target_sys = 42;
  m.target_comp = 1;
  m.req_seq = 7;
  m.controller = 1; /* rate */
  m.axis = 0;       /* roll */
  m.kp = 0.05f;
  m.ki = 0.0f;
  m.kd = 0.001f;
  m.kff = 0.0f;
  return navlink_cmd_set_pid_encode(out, &m, m.req_seq, 0xFF, 1);
}

static void step(const vayu_sitl_api_t *api, int n) {
  for (int i = 0; i < n; i++)
    api->run_step();
}

int main(int argc, char **argv) {
  /* The module path is passed in by ctest so this test has no opinion about
   * the build layout. */
  const char *path = (argc > 1) ? argv[1] : "./libvayu_sitl.so";
  int failures = 0;

#define CHECK(what, ok)                                                        \
  do {                                                                         \
    printf("  [%s] %s\n", (ok) ? "PASS" : "FAIL", (what));                     \
    if (!(ok))                                                                 \
      ++failures;                                                              \
  } while (0)

  /* Reply parser, wired before anything can arrive. */
  navlink_parser_init(&g_parser);
  memset(&g_handlers, 0, sizeof g_handlers);
  g_handlers.on_command_ack = on_ack;

  /* 1) It loads at all. RTLD_NOW so a missing symbol fails here, loudly,
   *    rather than at the first call through the vtable. */
  void *h = dlopen(path, RTLD_NOW | RTLD_LOCAL);
  if (!h) {
    printf("  [FAIL] dlopen(%s): %s\n", path, dlerror());
    return 1;
  }
  printf("  [PASS] dlopen(%s)\n", path);

  /* 2) The one exported symbol resolves. */
  vayu_sitl_get_api_fn get_api =
      (vayu_sitl_get_api_fn)dlsym(h, VAYU_SITL_ENTRY_SYMBOL);
  CHECK("dlsym(" VAYU_SITL_ENTRY_SYMBOL ")", get_api != NULL);
  if (!get_api) {
    dlclose(h);
    return 1;
  }

  /* 3) A wrong ABI version is refused rather than served a mismatched table. */
  CHECK("get_api(bogus version) == NULL",
        get_api(VAYU_SITL_ABI_VERSION + 1000u) == NULL);

  const vayu_sitl_api_t *api = get_api(VAYU_SITL_ABI_VERSION);
  CHECK("get_api(current version) != NULL", api != NULL);
  if (!api) {
    dlclose(h);
    return 1;
  }
  CHECK("api->abi_version matches", api->abi_version == VAYU_SITL_ABI_VERSION);

  /* The build stamp must be present and actually stamped. "unknown" is a legal
   * value for a module built outside a git tree, but not for one built here --
   * a stamp that quietly stops updating is worse than none, since it reads as
   * authoritative. */
  CHECK("build_id is set", api->build_id != NULL && api->build_id[0] != '\0');
  if (api->build_id) {
    printf("         build_id: %s\n", api->build_id);
    CHECK("build_id was stamped by the build",
          strcmp(api->build_id, "unknown") != 0);
  }

  /* 4) Every slot is filled. A designated initialiser that misses one leaves a
   *    NULL that would only crash on the call that happens to need it. */
  const void *const slots[] = {
      (const void *)api->boot,
      (const void *)api->set_telemetry_sink,
      (const void *)api->shutdown,
      (const void *)api->enable_serial_rc,
      (const void *)api->run_begin,
      (const void *)api->run_step,
      (const void *)api->get_pose,
      (const void *)api->reset_to,
      (const void *)api->set_testrig,
      (const void *)api->set_geometry,
      (const void *)api->set_world,
      (const void *)api->clear_obstacles,
      (const void *)api->add_obstacle,
      (const void *)api->set_world_mesh,
      (const void *)api->clear_world_mesh,
      (const void *)api->set_rates,
      (const void *)api->set_noise,
      (const void *)api->set_faults,
      (const void *)api->set_wind,
      (const void *)api->set_pause,
      (const void *)api->uart2_rx,
  };
  int null_slots = 0;
  for (size_t i = 0; i < sizeof slots / sizeof slots[0]; i++)
    if (slots[i] == NULL)
      ++null_slots;
  CHECK("no NULL vtable slots", null_slots == 0);

  /* 5) The table points at a live engine, not just valid addresses: boot it
   *    through the vtable and confirm the sim advances and telemetry flows.
   *    Note there is no firmware struct here -- just a callback, which is the
   *    whole point of the boot signature. */
  CHECK("api->boot(cb, NULL) == 0", api->boot(&on_uart2, NULL) == 0);
  api->run_begin();

  vsim_pose_frame_t p0, p1;
  memset(&p0, 0, sizeof p0);
  memset(&p1, 0, sizeof p1);
  api->get_pose(&p0);
  for (int i = 0; i < 2000; i++) /* 2 s of 1 ms steps */
    api->run_step();
  api->get_pose(&p1);

  const uint64_t t0 = ((uint64_t)p0.tick_hi << 32) | p0.tick_lo;
  const uint64_t t1 = ((uint64_t)p1.tick_hi << 32) | p1.tick_lo;
  CHECK("pose tick advanced through the vtable", t1 > t0);
  CHECK("firmware telemetry reached the sink", g_telem_bytes > 0);

  /* 6) Detaching goes quiet without tearing anything down -- the path a host
   *    takes when it stops its worker but leaves the process alive. */
  api->set_telemetry_sink(NULL, NULL);
  const size_t at_detach = g_telem_bytes;
  for (int i = 0; i < 500; i++)
    api->run_step();
  CHECK("no telemetry after detach", g_telem_bytes == at_detach);

  /* And re-attaching resumes it, which is what Stop -> Start must do. */
  api->set_telemetry_sink(&on_uart2, NULL);
  for (int i = 0; i < 500; i++)
    api->run_step();
  CHECK("telemetry resumes after re-attach", g_telem_bytes > at_detach);

  /* 7) GCS -> FC: an injected command reaches the firmware's real command path.
   *    The observable is the firmware's own COMMAND_ACK, which also proves the
   *    §10.5 time-sync gate is being enforced rather than bypassed -- the whole
   *    reason a host must not poke firmware functions directly. An unsynced FC
   *    must answer TEMPORARILY_REJECTED, not silently do the thing. */
  uint8_t frame[NAVLINK_MAX_FRAME];
  const size_t flen = make_set_pid(frame);
  CHECK("encoded a CMD_SET_PID frame", flen > 0);

  g_acks = 0;
  api->uart2_rx(frame, flen);
  step(api, 200); /* let the comm task drain and reply */

  CHECK("injected command produced an ACK", g_acks > 0);
  CHECK("unsynced FC rejects the command (time-sync gate enforced)",
        g_last_ack_result == NAVLINK_COMMAND_RESULT_TEMPORARILY_REJECTED);
  CHECK("ACK names the command we sent",
        g_last_ack_command == NAVLINK_MSGID_CMD_SET_PID);

  /* Deliberately NOT dlclose()d after boot: the engine has started vaios tasks
   * that are still running, and unloading the code under them would be a crash
   * with nothing to do with what this test measures. The process is exiting. */
  printf("%s: %d failure(s)\n", failures ? "FAIL" : "PASS", failures);
  return failures ? 1 : 0;
}
