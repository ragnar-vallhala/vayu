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

#include <assert.h>
#include <dlfcn.h>
#include <stdio.h>
#include <string.h>

static size_t g_telem_bytes = 0;

static void on_uart2(void *user, const uint8_t *data, size_t n) {
  (void)user;
  (void)data;
  g_telem_bytes += n;
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

  /* 4) Every slot is filled. A designated initialiser that misses one leaves a
   *    NULL that would only crash on the call that happens to need it. */
  const void *const slots[] = {
      (const void *)api->boot,          (const void *)api->shutdown,
      (const void *)api->enable_serial_rc,
      (const void *)api->run_begin,     (const void *)api->run_step,
      (const void *)api->get_pose,      (const void *)api->reset_to,
      (const void *)api->set_testrig,   (const void *)api->set_geometry,
      (const void *)api->set_world,     (const void *)api->clear_obstacles,
      (const void *)api->add_obstacle,  (const void *)api->set_world_mesh,
      (const void *)api->clear_world_mesh, (const void *)api->set_rates,
      (const void *)api->set_noise,     (const void *)api->set_faults,
      (const void *)api->set_wind,      (const void *)api->set_pause,
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
  CHECK("firmware telemetry reached the iface", g_telem_bytes > 0);

  /* Deliberately NOT dlclose()d after boot: the engine has started vaios tasks
   * that are still running, and unloading the code under them would be a crash
   * with nothing to do with what this test measures. The process is exiting. */
  printf("%s: %d failure(s)\n", failures ? "FAIL" : "PASS", failures);
  return failures ? 1 : 0;
}
