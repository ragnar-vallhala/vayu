/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * vayu_sitl_module.c -- the loadable module's single entry point.
 *
 * Nothing here does work: it binds the engine's existing free functions into
 * the vtable vayu_sitl_abi.h publishes, so the in-process engine, the
 * vayu_sitl_rtos binary and a runtime loader all drive exactly the same code.
 */
#include <stddef.h>

#include "vayu_sitl_abi.h"

#include "host_rtos_engine_api.h"
#include "vsim_iface.h"

/* The engine's telemetry plumbing, owned here so no firmware struct crosses the
 * ABI. One instance: the engine is single-instance per process anyway (boot is
 * idempotent), so there is nothing to key a second one off. */
static vsim_iface_t g_iface;
static int g_iface_live;

static void module_set_telemetry_sink(vayu_sitl_telemetry_fn cb, void *user) {
  if (!g_iface_live) {
    vsim_iface_init(&g_iface);
    g_iface_live = 1;
  }
  vsim_iface_set_uart2_callback(&g_iface, cb, user);
}

static int module_boot(vayu_sitl_telemetry_fn cb, void *user) {
  /* Re-pointing the sink on a second boot is deliberate: the engine itself
   * only boots once, but a host that stops and restarts its worker gets its new
   * sink honoured instead of silently keeping the old one. */
  module_set_telemetry_sink(cb, user);
  return rtos_engine_boot(&g_iface);
}

/* Process-exit cleanup only. The firmware's threads keep running and still
 * reference this plumbing, so tearing it down while the host is merely paused
 * would pull a live pthread mutex out from under them -- which is why stopping
 * a worker calls set_telemetry_sink(NULL) instead. */
static void module_shutdown(void) {
  if (!g_iface_live)
    return;
  /* Drop the host's callback FIRST, so a frame emitted mid-teardown cannot
   * reach a sink whose owner is already gone. */
  vsim_iface_set_uart2_callback(&g_iface, NULL, NULL);
  vsim_iface_destroy(&g_iface);
  g_iface_live = 0;
}

/* Stamped by the build (see sim/host/CMakeLists.txt). Defaulted so the file
 * still compiles outside it -- an honest "unknown" beats a build error, and
 * beats a stale identity baked in by hand. */
#ifndef VAYU_SITL_BUILD_ID
#define VAYU_SITL_BUILD_ID "unknown"
#endif

static const vayu_sitl_api_t API = {
    .abi_version = VAYU_SITL_ABI_VERSION,
    .build_id = VAYU_SITL_BUILD_ID,

    .boot = module_boot,
    .set_telemetry_sink = module_set_telemetry_sink,
    .shutdown = module_shutdown,
    .enable_serial_rc = rtos_engine_enable_serial_rc,
    .run_begin = rtos_engine_run_begin,
    .run_step = rtos_engine_run_step,

    .get_pose = vsim_inproc_get_pose,

    .reset_to = vsim_inproc_reset_to,
    .set_testrig = vsim_inproc_set_testrig,
    .set_geometry = vsim_inproc_set_geometry,
    .set_world = vsim_inproc_set_world,
    .clear_obstacles = vsim_inproc_clear_obstacles,
    .add_obstacle = vsim_inproc_add_obstacle,
    .set_world_mesh = vsim_inproc_set_world_mesh,
    .clear_world_mesh = vsim_inproc_clear_world_mesh,
    .set_rates = vsim_inproc_set_rates,
    .set_noise = vsim_inproc_set_noise,
    .set_faults = vsim_inproc_set_faults,
    .set_wind = vsim_inproc_set_wind,
    .set_pause = vsim_inproc_set_pause,

    .uart2_rx = rtos_engine_uart2_rx,
};

const vayu_sitl_api_t *vayu_sitl_get_api(uint32_t abi_version) {
  /* Exact match only. A loader built against an older ABI cannot be trusted to
   * have the same struct layouts, and there is no compatibility shim to fall
   * back to -- so refuse rather than hand back a table it will misread. */
  if (abi_version != VAYU_SITL_ABI_VERSION)
    return NULL;
  return &API;
}
