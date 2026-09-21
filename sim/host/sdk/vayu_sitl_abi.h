/*
 * vayu_sitl_abi.h -- the loadable SITL module's contract.
 *
 * libvayu_sitl is the firmware compiled for the host (real vaios scheduler +
 * real control code + in-process vsim physics, one deterministic stepper),
 * built as a shared module and loaded at runtime by whoever wants an in-process
 * simulator. Navigator is the first such host; nothing about this header is
 * Navigator-specific.
 *
 * The module exports exactly ONE symbol, vayu_sitl_get_api. Everything else is
 * reached through the returned vtable, so adding a call is an ABI version bump
 * in one place rather than another symbol for each loader to resolve.
 *
 * This header, vsim_proto.h and the world-mesh pair (trimesh_bvh.h +
 * vsim_math.h) are the whole compile-time surface: a loader needs those files
 * and nothing else from the firmware tree. The world-mesh pair is here because
 * it is a two-way contract -- the host BUILDS a BVH file and the module mmaps
 * it (VSIM_CTL_SET_WORLD_MESH passes a path, not the data) -- so both sides
 * must agree on the layout. In particular
 * it must NOT reach into sim/host/include -- those are NavHAL port shims that
 * shadow system headers (atomic.h, family) and will break a C++ translation
 * unit that picks them up. That is also why telemetry is delivered to a plain
 * callback rather than through vsim_iface_t: the host would otherwise have to
 * size a firmware struct, which both drags that header out of the tree and
 * turns every field added to it into an ABI break.
 *
 * Threading: the engine is single-threaded and holds global state, so ONE
 * instance per process. Call every entry point from the same thread, except
 * get_pose, which is a seqlock read and is safe from any thread. Running two
 * simulations at once means two processes, not two loads.
 */
#ifndef VAYU_SITL_ABI_H
#define VAYU_SITL_ABI_H

#include <stddef.h>
#include <stdint.h>

#include "vsim_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bump on ANY change to vayu_sitl_api_t: added call, changed signature, or a
 * changed struct in vsim_proto.h. The loader passes the version it was built
 * against and gets NULL if the module cannot serve it -- an honest failure to
 * load beats a silently mismatched struct layout. */
#define VAYU_SITL_ABI_VERSION 3u

/* The name the loader resolves, and the module's file name without the
 * platform's prefix/suffix (libvayu_sitl.so, vayu_sitl.dll). */
#define VAYU_SITL_ENTRY_SYMBOL "vayu_sitl_get_api"
#define VAYU_SITL_MODULE_NAME "vayu_sitl"

/* Telemetry sink: the engine hands the host each run of UART2 bytes the
 * firmware emits, exactly as they would leave the real board's serial port. */
typedef void (*vayu_sitl_telemetry_fn)(void *user, const uint8_t *data,
                                       size_t len);

typedef struct vayu_sitl_api {
  /* Mirrors the version the loader asked for; re-checkable after the call. */
  uint32_t abi_version;

  /* WHICH firmware this is: a NUL-terminated build identity, normally
   * `git describe --always --dirty` plus the build date, or "unknown" if the
   * module was built outside a git tree.
   *
   * abi_version gates struct layout, not identity -- any firmware built
   * against this ABI loads, which is the point of shipping the engine
   * separately. The cost is that a host can be driving a months-old engine
   * with nothing on screen to say so, the same blind spot as a real board,
   * which carries no version on the wire either. So the module states it, and
   * a host should log it at load and show it wherever it names the sim.
   *
   * Static storage owned by the module; valid until unload, never freed. */
  const char *build_id;

  /* ---- lifecycle ----
   * boot wires telemetry to `cb` and starts the vaios scheduler. It is
   * idempotent: a second call is a no-op returning 0, because the kernel boots
   * once per process (a host's Stop/Start resumes the step loop rather than
   * re-initialising). Returns non-zero if the firmware failed to start.
   *
   * `cb` is called from the engine's thread as the firmware emits UART2 bytes;
   * keep it short and do not call back into the API from it. `user` is passed
   * through untouched.
   *
   * The engine owns its telemetry plumbing: the host passes a plain function
   * pointer and never sees, sizes or allocates a firmware struct, so growing
   * that struct is not an ABI break. */
  int (*boot)(vayu_sitl_telemetry_fn cb, void *user);

  /* Re-point or detach the telemetry sink without touching lifecycle. Pass
   * NULL to go quiet -- which is what a host wants when it stops its worker,
   * since the firmware's tasks cannot actually be stopped and would otherwise
   * keep calling a sink whose owner has moved on.
   *
   * This is NOT shutdown(): the plumbing stays constructed, because firmware
   * threads still reference it. Detaching is safe at any time; tearing down is
   * only safe as the process exits. */
  void (*set_telemetry_sink)(vayu_sitl_telemetry_fn cb, void *user);

  /* Release the telemetry plumbing. Call once, as the process is exiting: the
   * firmware's threads keep running and still hold references, so this is a
   * last-gasp cleanup rather than something to pair with each boot. */
  void (*shutdown)(void);
  void (*enable_serial_rc)(void); /* RC from VAYU_UART_RC_PATH */
  void (*run_begin)(void);        /* reset internal stepper + pacer */
  void (*run_step)(void);         /* one 1 ms step, wall-clock paced */

  /* ---- latest pose snapshot for a renderer (seqlock; safe off-thread) ---- */
  void (*get_pose)(vsim_pose_frame_t *out);

  /* ---- config surface: one call per VSIM_CTL_* message ---- */
  void (*reset_to)(const vsim_ctl_reset_t *b);
  void (*set_testrig)(const vsim_ctl_testrig_t *t);
  void (*set_geometry)(const vsim_ctl_geometry_t *g);
  void (*set_world)(const vsim_ctl_world_t *w);
  void (*clear_obstacles)(void);
  void (*add_obstacle)(const vsim_ctl_obstacle_t *b);
  int (*set_world_mesh)(const vsim_ctl_world_mesh_t *m);
  void (*clear_world_mesh)(void);
  void (*set_rates)(const vsim_ctl_rates_t *r);
  void (*set_noise)(const vsim_ctl_noise_t *n);
  void (*set_faults)(const vsim_ctl_faults_t *f);
  void (*set_wind)(const vsim_ctl_wind_t *w);
  void (*set_pause)(int paused);

  /* ---- GCS -> FC ----
   * Hand the engine bytes as if a ground station had sent them on the wire.
   * They go through the firmware's real parser, router and command gates, so
   * a host drives the simulated FC with the SAME NavLink frames it sends to a
   * real board -- including the time-sync handshake commands are gated on.
   *
   * This replaced a set of direct firmware entry points a host used to call.
   * Those bypassed the gates, which is how a geometry command once appeared to
   * work in the in-app sim while the firmware silently flew the default mix. */
  void (*uart2_rx)(const uint8_t *data, size_t len);
} vayu_sitl_api_t;

/* The module builds with -fvisibility=hidden so the firmware's globals stay out
 * of the loader's symbol namespace, so the one symbol we DO export has to say
 * so explicitly -- a version script cannot re-export what the compiler already
 * hid. A loader never references the symbol directly (it resolves it by name),
 * so this expands to nothing unless the module itself is being built. */
#if defined(VAYU_SITL_BUILDING_MODULE)
#if defined(_WIN32)
#define VAYU_SITL_EXPORT __declspec(dllexport)
#else
#define VAYU_SITL_EXPORT __attribute__((visibility("default")))
#endif
#else
#define VAYU_SITL_EXPORT
#endif

/* Returns NULL if the module cannot serve the requested ABI version. The
 * returned table is static storage owned by the module: valid until unload,
 * and the caller must not free or modify it. */
VAYU_SITL_EXPORT const vayu_sitl_api_t *vayu_sitl_get_api(uint32_t abi_version);

/* Signature for the loader's dlsym/QLibrary::resolve result. */
typedef const vayu_sitl_api_t *(*vayu_sitl_get_api_fn)(uint32_t);

#ifdef __cplusplus
}
#endif

#endif /* VAYU_SITL_ABI_H */
