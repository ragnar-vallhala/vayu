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
 * host_rtos_engine_api.h -- the GCS-facing slice of the RTOS SITL engine.
 *
 * SimWorker (a Qt C++ TU) drives the in-process engine through these calls and
 * must NOT pull in the firmware headers (control_buffer.h / sensor.h etc. shadow
 * system headers for C++). So this header declares ONLY the type-free engine
 * entry points plus the vsim_proto.h wire structs SimWorker already uses — the
 * full host_rtos_engine.h (with firmware types) is for the C TUs that implement
 * the loop. Both declare the same extern "C" symbols; signatures match.
 *
 * Model: the engine runs BOTH firmware and physics in one stepper (no vsim_d, no
 * FIFO). The worker thread: boot(iface) -> enable_serial_rc -> run_begin -> loop
 * { apply queued config via vsim_inproc_set_*; run_step; get_pose -> render }.
 */
#ifndef VAYU_HOST_RTOS_ENGINE_API_H
#define VAYU_HOST_RTOS_ENGINE_API_H

#include <stddef.h>

#include "vsim_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Lifecycle (see host_rtos_engine.h for full docs). boot is idempotent. */
int rtos_engine_boot(void *iface); /* iface = vsim_iface_t* (telemetry cb) */
void rtos_engine_enable_serial_rc(void); /* RC from VAYU_UART_RC_PATH */
void rtos_engine_run_begin(void);        /* reset internal stepper + pacer */
void rtos_engine_run_step(void);         /* one 1 ms step, wall-clock paced */

/* Feed bytes to the firmware's UART2 receiver, as if a GCS had sent them on
 * the wire: the real parser, the real router, the real command gates. */
void rtos_engine_uart2_rx(const uint8_t *data, size_t len);

/* Latest pose snapshot for the renderer (seqlock; safe off-thread). */
void vsim_inproc_get_pose(vsim_pose_frame_t *out);

/* Config surface — one call per VSIM_CTL_* message (apply on the worker thread
 * only; the engine is single-threaded). */
void vsim_inproc_reset_to(const vsim_ctl_reset_t *b);
void vsim_inproc_set_testrig(const vsim_ctl_testrig_t *t);
void vsim_inproc_set_geometry(const vsim_ctl_geometry_t *g);
void vsim_inproc_set_world(const vsim_ctl_world_t *w);
void vsim_inproc_clear_obstacles(void);
void vsim_inproc_add_obstacle(const vsim_ctl_obstacle_t *b);
int vsim_inproc_set_world_mesh(const vsim_ctl_world_mesh_t *m);
void vsim_inproc_clear_world_mesh(void);
void vsim_inproc_set_rates(const vsim_ctl_rates_t *r);
void vsim_inproc_set_noise(const vsim_ctl_noise_t *n);
void vsim_inproc_set_faults(const vsim_ctl_faults_t *f);
void vsim_inproc_set_wind(const vsim_ctl_wind_t *w);
void vsim_inproc_set_pause(int paused);

#ifdef __cplusplus
}
#endif

#endif /* VAYU_HOST_RTOS_ENGINE_API_H */
