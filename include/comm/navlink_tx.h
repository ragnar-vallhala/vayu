#ifndef VAYU_NAVLINK_TX_H
#define VAYU_NAVLINK_TX_H

/* TX half of the NavLink seam (RX half: comm/navlink_router.h).
 *
 * The single home for firmware telemetry/command-response *encoding*: this and
 * navlink_router.c are the only TUs that include the generated codec or call
 * send_packet()/write_channel(). Telemetry producers (telemetry_task.c,
 * comm_processor.c) hand domain data to the navlink_tx_* publishers below and
 * stay codec-blind. The firmware now emits NavLink v2 exclusively — every
 * telemetry and command-response message goes out as a typed v2 frame; the v1
 * wire path is retired. See navlink/INTEGRATION.md. */

#include "comm/comm_types.h"    /* time_sync_payload_t, PACKET_TYPE_*, origins */
#include "comm/ibus.h"          /* ibus_data_t */
#include "comm/perf_packet.h"   /* perf_global_body_t, perf_task_row_t, perf_fifo_row_t */
#include "actuator/actuator.h"  /* motor_outputs_t */
#include "est/est.h"            /* attitude_t, est_perf_telemetry_t */
#include "variables.h"          /* control_telemetry_t */
#include <stdint.h>

/* --- periodic telemetry (FC -> GCS) --------------------------------------- */
void navlink_tx_log(const char *buf, uint8_t len);            /* v2 STATUSTEXT */
void navlink_tx_heartbeat(void);                              /* v2 HEARTBEAT (+nav_state) */
void navlink_tx_flight_mode(uint8_t mode, uint8_t source);    /* v2 FLIGHT_MODE */
void navlink_tx_health(uint32_t tx_overflow, uint32_t imu_drop,
                       uint32_t log_wrap);                    /* v2 SYSTEM_HEALTH */
void navlink_tx_pid_error(const control_telemetry_t *c);      /* v2 CONTROL_TRACE */
void navlink_tx_est_perf(const est_perf_telemetry_t *e);      /* v2 EST_PERF */
void navlink_tx_imu_full(const float floats10[10]);           /* v2 IMU_RAW */
void navlink_tx_imu_compressed(const uint16_t delta_f16[10]); /* v2 IMU_COMPRESSED */
void navlink_tx_attitude(const attitude_t *att_deg);          /* v2 ATTITUDE_EULER */
void navlink_tx_rc_channels(const ibus_data_t *rc);           /* v2 RC_CHANNELS */
void navlink_tx_motor(const motor_outputs_t *m);              /* v2 MOTOR_TELEMETRY */
void navlink_tx_calibration(const uint8_t *buf,
                            uint8_t len);                     /* v2 CALIBRATION_STATUS */

/* PERF report (msgid 1034/1035/1036). The v1 fragmented PERF_STATS becomes one
 * v2 message per row; `seq` ties a report's GLOBAL/TASK/FIFO messages together.
 * perf_telemetry.c gathers the firmware structs and hands them here. */
void navlink_tx_perf_global(const perf_global_body_t *g, uint32_t seq);
void navlink_tx_perf_task(const perf_task_row_t *row, uint32_t seq);
void navlink_tx_perf_fifo(const perf_fifo_row_t *row, uint32_t seq);

/* --- command responses (moved out of comm_processor.c) -------------------- */
void navlink_tx_time_sync_response(const time_sync_payload_t *out);
void navlink_tx_perf_taskname(uint8_t id, const char *name);

#endif /* VAYU_NAVLINK_TX_H */
