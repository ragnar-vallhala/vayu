#ifndef VAYU_NAVLINK_TX_H
#define VAYU_NAVLINK_TX_H

/* TX half of the NavLink seam (RX half: comm/navlink_router.h).
 *
 * The single home for firmware telemetry/command-response *encoding*: this and
 * navlink_router.c are the only TUs that include the generated codec or call
 * send_packet()/write_channel(). Telemetry producers (telemetry_task.c,
 * comm_processor.c) hand domain data to the navlink_tx_* publishers below and
 * stay codec-blind — so migrating a message v1->v2 is a one-line change *here*,
 * invisible to the producers. Migration is in progress: the dashboard telemetry
 * set (ATTITUDE, IMU full/compressed, RC, MOTOR, FLIGHT_MODE, SYSTEM_HEALTH,
 * EST_PERF, CONTROL_TRACE) now emits NavLink v2; the rest (heartbeat,
 * system-state, log, calibration, time-sync, perf-taskname) is still v1
 * (wrapped send_packet) pending its own migration. See navlink/INTEGRATION.md.
 *
 * EXCEPTION: perf_telemetry.c keeps its own fragmented PERF_STATS framing (it
 * carries no codec dependency); it joins this seam when PERF itself migrates. */

#include "comm/comm_types.h"    /* time_sync_payload_t, PACKET_TYPE_*, origins */
#include "comm/ibus.h"          /* ibus_data_t */
#include "actuator/actuator.h"  /* motor_outputs_t */
#include "est/est.h"            /* attitude_t, est_perf_telemetry_t */
#include "variables.h"          /* control_telemetry_t */
#include <stdint.h>

/* --- periodic telemetry (FC -> GCS) --------------------------------------- */
void navlink_tx_log(const char *buf, uint8_t len);
void navlink_tx_heartbeat(void);
void navlink_tx_system_state(int sys_state);                  /* v1 SYSTEM_STATUS 0x04 */
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
                            uint8_t len);                     /* SYSTEM_STATUS (calib blob) */

/* --- command responses (moved out of comm_processor.c) -------------------- */
void navlink_tx_time_sync_response(const time_sync_payload_t *out);
void navlink_tx_perf_taskname(uint8_t id, const char *name);

#endif /* VAYU_NAVLINK_TX_H */
