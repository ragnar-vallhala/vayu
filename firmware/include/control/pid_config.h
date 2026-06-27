#ifndef VAYU_PID_CONFIG_H
#define VAYU_PID_CONFIG_H

#include "vayu_status.h"
#include <stdbool.h>
#include <stdint.h>

/* ----------------------------------------------------------------------------
 * CMD_SET_PID (0x000A) — live PID gain update (COMM-CMD-003).
 *
 * Payload schema (little-endian), after the NavLink framing strips the
 * header. Mirrors the calibration command layout: a 2-byte command id, a
 * 1-byte argc, then argc 4-byte arguments.
 *
 *   payload[0..1]   cmd_id  = CMD_SET_PID
 *   payload[2]      argc    (must be >= PID_SET_ARGC)
 *   payload[3..6]   arg0  controller  (float; 0 = angle, 1 = rate)
 *   payload[7..10]  arg1  axis        (float; 0 = roll, 1 = pitch, 2 = yaw)
 *   payload[11..14] arg2  Kp  (float)
 *   payload[15..18] arg3  Ki  (float)
 *   payload[19..22] arg4  Kd  (float)
 *   payload[23..26] arg5  Kff (float)
 *
 * Controller / axis selectors are carried as floats to keep every argument
 * a uniform 4-byte slot (the GCS encodes all command args the same way).
 * --------------------------------------------------------------------------*/
#define PID_SET_ARGC 6

/* ----------------------------------------------------------------------------
 * CMD_SET_GYRO_LPF (0x000B) — live rate-loop gyro low-pass update.
 *
 *   payload[0..1]   cmd_id  = CMD_SET_GYRO_LPF
 *   payload[2]      argc    (must be >= GYRO_LPF_ARGC)
 *   payload[3..6]   arg0  axis  (float; 0 = roll, 1 = pitch, 2 = yaw)
 *   payload[7..10]  arg1  rc    (float; gyro LPF time constant [s], <=0 = off)
 * --------------------------------------------------------------------------*/
#define GYRO_LPF_ARGC 2

/* ----------------------------------------------------------------------------
 * CMD_SET_D_LPF (0x000E) — live rate-loop derivative (D-term) low-pass update.
 * Same payload shape as CMD_SET_GYRO_LPF, but targets the rate PID's D filter.
 *
 *   payload[0..1]   cmd_id  = CMD_SET_D_LPF
 *   payload[2]      argc    (must be >= D_LPF_ARGC)
 *   payload[3..6]   arg0  axis  (float; 0 = roll, 1 = pitch, 2 = yaw)
 *   payload[7..10]  arg1  rc    (float; D-term LPF time constant [s], <=0 = off)
 * --------------------------------------------------------------------------*/
#define D_LPF_ARGC 2

typedef enum {
  PID_CTRL_ANGLE = 0,
  PID_CTRL_RATE  = 1,
  PID_CTRL_COUNT = 2,
} pid_ctrl_sel_t;

/**
 * @brief Load any persisted PID gains from SD into the in-memory store.
 *
 * Call once at boot, after the SD/VFS layer is up (fs_owner_boot_init) and
 * before the scheduler starts. The controllers consult the store via
 * pid_config_get_*() during their own init, so order between this call
 * and the (later) controller init is irrelevant.
 *
 * @implements COMM-CMD-003
 */
void pid_config_init(void);

/**
 * @brief Fetch the stored gains for one (controller, axis), if any.
 * @return true and fills the out-params if a persisted value exists for
 *         this slot; false (out-params untouched) otherwise.
 * @implements COMM-CMD-003
 */
bool pid_config_get_rate(uint8_t axis, float *kp, float *ki, float *kd,
                         float *kff);
/** @implements COMM-CMD-003 */
bool pid_config_get_angle(uint8_t axis, float *kp, float *ki, float *kd,
                          float *kff);

/**
 * @brief Validate and apply a CMD_SET_PID payload: parse the schema, push
 *        the gains to the live controller, and persist the full set to SD.
 *
 * Performs the COMM-CMD-002 length/argc validation before reading args.
 *
 * @param payload      the command payload (payload[0..1] == CMD_SET_PID).
 * @param payload_len  payload byte count from the framing layer.
 * @return VAYU_OK on apply; VAYU_ERR_INVALID on a malformed/short payload
 *         or out-of-range selector.
 * @implements COMM-CMD-003, COMM-CMD-002
 */
vayu_status_t pid_config_apply_command(const uint8_t *payload,
                                       uint16_t payload_len);

/**
 * @brief Fetch the stored gyro LPF time constant for one rate axis, if any.
 * @return true + fills *rc if a persisted value exists; false otherwise.
 */
bool pid_config_get_gyro_lpf(uint8_t axis, float *rc);

/**
 * @brief Validate and apply a CMD_SET_GYRO_LPF payload: parse [axis, rc],
 *        push to the live rate controller, and persist.
 */
vayu_status_t pid_config_apply_gyro_lpf_command(const uint8_t *payload,
                                                uint16_t payload_len);

/**
 * @brief Fetch the stored D-term LPF time constant for one rate axis, if any.
 * @return true + fills *rc if a persisted value exists; false otherwise.
 */
bool pid_config_get_d_lpf(uint8_t axis, float *rc);

/**
 * @brief Validate and apply a CMD_SET_D_LPF payload: parse [axis, rc],
 *        push to the live rate controller's D filter, and persist.
 */
vayu_status_t pid_config_apply_d_lpf_command(const uint8_t *payload,
                                             uint16_t payload_len);

#endif // VAYU_PID_CONFIG_H
