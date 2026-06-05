#include "comm/comm_types.h"
#include "comm/ibus.h"
#include "comm/serializer.h"
#include "control/control.h"
#include "memory.h"
#include "sensor/sensor.h"
#include "sys/state.h"
#include "task.h"
#include "utils.h"
#include "sys/sys_utils.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_status.h"
#include "vayu_tasks.h"
#include "vayu_tasks.h"
#include <stdbool.h>
#include <stdint.h>
static uint32_t _calibration_task_handle = 0;

/* COMM-CMD-002: a command payload is laid out as
 *   [cmd_id:2][argc:1][arg0:4][arg1:4]...
 * so reading `argc` 4-byte args requires the framing layer to have
 * delivered at least argc*4 + 3 payload bytes. Validate both argc and the
 * payload length before any handler dereferences an argument. `len` is
 * pkt.length — the payload byte count produced by the deserializer.
 * @implements COMM-CMD-002 */
static bool command_payload_valid(uint16_t len, uint8_t argc,
                                  uint8_t expected_argc) {
  if (argc < expected_argc) {
    return false;
  }
  return len >= (uint16_t)argc * 4u + 3u;
}
/* Dispatch a single received packet to its handler. Extracted from
 * comm_processor_task's loop so the GCS -> FC command path can be exercised
 * end-to-end in SITL (deserialize a real GCS frame -> dispatch), independent
 * of the blocking RX queue. */
void comm_processor_dispatch(const packet_t *pkt) {
  uint8_t packet_type = (pkt->protocol_packet_type >> 4) & 0x0F;

  if (packet_type == PACKET_TYPE_HEARTBEAT) {
    set_timestamp(pkt->timestamp);
    set_device_id(pkt->device_id);
  } else if (packet_type == PACKET_TYPE_COMMAND && pkt->length >= 2) {
    uint16_t cmd_id;
    v_memcpy(&cmd_id, pkt->payload, 2);
    /* argc is payload[2]; only present when length >= 3. */
    uint8_t argc = (pkt->length >= 3) ? pkt->payload[2] : 0;

    if (cmd_id == CMD_CALIBRATE_IMU) {
      /* COMM-CMD-002: need 2 args (imu_id, type) -> argc>=2 and a
       * payload long enough to actually hold them. */
      if (system_state_get() != SYSTEM_STATE_CALIBRATING &&
          command_payload_valid(pkt->length, argc, 2)) {
        calibration_args_t *cal_args =
            (calibration_args_t *)v_malloc(sizeof(calibration_args_t));
        if (cal_args != NULL) {
          v_memcpy(&cal_args->imu_id, &pkt->payload[3], 4);
          v_memcpy(&cal_args->type, &pkt->payload[7], 4);
        }
        _calibration_task_handle =
            task_create(calibration_task, cal_args, 4096, 0);
      }
    } else if (cmd_id == 0x0009) { // CMD_CANCEL_CALIBRATION
      if (_calibration_task_handle != 0) {
        task_exit_request(_calibration_task_handle);
        _calibration_task_handle = 0;
      }
      VAYU_DISCARD(system_state_set(SYSTEM_STATE_STANDBY));
    } else if (cmd_id == CMD_ARM) {
      /* GCS software-arm: set the latch. The RC task evaluates it (OR'd
       * with the physical arm switch via rc_arm_engaged) against the arm
       * preconditions on its next frame, so throttle/link/estimator gates
       * still apply. Lets a 4-channel stick with no arm channel arm from
       * the GCS. */
      g_sw_arm_request = 1;
    } else if (cmd_id == CMD_DISARM) {
      /* Clear the latch; the RC task disarms to STANDBY on its next frame
       * (mirrors releasing the arm switch). */
      g_sw_arm_request = 0;
    } else if (cmd_id == CMD_SET_PID) {
      /* COMM-CMD-003: validate, apply to the live controller, persist. */
      VAYU_DISCARD(pid_config_apply_command(pkt->payload, pkt->length));
    } else if (cmd_id == CMD_SET_GYRO_LPF) {
      /* Live rate-loop gyro LPF update (co-tuned with the gains). */
      VAYU_DISCARD(pid_config_apply_gyro_lpf_command(pkt->payload, pkt->length));
    } else if (cmd_id == CMD_SET_MOTOR_GEOMETRY) {
      /* Set the mixer signs from the airframe motor layout (sim/vehicle). */
      VAYU_DISCARD(
          angle_rate_controller_apply_geometry_command(pkt->payload, pkt->length));
    }
  }
}

void comm_processor_task(void *args) {
  (void)args;
  packet_t pkt;

  while (1) {
    if (get_next_rx_packet(&pkt) == NONE) {
      comm_processor_dispatch(&pkt);
    } else {
      v_delay(4); // Wait for more packets
    }
  }
}
