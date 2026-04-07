#include "comm/comm_types.h"
#include "comm/serializer.h"
#include "maths/control.h"
#include "maths/control_buffer.h"
#include "memory.h"
#include "sensor/bmx160.h"
#include "sys/state.h"
#include "task.h"
#include "utils.h"
#include "utils/utils.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_tasks.h"
#include <stdint.h>
static uint32_t _calibration_task_handle = 0;
void comm_processor_task(void *args) {
  (void)args;
  packet_t pkt;

  while (1) {
    if (get_next_rx_packet(&pkt) == NONE) {
      uint8_t packet_type = (pkt.protocol_packet_type >> 4) & 0x0F;

      if (packet_type == PACKET_TYPE_HEARTBEAT) {
        set_timestamp(pkt.timestamp);
        set_device_id(pkt.device_id);
      } else if (packet_type == PACKET_TYPE_COMMAND) {
        uint16_t cmd_id;
        v_memcpy(&cmd_id, pkt.payload, 2);

        if (cmd_id == CMD_CALIBRATE_IMU) {
          if (system_state_get() != SYSTEM_STATE_CALIBRATING) {
            uint8_t argc = pkt.payload[2];
            calibration_args_t *cal_args = NULL;
            if (argc >= 2) {
              cal_args =
                  (calibration_args_t *)v_malloc(sizeof(calibration_args_t));
              if (cal_args != NULL) {
                v_memcpy(&cal_args->imu_id, &pkt.payload[3], 4);
                v_memcpy(&cal_args->type, &pkt.payload[7], 4);
              }
            }
            _calibration_task_handle =
                task_create(calibration_task, cal_args, 4096, 0);
          }
        } else if (cmd_id == 0x0009) { // CMD_CANCEL_CALIBRATION
          if (_calibration_task_handle != 0) {
            task_exit_request(_calibration_task_handle);
            _calibration_task_handle = 0;
          }
          system_state_set(SYSTEM_STATE_STANDBY);
        } else if (cmd_id == CMD_SET_PID) {
          control_config_t new_config;
          if (pkt.length >= 2 + sizeof(control_config_t)) {
            v_memcpy(&new_config, &pkt.payload[2], sizeof(control_config_t));
            pid_config_t2c_push(&new_config);
          }
        }
      }
    } else {
      v_delay(4); // Wait for more packets
    }
  }
}
