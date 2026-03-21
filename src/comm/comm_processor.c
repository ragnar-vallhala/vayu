#include "comm/comm_types.h"
#include "comm/serializer.h"
#include "core/cortex-m4/uart.h"
#include "sys/state.h"
#include "task.h"
#include "utils.h"
#include "utils/utils.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_tasks.h"
#include <stdint.h>

void comm_processor_task(void *args) {
  (void)args;
  packet_t pkt;

  while (1) {
    if (get_next_rx_packet(&pkt) == NONE) {
      // uart2_write("Comm Processor started");
      // Handle packet
      uint8_t packet_type = (pkt.protocol_packet_type >> 4) & 0x0F;

      if (packet_type == PACKET_TYPE_HEARTBEAT) {
        set_timestamp(pkt.timestamp);
        set_device_id(pkt.device_id);
      } else if (packet_type == PACKET_TYPE_COMMAND) {
        uint16_t cmd_id;
        v_memcpy(&cmd_id, pkt.payload, 2);
        if (cmd_id == 0x0006) { // CMD_CALIBRATE_GYR
          if (system_state_get() != SYSTEM_STATE_CALIBRATING) {
            task_create(calibration_task, NULL, 4096, 0);
          }
        } else if (cmd_id == 0x0009) { // CMD_CANCEL_CALIBRATION
          system_state_set(SYSTEM_STATE_STANDBY);
        }
      }
    } else {
      v_delay(10); // Wait for more packets
    }
  }
}
