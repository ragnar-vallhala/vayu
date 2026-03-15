#include "comm/comm_types.h"
#include "comm/serializer.h"
#include "sys/state.h"
#include "utils.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_tasks.h"

void calibration_task(void *args) {
  (void)args;

  system_state_set(SYSTEM_STATE_CALIBRATING);

  uint8_t payload[18];
  payload[0] = 0x01; // SYSTEM_ORIGIN_CALIBRATION
  payload[1] = 0x04; // n = 4 values

  // Fill 0.0f for bias_x/y/z initially
  float zero = 0.0f;
  v_memcpy(&payload[6], &zero, 4);
  v_memcpy(&payload[10], &zero, 4);
  v_memcpy(&payload[14], &zero, 4);

  for (int i = 0; i <= 10; i++) {
    if (system_state_get() != SYSTEM_STATE_CALIBRATING) {
      break;
    }
    float progress = (float)i * 10.0f;
    v_memcpy(&payload[2], &progress, 4);

    send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS, payload, 18);

    if (i < 10) {
      v_delay(500); // 10 steps of 500ms = 5s total
    }
  }

  system_state_set(SYSTEM_STATE_STANDBY);
  return;
}
