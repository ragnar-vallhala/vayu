#include "comm/comm_types.h"
#include "comm/serializer.h"
#include "logger/logger.h"
#include "navhal.h"
#include "vaios.h"
#include "variables.h"
#include "vfs.h"
int write_pos = 0;
void test_task(void *args) {
  while (1) {
    send_packet(&g_telemetry_channel, PACKET_TYPE_LOG, (byte *)"Hello", 5);
    v_delay(500);
  }
}