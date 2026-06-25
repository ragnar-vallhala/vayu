#include "comm/comm.h"
#include "storage/fs_owner.h"
#include "navhal.h"
#include "vaios.h"
#include "variables.h"
#include "vfs.h"
int write_pos = 0;
void test_task(void *args) {
  while (1) {
    // send_packet(&g_telemetry_channel, PACKET_TYPE_LOG, (byte *)"Hello", 5);
    fs_owner_enqueue_log(GENERAL_LOGGER, (byte *)"Hello How are you?", 18);
    v_delay(500);
  }
}
