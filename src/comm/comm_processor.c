#include "comm/channel.h"
#include "comm/serializer.h"
#include "utils/utils.h"
#include "vaios.h"

void comm_processor_task(void *args) {
  (void)args;
  packet_t pkt;

  while (1) {
    if (get_next_rx_packet(&pkt) == NONE) {
      // Handle packet
      uint8_t packet_type = (pkt.protocol_packet_type >> 4) & 0x0F;

      if (packet_type == PACKET_TYPE_HEARTBEAT) {
        set_timestamp(pkt.timestamp);
        set_device_id(pkt.device_id);
      }
    } else {
      v_delay(10); // Wait for more packets
    }
  }
}
