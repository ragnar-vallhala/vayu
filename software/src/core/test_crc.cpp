#include "../comm/comm_types.h"
#include "crc.h"
#include <cstring>
#include <iomanip>
#include <iostream>

// Simulated Heartbeat packet payload content representation without CRC since
// CRC is appended
void test() {
  packet_t packet = {};
  packet.sync = SYNC_BYTE;
  packet.protocol_packet_type = (0x0 << 4) | 0x1;
  packet.length = 0;
  packet.device_id = 42;
  packet.timestamp = 123456789;

  uint8_t header_size = 8;
  uint32_t expected_crc =
      CRC32::calculate((const uint8_t *)&packet, header_size);
  std::cout << "Computed CRC32: 0x" << std::hex << expected_crc << std::endl;
}

int main() {
  test();
  return 0;
}
