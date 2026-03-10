#ifndef VAYU_COMM_IBUS_H
#define VAYU_COMM_IBUS_H

#include <stdbool.h>
#include <stdint.h>

#define IBUS_MAX_CHANNELS 14
#define IBUS_PACKET_SIZE 32
#define IBUS_START_BYTE 0x20
#define IBUS_CMD_CHANNELS 0x40

typedef struct {
  uint16_t channels[IBUS_MAX_CHANNELS];
  bool is_failsafe;
} ibus_data_t;

typedef enum {
  IBUS_STATE_WAIT_START,
  IBUS_STATE_WAIT_CMD,
  IBUS_STATE_PAYLOAD,
  IBUS_STATE_CHECKSUM_L,
  IBUS_STATE_CHECKSUM_H
} ibus_state_t;

void ibus_init(ibus_data_t *data);
bool ibus_parse_byte(uint8_t byte, ibus_data_t *data);

extern uint16_t rc_channels[IBUS_MAX_CHANNELS];

#endif // VAYU_COMM_IBUS_H
