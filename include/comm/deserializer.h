#ifndef DESERIALIZER_H
#define DESERIALIZER_H

#include "comm/comm_types.h"
#include <stdint.h>

/**
 * @brief State machine for binary packet deserialization
 */
typedef enum {
  STATE_SYNC,
  STATE_HEADER,
  STATE_PAYLOAD,
  STATE_CRC
} deserialize_state_t;

typedef struct {
  deserialize_state_t state;
  packet_t packet;
  uint16_t bytes_read;
} deserializer_t;

void deserializer_init(deserializer_t *d);
int deserializer_feed(deserializer_t *d, uint8_t byte);

#endif // !DESERIALIZER_H
