#include "comm/deserializer.h"
#include "core/cortex-m4/crc.h"
#include "utils.h"
#include "vaios.h"
#include <stddef.h>

void deserializer_init(deserializer_t *d) {
  v_memset(d, 0, sizeof(deserializer_t));
  d->state = STATE_SYNC;
}

int deserializer_feed(deserializer_t *d, uint8_t b) {
  switch (d->state) {
  case STATE_SYNC:
    if (b == SYNC_BYTE) {
      d->packet.sync = b;
      d->state = STATE_HEADER;
      d->bytes_read = 1; // Already have sync
    }
    break;

  case STATE_HEADER:
    ((uint8_t *)&d->packet)[d->bytes_read++] = b;
    if (d->bytes_read == 8) { // Header is 8 bytes
      if (d->packet.length == 0) {
        d->state = STATE_CRC;
        d->bytes_read = 0;
      } else {
        d->state = STATE_PAYLOAD;
        d->bytes_read = 0;
      }
    }
    break;

  case STATE_PAYLOAD:
    d->packet.payload[d->bytes_read++] = b;
    if (d->bytes_read == d->packet.length) {
      d->state = STATE_CRC;
      d->bytes_read = 0;
    }
    break;

  case STATE_CRC:
    ((uint8_t *)&d->packet.crc32)[d->bytes_read++] = b;
    if (d->bytes_read == 4) {
      // Validate CRC
      hal_crc_reset();
      hal_crc_accumulate((uint8_t *)&d->packet, 8); // Header
      if (d->packet.length > 0) {
        hal_crc_accumulate(d->packet.payload, d->packet.length);
      }
      uint32_t computed = hal_crc_accumulate(NULL, 0); // Get result

      if (computed == d->packet.crc32) {
        d->state = STATE_SYNC;
        d->bytes_read = 0;
        return 1; // Valid packet!
      } else {
        deserializer_init(d); // Malformed, restart
      }
    }
    break;
  }
  return 0;
}
