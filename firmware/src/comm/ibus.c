/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "comm/ibus.h"
#include <string.h>

static ibus_state_t state = IBUS_STATE_WAIT_START;
static uint8_t buffer[IBUS_PACKET_SIZE];
static uint8_t idx = 0;
static uint16_t checksum = 0xFFFF;

/** @noreq parser state reset; no behavioral requirement */
void ibus_init(ibus_data_t *data) {
  if (data) {
    memset(data, 0, sizeof(ibus_data_t));
  }
  state = IBUS_STATE_WAIT_START;
  idx = 0;
}

/** @implements COMM-RC-001 */
bool ibus_parse_byte(uint8_t b, ibus_data_t *data) {
  switch (state) {
  case IBUS_STATE_WAIT_START:
    if (b == IBUS_START_BYTE) {
      buffer[0] = b;
      checksum = 0xFFFF - b;
      idx = 1;
      state = IBUS_STATE_WAIT_CMD;
    }
    break;

  case IBUS_STATE_WAIT_CMD:
    if (b == IBUS_CMD_CHANNELS) {
      buffer[1] = b;
      checksum -= b;
      idx = 2;
      state = IBUS_STATE_PAYLOAD;
    } else {
      state = IBUS_STATE_WAIT_START;
    }
    break;

  case IBUS_STATE_PAYLOAD:
    buffer[idx++] = b;
    checksum -= b;
    if (idx >= 30) {
      state = IBUS_STATE_CHECKSUM_L;
    }
    break;

  case IBUS_STATE_CHECKSUM_L:
    if (b == (checksum & 0xFF)) {
      state = IBUS_STATE_CHECKSUM_H;
    } else {
      state = IBUS_STATE_WAIT_START;
    }
    break;

  case IBUS_STATE_CHECKSUM_H:
    if (b == ((checksum >> 8) & 0xFF)) {
      // Packet valid!
      if (data) {
        for (int i = 0; i < IBUS_MAX_CHANNELS; i++) {
          data->channels[i] =
              (uint16_t)(buffer[2 + i * 2] | (buffer[3 + i * 2] << 8));
        }
        /* FlySky has no in-protocol failsafe flag — link-loss is inferred from
         * the throttle channel (rc_throttle_failsafe_step) and the staleness
         * watchdog, set by the RC task. Leave it clear here. */
        data->is_failsafe = false;
      }
      state = IBUS_STATE_WAIT_START;
      return true;
    }
    state = IBUS_STATE_WAIT_START;
    break;
  }
  return false;
}
