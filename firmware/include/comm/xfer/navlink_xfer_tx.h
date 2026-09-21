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
/**
 * @file include/comm/xfer/navlink_xfer_tx.h
 * @brief Codec/channel seam for the xfer substrate.
 *
 * The ONLY xfer file that includes the generated navlink codec. Provides the
 * xfer_tx_ops_t the core SM emits through (g_xfer_tx_ops) plus the pure
 * frame-builders it wraps (exposed so the host test can decode + verify the
 * exact field mapping the firmware uses). Frames go to g_telemetry_channel,
 * same as navlink_tx.c.
 */
#ifndef NAVLINK_XFER_TX_H
#define NAVLINK_XFER_TX_H

#include <stddef.h>
#include <stdint.h>

#include "comm/xfer/fs_query.h"
#include "comm/xfer/navlink_xfer.h"

/* Emitter wired into xfer_init() in the xfer_service_task. */
extern const xfer_tx_ops_t g_xfer_tx_ops;

/* Emitter wired into fs_query_init() (filesystem navigation). */
extern const fs_query_tx_ops_t g_fs_query_tx_ops;

/* Pure builders (encode into `frame`, return its length). No channel write — the
 * ops wrap these with write_channel(); the test decodes them. `frame` must hold
 * NAVLINK_MAX_FRAME bytes. */
size_t xfer_build_command_ack(uint8_t *frame, uint32_t acked_msgid,
                              uint8_t req_seq, uint8_t result, int32_t param2);
size_t xfer_build_info(uint8_t *frame, const xfer_session_t *s, uint8_t result,
                       uint16_t chunk_size, uint32_t total_size,
                       uint32_t mtime);
size_t xfer_build_data(uint8_t *frame, const xfer_session_t *s, uint8_t flags,
                       uint8_t len, uint32_t offset, const uint8_t *buf);
size_t xfer_build_ack(uint8_t *frame, const xfer_session_t *s, uint8_t flags,
                      uint8_t result, uint32_t next_offset);

#endif /* NAVLINK_XFER_TX_H */
