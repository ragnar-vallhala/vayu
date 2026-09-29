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
 * @file driver/uart.h
 * @brief The serial links, and who owns their interrupts.
 *
 * @details
 * Two things live behind one link and they are easy to get wrong separately:
 * the peripheral, and the two interrupt vectors that make it useful (RX, and
 * TX-DMA completion). `comm/` used to drive all of it directly -- init the
 * peripheral, pick the vector with a ternary over UART instances, attach the
 * callback, enable the line. That is a driver's work wearing the transport's
 * name, and it meant the comm layer knew which DMA stream belonged to which
 * UART on this part.
 *
 * A link is identified by `vuart_t`, whose values come from the board
 * (`BOARD_TELEMETRY_UART`, `BOARD_RC_UART`). The IRQ vectors come from the
 * board too and are passed in, because which vector serves which peripheral is
 * a package fact -- this driver does not guess it from the instance.
 *
 * Every vector claim goes through sys/irq_registry.h, so a second owner is a
 * log line at init rather than a peripheral that stops working later (fault
 * line F12).
 *
 * NOT a buffering layer. There is no ring and no queue here: `comm/channel.c`
 * owns the ping-pong TX buffers and the flow control, because how much to
 * buffer and when to drop is a transport policy, not a UART's.
 */
#ifndef VAYU_DRIVER_UART_H
#define VAYU_DRIVER_UART_H

#include "vayu_status.h"
#include <stdint.h>

/** A serial link. Values are the board's (BOARD_TELEMETRY_UART, BOARD_RC_UART);
 *  0 is "no link", which is how channel.c marks a free slot. */
typedef uint32_t vuart_t;

/** Bring the peripheral up at `baud`. Does not touch interrupts. */
vayu_status_t vuart_init(vuart_t u, uint32_t baud);

/**
 * Route this link's receive interrupt to `cb` and enable RX.
 *
 * @param vector the board's RX vector for this link
 * @param owner  a string literal naming the claimant, for the IRQ registry
 */
vayu_status_t vuart_attach_rx(vuart_t u, uint32_t vector, void (*cb)(void),
                              const char *owner);

/**
 * Silence and release this link's receive interrupt.
 *
 * @return VAYU_ERR_FAULT if the vector could not be masked -- the caller must
 *         treat the link as still live, because detaching the callback after a
 *         failed mask leaves an enabled vector pointing at nothing.
 */
vayu_status_t vuart_detach_rx(vuart_t u, uint32_t vector, const char *owner);

/**
 * Route a TX-DMA completion vector to `cb` and enable it.
 *
 * Takes the vector rather than the link because a UART's TX DMA stream is not
 * derivable from the UART: on this part the telemetry link's default stream is
 * shared with SDIO, so the board picks the alternate one. See the note in
 * sys/irq_registry.h for what that cost before anyone wrote it down.
 */
vayu_status_t vuart_attach_tx_dma(uint32_t vector, void (*cb)(void),
                                  const char *owner);

/** Hand `n` bytes to the TX DMA. The buffer must outlive the transfer. */
vayu_status_t vuart_write_dma(vuart_t u, const uint8_t *data, uint16_t n);

/** Blocking single-byte write; the fallback when DMA is unavailable. */
void vuart_write_char(vuart_t u, char c);

/** Blocking single-byte read. */
char vuart_read_char(vuart_t u);

/** Start a circular DMA receive into `buf`, with `on_idle` on line idle. */
vayu_status_t vuart_rx_dma_start(vuart_t u, uint8_t *buf, uint16_t n,
                                 void (*on_idle)(void));

/** How far the RX DMA has written into that buffer. */
vayu_status_t vuart_rx_dma_index(vuart_t u, uint16_t *out_index);

#endif // VAYU_DRIVER_UART_H
