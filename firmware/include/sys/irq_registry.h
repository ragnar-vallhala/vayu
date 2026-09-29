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
/* Who owns which interrupt vector.
 *
 * An interrupt vector holds exactly one callback, so a second claim silently
 * evicts the first. Nothing listed the claims, so the collisions were found by
 * their symptoms instead (fault line F12) -- the one that is written down cost
 * real debugging:
 *
 *   USART6_TX and the SDIO write DMA both map to DMA2 Stream6. SDIO re-grabbed
 *   the Stream6 completion IRQ on every SD block write, which stranded the
 *   telemetry channel at busy=1, so telemetry died after the first card write
 *   -- saving a calibration was enough to do it. The fix was to move USART6_TX
 *   to its alternate Stream7 mapping. See comm/channel.c.
 *
 * Claiming through this registry turns that class of bug into a log line at
 * init instead of a peripheral that quietly stops months later.
 *
 * The vector is a plain uint32_t, not hal_irq_t, so that naming a vector does
 * not drag navhal.h into every caller. Vector NUMBERS are board facts and live
 * in board/<name>/vayu_board.h.
 *
 * KNOWN GAP: vaios claims vectors too -- its console DMA takes
 * DMA1_Stream6_IRQn and its console RX takes USART2_IRQn (portable/cortex-m4/
 * port_hw.c), both behind its own build flags. vaios is a pristine submodule,
 * and mirroring another repo's config macros here to pre-register them is the
 * duplication that F13 was about. So this registry sees vayu's claims only;
 * a vayu-vs-vaios collision still has to be reasoned about by hand.
 */
#ifndef VAYU_IRQ_REGISTRY_H
#define VAYU_IRQ_REGISTRY_H

#include <stdbool.h>
#include <stdint.h>

/** How many distinct vectors the firmware may own at once. */
#define IRQ_REGISTRY_MAX 12

/**
 * Record `owner` as the holder of `vector`.
 *
 * @return true if the claim stands. False if another owner already holds it --
 *         the caller has evicted them, and the conflict is logged. Re-claiming
 *         with the same owner name succeeds and changes nothing.
 *
 * Call it immediately BEFORE attaching the callback, so the log names the
 * vector that is about to be taken over.
 */
bool irq_registry_claim(uint32_t vector, const char *owner);

/** Give a vector up. A release by a non-owner is ignored. */
void irq_registry_release(uint32_t vector, const char *owner);

/** Current owner of a vector, or NULL if unclaimed. */
const char *irq_registry_owner(uint32_t vector);

/** Log the whole table. For bring-up and for a bug report. */
void irq_registry_dump(void);

#endif // VAYU_IRQ_REGISTRY_H
