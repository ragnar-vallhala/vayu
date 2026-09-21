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
#ifndef VAYU_SIM_NAVHAL_PORT_INTERRUPT_H
#define VAYU_SIM_NAVHAL_PORT_INTERRUPT_H

#include "common/hal_status.h"

/* hal_irq_t would normally come from the cortex-m4 family/interrupt_reg.h
 * (IRQn_Type enum). For host SITL we don't fire real IRQs; just give the
 * type a definition so the shimmed hal_interrupt_* function signatures
 * type-check. */
typedef int hal_irq_t;

/* Interrupt-controller entry points the firmware calls (e.g. channel.c).
 * Implemented as host no-ops in host_navhal.c; declared here so host
 * translation units that include navhal.h see real prototypes instead of
 * implicit declarations. */
hal_status_t hal_interrupt_enable(hal_irq_t irq);
hal_status_t hal_interrupt_disable(hal_irq_t irq);
hal_status_t hal_interrupt_attach_callback(hal_irq_t irq, void (*cb)(void));
hal_status_t hal_interrupt_detach_callback(hal_irq_t irq);

#endif
