#ifndef VAYU_SIM_NAVHAL_PORT_UART_H
#define VAYU_SIM_NAVHAL_PORT_UART_H

#include "common/hal_status.h"
#include "utils/uart_types.h"
#include <stdint.h>

/* Host SITL stub. The DMA-write entry point the firmware calls (e.g. channel.c)
 * is implemented synchronously in host_navhal.c; declared here so host
 * translation units see a real prototype instead of an implicit declaration. */
hal_status_t hal_uart_write_dma(hal_uart_t uart, const uint8_t *buf,
                                uint16_t len);

#endif
