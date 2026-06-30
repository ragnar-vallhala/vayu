# RC iBus Task (`rc_ibus_task`)

**Source File**: `src/comm/rc_task.c`
**Stack Size**: 1024
**Priority**: 0
**Loop Rate**: event-driven (woken by the UART idle-line interrupt, ~one wake per iBus frame ≈ 140 Hz), with a bounded fallback timeout so the RC-loss watchdog keeps ticking when frames stop.

## Description

Manages RC data ingestion over the iBus protocol on **USART2** (PA2/PA3, 115200 baud), where the receiver is wired; telemetry owns USART6. The UART runs circular DMA RX into a 128-byte ring. The task blocks on a semaphore signalled from the UART idle-line ISR (`hal_uart_attach_idle_callback`) — i.e. it wakes at the inter-frame gap rather than polling — then drains the ring up to the current DMA write index obtained via `hal_uart_dma_rx_index()` (no direct DMA-register access). The per-frame arm/state-machine and channel deadband run once per completed iBus frame, and a valid 14-channel frame is pushed to the control and telemetry queues.
