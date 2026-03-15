# RC iBus Task (`rc_ibus_task`)

**Source File**: `src/comm/rc_task.c`
**Stack Size**: 2048
**Priority**: 0
**Loop Rate**: ~500 Hz (2ms delay)

## Description

Manages RC data ingestion over the iBus protocol. It utilizes UART1 DMA RX (at 115200 baud) to continuously read from the receiver and populates a shared 14-channel `rc_channels` array. A built-in circular buffer safely pulls data as it streams in from the DMA via NDTR register tracking.
