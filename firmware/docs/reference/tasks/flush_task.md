# Channel Flush Task (`flush_task`)

**Source File**: `src/comm/channel.c`
**Stack Size**: 1024
**Priority**: 0
**Loop Rate**: 100 Hz (10ms delay)

## Description

Iterates through the active `channel_handlers` linked list (including both serial loops and other forms of IO) and periodically invokes individual IO hardware pipeline flushes (`flush_channel`). For serial interfaces, this seamlessly shifts buffered ring buffers into DMA transmission engines while preventing synchronous write stalling elsewhere in the RTOS pipeline.
