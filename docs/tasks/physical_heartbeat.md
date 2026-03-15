# Physical Heartbeat (`physical_heartbeat`)

**Source File**: `src/comm/physical_heartbeat.c`
**Stack Size**: 1024
**Priority**: 0
**Loop Rate**: 1 Hz (500ms High / 500ms Low cycles)

## Description

A visual logic cue that blinks the core heartbeat LED at 1Hz (`_HEARTBEAT_LED_PIN`). At the completion of each pulse sequence, it actively emits a `PACKET_TYPE_HEARTBEAT` sync packet via UART2 towards the GCS logic.
