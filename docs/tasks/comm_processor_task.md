# Communications Processor (`comm_processor_task`)

**Source File**: `src/comm/comm_processor.c`
**Stack Size**: 1024
**Priority**: 0
**Loop Rate**: ~100 Hz (10ms delay when idle)

## Description

Designed to intercept rx packets from the drone's active serial channels. Calls `get_next_rx_packet(...)` and examines standard command payloads. For example, if it receives a `PACKET_TYPE_HEARTBEAT` from the GCS, it synchronizes the local `timestamp` and `device_id`.
