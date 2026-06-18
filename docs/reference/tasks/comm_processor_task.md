# Communications Processor (`comm_processor_task`)

**Source File**: `src/comm/comm_processor.c`
**Stack Size**: 1024
**Priority**: 0
**Loop Rate**: ~100 Hz (10ms delay when idle)

## Description

Designed to intercept rx packets from the drone's active serial channels and dispatch v2 RX packets (commands and `PACKET_TYPE_TIME_SYNC`). Clock alignment now happens via the `PACKET_TYPE_TIME_SYNC` handler (an NTP-style request/response handshake that disciplines the local clock). A received `PACKET_TYPE_HEARTBEAT` is liveness-only and merely records the GCS `device_id`; it no longer jams the local timestamp.
