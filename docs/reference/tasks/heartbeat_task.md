# Heartbeat Task (`heartbeat_task`)

**Source File**: `src/sys/heartbeat.c`
**Stack Size**: 1024
**Priority**: 0
**Loop Rate**: 1 Hz (500ms High / 500ms Low cycles)

## Description

A visual/audible liveness cue that blinks the core heartbeat LED at 1Hz (`_HEARTBEAT_LED_PIN`) and drives the buzzer. It is LED/buzzer-only and does not emit any packet over UART; heartbeat telemetry is now produced by the telemetry task.
