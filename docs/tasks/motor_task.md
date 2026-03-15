# Motor Task (`motor_task`)

**Source File**: `src/actuator/motor_task.c`
**Stack Size**: 2048
**Priority**: 0
**Loop Rate**: 400 Hz (2ms delay)

## Description

Translates RC input to motor outputs. It reads the raw throttle value from the shared `rc_channels` array (channel 3) and maps the 1000-2000 µs range to a 0.0-1.0 float. This throttle value is applied uniformly to the four electronic speed controllers (ESCs) via PWM on TIM1 (pins PA8-PA11).
