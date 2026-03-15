# State Machines

This directory contains documentation for the various state machines and status bitmasks used throughout the Vayu firmware.

## List of State Machines

- [System State](system_state.md): High-level operational states of the drone.
- [Boot Check](boot_check.md): Bitmask and sequence for system initialization checks.
- [IMU Health Check](imu_health.md): Detailed status bitmask for onboard sensors.
- [LED & Buzzer Patterns](led_buzzer.md): Guide to visual and audible status signals.
- [Packet Deserializer](deserializer_state.md): State machine for parsing incoming binary telemetry.
- [IBUS Protocol](ibus_state.md): State machine for decoding RC receiver data.
