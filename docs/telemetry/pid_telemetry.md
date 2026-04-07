# PID Configuration Telemetry Protocol

This document outlines the telemetry handshake protocol used to dynamically configure the Flight Controller (FC) PID and control parameters from the Ground Control Station (GCS).

## Architecture

To ensure high-reliability parameter updates over potentially lossy telemetry links, the PID update mechanism utilizes a **3-Way Handshake** confirmation protocol encapsulated within the `SYSTEM_STATUS` (`0x05`) packet type.

All PID tuning commands and acknowledgments use a designated `PID_ORIGIN` sub-type to distinguish them from standard status updates or error strings.

### 3-Way Handshake Sequence

1. **GCS `SET_PID` Command (GCS -> FC)**
   The GCS transmits a `SYSTEM_STATUS` packet containing the new `control_config_t` values (or a targeted subset) marked with the `SET_PID` command flag.

2. **FC `PID_ACK` Response (FC -> GCS)**
   Upon receiving the `SET_PID` command, the FC applies the changes to its active control loop configuration. It immediately responds with a `PID_ACK` packet.
   - **Payload**: The `PID_ACK` packet must contain the _newly applied_ PID values.
   - **GCS Role**: The GCS parses the `PID_ACK` payload and compares the returned values against its requested state to visually confirm the parameters were safely adopted by the FC.

3. **GCS Final `GCS_ACK` (GCS -> FC)**
   Once the GCS verifies the configuration was successfully applied, it sends a final `GCS_ACK` packet back to the FC.
   - **FC Role**: The FC expects this final confirmation. If the FC does not receive the `GCS_ACK` within a predefined timeout window, it assumes the `PID_ACK` was lost in transit and will **re-transmit** the `PID_ACK` packet.

## Persistence (SD Card)

To ensure PID tunings are permanently retained:

- Upon successfully completing the 3-Way Handshake (the FC receives `GCS_ACK`), the FC will serialize the updated `control_config_t` structure and save it to the onboard SD card.
- During system boot, the FC will read this configuration file from the SD card and apply these PID values during `control_init()`. If the file is not found or corrupted, it will fall back to hardcoded safe defaults.

### Data Encapsulation

All related packets sit under the global telemetry protocol:

- **Primary Type**: `PACKET_TYPE_SYSTEM_STATUS`
- **Payload Structure**:
  - `[0]` - Status Message Type Enum (e.g. `STATUS_TYPE_PID_UPDATE`)
  - `[1]` - PID Command Phase (`SET_PID`, `PID_ACK`, `GCS_ACK`)
  - `[2..N]` - The targeted serialized float values (kp, ki, kd, etc.)

### Timeout & Retry Mechanism

- The FC will buffer the `PID_ACK` state until `GCS_ACK` is intercepted.
- Retries happen on a fixed generic interval (e.g. 500ms) up to a maximum threshold. If the threshold is reached without GCS acknowledgement, the FC gracefully drops the retry loop (the parameters remain active on the device).
