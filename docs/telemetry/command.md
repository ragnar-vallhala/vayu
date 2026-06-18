# Command Messages [GCS → FC]

> Updated for NavLink v2. Authoritative wire spec: navlink/dialect.json + docs/analysis/navlink-v2-spec.md.

Commands are sent **GCS → FC** to trigger an action or update a parameter. The v1 single `0x3` packet with a `cmd_id`/`argc`/`args[]` payload has been retired. In NavLink v2 each command is its **own message** with a typed payload, and every command is acknowledged by **`COMMAND_ACK` (msgid 5)** correlated by `req_seq`. The ack `result` uses the `command_result` enum (`ACCEPTED`, `TEMPORARILY_REJECTED`, `DENIED`, `UNSUPPORTED`, `FAILED`, `IN_PROGRESS`).

---

## Command Set

| msgid  | Message                  | Description                                              |
| -----: | ------------------------ | ------------------------------------------------------- |
| `8192` | `CMD_ARM`                | Arm the vehicle.                                        |
| `8193` | `CMD_DISARM`             | Disarm the vehicle.                                     |
| `8194` | `CMD_CALIBRATE_IMU`      | Trigger / cancel IMU calibration (see below).          |
| `8195` | `CMD_SET_PID`            | Set PID gains for one (controller, axis) slot.         |
| `8196` | `CMD_SET_GYRO_LPF`       | Set gyro low-pass-filter cutoff per axis.              |
| `8197` | `CMD_SET_MOTOR_GEOMETRY` | Set motor mixer geometry.                              |
| `8198` | `CMD_SET_FLIGHT_MODE`    | Select flight mode (`ANGLE`/`ACRO`/`RELEASE_TO_RC`).   |

For the exact payload fields of each command, see `../../navlink/dialect.json` and `../analysis/navlink-v2-spec.md`.

---

## CMD_CALIBRATE_IMU (8194)

Manages the onboard sensor calibration procedures. The message selects the target sensor (accelerometer / gyroscope / magnetometer) and the calibration type (bias-only vs full; hard-iron / soft-iron / both for the magnetometer), or cancels an active calibration and returns the system to `STANDBY`. See the dialect for the exact field/enum encoding; live progress is reported back via `CALIBRATION_STATUS` (12320) — see [System Status](system_status.md).

> [!IMPORTANT]
> The drone must remain stationary and level on a flat surface during calibration. Unexpected motion may produce invalid bias calculations.

> Unrecognised or malformed commands are rejected; the FC reports the outcome via `COMMAND_ACK`.

## Changelog

| Date    | Author | Description                                  |
| ------- | ------ | -------------------------------------------- |
| 06/2026 | —      | NavLink v2 command set (8192–8198) + COMMAND_ACK |
