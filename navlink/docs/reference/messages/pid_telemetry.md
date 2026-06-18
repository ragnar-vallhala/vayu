# PID Configuration Telemetry

> Updated for NavLink v2. Authoritative wire spec: navlink/dialect.json + docs/analysis/navlink-v2-spec.md.

This document describes how the Ground Control Station (GCS) configures the Flight Controller (FC) PID and control parameters at runtime.

## Protocol

The bespoke 3-way `SET_PID` / `PID_ACK` / `GCS_ACK` handshake carried over the v1 `SYSTEM_STATUS` (`0x05`) packet has been **removed**. PID updates now use the standard NavLink v2 command path:

1. The GCS sends **`CMD_SET_PID` (msgid 8195)**. Each message carries one slot: `controller` (0=`ANGLE` / 1=`RATE`), `axis` (0=roll / 1=pitch / 2=yaw), and the gains. (See `pid_controller` / `pid_axis` enums in `../../navlink/dialect.json`.)
2. The FC applies the gains to its live control configuration and replies with **`COMMAND_ACK` (msgid 5)**, correlated by `req_seq`, with a `command_result` (`ACCEPTED` on success). No application-level re-confirmation from the GCS is required; reliability is provided by the command/ack correlation.

This path is implemented in `src/comm/navlink_router.c` (`on_cmd_set_pid`) and `src/control/pid_config.c`.

## Persistence (SD Card)

`CMD_SET_PID` applies the gains live and **persists** the updated control configuration to the onboard SD card. On boot the FC reads this configuration during control init; if the file is missing or corrupt it falls back to hardcoded safe defaults.

## Changelog

| Date    | Author | Description                                              |
| ------- | ------ | ------------------------------------------------------- |
| 06/2026 | —      | NavLink v2: CMD_SET_PID (8195) + COMMAND_ACK (5); drop handshake |
