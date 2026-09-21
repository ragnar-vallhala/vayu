"""Wire/transport layers for the headless SDK.

- `vsim`: sim control + pose framing (mirrors sim/host/sdk/vsim_proto.h).
- `navlink`: FC telemetry decode (wraps the generated NavLink codec).
- `rc`: RC channel encoding for the firmware host's RC feeder.
"""
