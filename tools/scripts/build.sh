#!/usr/bin/env bash
# Firmware build (shim). Equivalent to: vayu.sh build firmware.
# Kept as a stable entry point; the dispatcher does the work.
exec "$(dirname "${BASH_SOURCE[0]}")/vayu.sh" build firmware "$@"
