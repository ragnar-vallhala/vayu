"""Binary, FIFO/pty, and singleton-path resolution for the SDK.

One place for every filesystem literal the harness used to hard-code, with env
overrides. Per-session FIFOs are isolated by a private suffix (so many sessions
coexist); the GCS-facing singletons (pose fan-out, UART2 advert, control
socket) are fixed because the GCS connects to them by a well-known name.
"""
import os

from ._repo import repo_root

# GCS-facing singletons (well-known names the Navigator connects to).
GCS_ADVERT = "/tmp/vayu_uart2_pty"     # SimToolbar auto-offers this as "SITL UART2"
GCS_POSE = "/tmp/vsim_pose"            # SimWorker "Attach Ext" reads this
SOCK_PATH = "/tmp/sitl_lab.sock"       # serve/do control socket


def rtos_bin():
    """The single in-process SITL binary (firmware + physics in one process,
    driven over the same pose/ctl FIFOs + UART2/RC ptys the old vsim_d+vayu_sitl
    pair used). Run it with VAYU_RTOS_SCENARIO=driver."""
    env = os.environ.get("VAYU_SITL_RTOS_BIN")
    if env:
        return env
    return os.path.join(repo_root(), "sim", "host", "build_sitl_rtos",
                        "vayu_sitl_rtos")


def worldmesh_bin():
    env = os.environ.get("VSIM_WORLDMESH_BIN")
    if env:
        return env
    root = repo_root()
    candidates = [
        os.path.join(root, "navigator", "headless-sdk", "cpp", "worldmesh",
                     "build", "vsim_worldmesh"),
        os.path.join(root, "sim", "host", "worldmesh", "build",
                     "vsim_worldmesh"),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return candidates[0]


def gcs_conf_default():
    env = os.environ.get("VAYU_GCS_CONF")
    if env:
        return env
    return os.path.expanduser("~/.config/Vayu/Vayu GCS.conf")


def fifo_paths(suffix):
    # The driver publishes exactly two FIFOs: ground-truth pose (out) and the
    # config channel (in). The old pwm/imu/baro FIFOs were the firmware<->physics
    # boundary — that hop is now in-process, so they no longer exist.
    return {n: f"/tmp/vsim_{n}{suffix}" for n in ("pose", "ctl")}


def uart_advert(suffix):
    return f"/tmp/vayu_uart2_pty{suffix}"


def world_mesh_bin(suffix):
    return f"/tmp/vsim_world{suffix}.bin"
