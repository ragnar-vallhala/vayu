"""Unit tests for path/binary resolution."""
import os

from vayu_headless import paths


def test_env_override_wins(monkeypatch):
    monkeypatch.setenv("VAYU_SITL_RTOS_BIN", "/custom/vayu_sitl_rtos")
    monkeypatch.setenv("VSIM_WORLDMESH_BIN", "/custom/wm")
    monkeypatch.setenv("VAYU_GCS_CONF", "/custom/x.conf")
    assert paths.rtos_bin() == "/custom/vayu_sitl_rtos"
    assert paths.worldmesh_bin() == "/custom/wm"
    assert paths.gcs_conf_default() == "/custom/x.conf"


def test_default_paths_are_absolute_under_repo(monkeypatch):
    monkeypatch.delenv("VAYU_SITL_RTOS_BIN", raising=False)
    assert os.path.isabs(paths.rtos_bin())
    assert paths.rtos_bin().endswith("sim/host/build_sitl_rtos/vayu_sitl_rtos")


def test_suffix_isolation():
    p = paths.fifo_paths("_lab42")
    assert p["pose"] == "/tmp/vsim_pose_lab42"
    assert set(p) == {"pose", "ctl"}
    assert paths.uart_advert("_lab42") == "/tmp/vayu_uart2_pty_lab42"
    assert paths.world_mesh_bin("_lab42") == "/tmp/vsim_world_lab42.bin"


def test_gcs_singletons_fixed():
    assert paths.GCS_POSE == "/tmp/vsim_pose"
    assert paths.GCS_ADVERT == "/tmp/vayu_uart2_pty"
    assert paths.SOCK_PATH == "/tmp/sitl_lab.sock"
