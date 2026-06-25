"""Shared test fixtures for the headless SDK.

Locates the repo root, the SITL binaries, and the GCS config, and skips
integration tests cleanly when a binary is missing (so unit tests still run on
a machine that hasn't built the firmware host).
"""
import os
import sys

import pytest

# repo root = .../vayu (this file is software/headless-sdk/tests/conftest.py)
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))

VSIM_BIN = os.environ.get("VSIM_BIN_PATH",
                          os.path.join(ROOT, "sim/vsim/build/vsim_d"))
SITL_BIN = os.environ.get("VAYU_SITL_BIN",
                          os.path.join(ROOT, "sim/host/build_sitl/vayu_sitl"))
GCS_CONF = os.environ.get("VAYU_GCS_CONF",
                          os.path.expanduser("~/.config/Vayu/Vayu GCS.conf"))


@pytest.fixture(scope="session")
def repo_root():
    return ROOT


@pytest.fixture(scope="session")
def gcs_conf():
    return GCS_CONF if os.path.exists(GCS_CONF) else None


@pytest.fixture
def require_binaries():
    """Skip an integration test unless both SITL binaries exist."""
    for b in (VSIM_BIN, SITL_BIN):
        if not os.path.exists(b):
            pytest.skip(f"missing SITL binary: {b} (build it to run integration tests)")
    os.environ["VSIM_BIN_PATH"] = VSIM_BIN
    os.environ["VAYU_SITL_BIN"] = SITL_BIN
    return (VSIM_BIN, SITL_BIN)
