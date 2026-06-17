"""Phase 2/6 integration: the `vayu-headless run` one-shot path via cli.main."""
import pytest

from vayu_headless import cli


@pytest.mark.integration
def test_cli_run_oneshot(require_binaries, gcs_conf, monkeypatch):
    monkeypatch.setenv("VAYU_GCS_CONF", gcs_conf or "/no/conf")
    rc = cli.main(["run", "--box", "6", "--alt", "-5", "--secs", "8"])
    assert rc == 0
