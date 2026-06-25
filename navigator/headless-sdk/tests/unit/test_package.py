"""Phase 0 smoke: the package imports and exposes a version."""
import vayu_headless


def test_version_present():
    assert isinstance(vayu_headless.__version__, str)
    assert vayu_headless.__version__.count(".") >= 1
