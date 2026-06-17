"""Unit tests for RC channel encoding."""
from vayu_headless.transport import rc


def test_stick_centre_and_extremes():
    assert rc.stick_us(0.0) == 1500
    assert rc.stick_us(1.0) == 2000
    assert rc.stick_us(-1.0) == 1000
    assert rc.stick_us(2.0) == 2000     # clamped
    assert rc.stick_us(-9.0) == 1000    # clamped


def test_throttle_range():
    assert rc.throttle_us(0.0) == 1000
    assert rc.throttle_us(1.0) == 2000
    assert rc.throttle_us(0.5) == 1500
    assert rc.throttle_us(-1.0) == 1000  # clamped
    assert rc.throttle_us(5.0) == 2000   # clamped


def test_rc_csv():
    assert rc.rc_csv([1500, 1500, 1000, 1500, 1000, 1500]) == \
        "1500,1500,1000,1500,1000,1500\n"
