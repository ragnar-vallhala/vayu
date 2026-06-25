"""Phase 4/6 integration: the GCS-facing bridges (pose fan-out + UART2 advert).

Covers the gcs=True path: the harness re-broadcasts pose to /tmp/vsim_pose for
'Attach Ext' and advertises a UART2 bridge pty. We act as the GCS reader and
confirm the fan-out delivers decodable pose frames.
"""
import os
import time

import pytest

from vayu_headless import Pilot, SitlSession
from vayu_headless import paths
from vayu_headless.transport import vsim


@pytest.mark.integration
def test_gcs_pose_fanout_and_advert(require_binaries, gcs_conf):
    with SitlSession(gcs=True, conf=gcs_conf) as sess:
        assert os.path.exists(paths.GCS_POSE), "pose fan-out FIFO not created"
        assert sess.gcs_path, "UART2 bridge pty not advertised"
        assert os.path.exists(paths.GCS_ADVERT)

        pilot = Pilot(sess, alt=-5.0)
        pilot.arm_takeoff(alt=-5.0)
        time.sleep(2.0)

        # Act as the GCS 'Attach Ext' reader on the fan-out FIFO.
        fd = os.open(paths.GCS_POSE, os.O_RDONLY | os.O_NONBLOCK)
        buf = bytearray()
        truth = None
        t0 = time.time()
        try:
            while time.time() - t0 < 3.0 and truth is None:
                try:
                    d = os.read(fd, 65536)
                except BlockingIOError:
                    d = b""
                if d:
                    buf += d
                    truth = vsim.parse_pose(buf)
                else:
                    time.sleep(0.02)
        finally:
            os.close(fd)
        assert truth is not None, "fan-out delivered no decodable pose frame"
        assert -truth["pos"][2] > 1.0, "craft should be airborne in the relayed pose"
        pilot.land()
    # close() must remove the singleton it created
    assert not os.path.exists(paths.GCS_POSE)
