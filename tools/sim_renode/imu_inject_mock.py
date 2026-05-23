# -*- coding: utf-8 -*-
# Phase 5 Light: IMU sample injection peripheral.
#
# Sits at an otherwise-unused sysbus address and exposes a single
# bmx160_all_reading_t-shaped sample buffer. A vayu sim task (built
# under VAYU_SIM) reads the buffer every millisecond and pushes the
# sample through the firmware's normal imu_buffer / imu_queue_control /
# imu_queue_telemetry APIs - so the control loop, sensor fusion, and
# telemetry all see real-looking IMU data without us having to mock
# the full I2C + BMX160 register protocol (that's the "Heavy" Phase 5
# follow-up).
#
# The peripheral itself reads bmx160_all_reading_t-shaped bytes from
# the host FIFO /tmp/vayu_imu.fifo (Phase 4's Gazebo bridge writes to
# this). Until that bridge is online, the peripheral generates a
# benign default sample (level flight, gravity on Z, no rotation) so
# the firmware sees a sensible IMU rather than NaNs.
#
# Layout matches bmx160_all_reading_t (= union of converted + raw;
# converted is bigger so the union is sized to it):
#
#   offset  type        meaning                  bytes
#     0x00  float[3]    acc   (calibrated m/s2)    12
#     0x0C  float[3]    gyr   (calibrated dps)     12
#     0x18  float[3]    mag   (calibrated uT)      12
#     0x24  float[3]    acc_raw                    12
#     0x30  float[3]    gyr_raw                    12
#     0x3C  float[3]    mag_compensated            12
#     0x48  float       temp                        4
#                                          total = 76 B (size of union)

import errno
import os
import select
import struct

FIFO_PATH = "/tmp/vayu_imu.fifo"
O_NONBLOCK_LINUX = 0x800
SAMPLE_BYTES = 76    # size of bmx160_all_converted_reading_t


def _default_sample():
    """Stationary, level, no spin: acc=(0,0,9.81), gyr=(0,0,0), mag=(25,5,40),
    temp=25 C. Matches what the BMX160 would report on a calibrated bench."""
    return struct.pack(
        "<3f 3f 3f 3f 3f 3f f",
        0.0, 0.0, 9.81,         # acc (calibrated)
        0.0, 0.0, 0.0,          # gyr (calibrated)
        25.0, 5.0, 40.0,        # mag (calibrated)
        0.0, 0.0, 9.81,         # acc_raw
        0.0, 0.0, 0.0,          # gyr_raw
        25.0, 5.0, 40.0,        # mag_compensated
        25.0,                   # temp (deg C)
    )


if request.IsInit:
    sample_bytes = _default_sample()
    fifo_fd = -1
    try:
        os.system("mkfifo " + FIFO_PATH + " 2>/dev/null")
        # O_RDWR (not O_RDONLY) on the FIFO: IronPython 2.7's os.read
        # on a RDONLY FIFO with no writer blocks even with O_NONBLOCK
        # set, hanging Renode on every IsRead until the bridge starts.
        # Holding the read-end open RDWR makes the peripheral its own
        # writer too, so os.read always returns immediately (EAGAIN
        # when empty) and IsRead never blocks.
        fifo_fd = os.open(FIFO_PATH, os.O_RDWR | O_NONBLOCK_LINUX)
        self.InfoLog("imu-inject: FIFO at " + FIFO_PATH +
                     " (host writes 76-byte samples)")
    except Exception as exc:
        self.WarningLog("imu-inject: FIFO setup failed: " + str(exc))

elif request.IsRead:
    # Drain new samples from the FIFO before responding.
    #
    # Why select() and not just os.read on an O_NONBLOCK FIFO?
    # IronPython 2.7's os.read on a FIFO in Renode blocks even with
    # the descriptor opened O_NONBLOCK — the kernel flag is set but
    # the IronPython wrapper doesn't surface EAGAIN. Probing with
    # select(timeout=0) tells us readiness without ever entering the
    # blocking read path.
    if fifo_fd >= 0:
        try:
            r, _, _ = select.select([fifo_fd], [], [], 0)
        except Exception:
            r = []
        if r:
            try:
                data = os.read(fifo_fd, 256 * SAMPLE_BYTES)
            except Exception as e:
                data = ""
                self.WarningLog("imu-inject: FIFO read err: " + str(e))
            if data:
                n_full = len(data) // SAMPLE_BYTES
                if n_full > 0:
                    start = (n_full - 1) * SAMPLE_BYTES
                    sample_bytes = data[start:start + SAMPLE_BYTES]

    off = request.Offset
    if 0 <= off < SAMPLE_BYTES - 3:
        b0 = ord(sample_bytes[off])
        b1 = ord(sample_bytes[off + 1])
        b2 = ord(sample_bytes[off + 2])
        b3 = ord(sample_bytes[off + 3])
        request.Value = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    else:
        request.Value = 0

else:    # write - ignore (vayu shouldn't write here)
    pass
