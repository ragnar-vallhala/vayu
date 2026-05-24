#!/usr/bin/env python3
"""
gz_imu_to_vayu.py - Gazebo IMU -> host SITL bridge.

Subscribes to the Gazebo IMU + magnetometer topics, packs each sample
into the bmx160_all_converted_reading_t layout vayu expects, and writes
it to /tmp/vayu_imu.fifo. The host SITL binary reads the FIFO and pushes
each sample through imu_buffer / imu_queue_control / imu_queue_telemetry
so vayu's sensor fusion and control loops see Gazebo's IMU.

Why subprocess and not gz.transport13 bindings? On this host the
Python subscribe() returned True but the discovery service never
registered the subscriber, so callbacks never fired (gz topic -i
reported "No subscribers"). The `gz topic -e` CLI works reliably; we
spawn one per topic and parse the streaming text-proto output.

Closed-loop usage:
    Terminal 1:  gz sim -s -r --headless-rendering tools/sim_gazebo/worlds/vayu_quad.sdf
    Terminal 2:  ./build_sitl/vayu_sitl
    Terminal 3:  python3 tools/sim_gazebo/gz_imu_to_vayu.py

bmx160_all_converted_reading_t layout (76 B, little-endian):
    float[3] acc             m/s^2 (calibrated)
    float[3] gyr             dps   (calibrated)
    float[3] mag             uT    (calibrated)
    float[3] acc_raw         m/s^2 (uncalibrated)
    float[3] gyr_raw         dps   (uncalibrated)
    float[3] mag_compensated uT    (compensated)
    float    temp            deg C
"""
from __future__ import annotations

import argparse
import math
import os
import re
import struct
import subprocess
import sys
import threading
import time

RAD_TO_DEG = 180.0 / math.pi
TESLA_TO_UT = 1.0e6

DEFAULT_WORLD = "vayu_quad_world"
DEFAULT_MODEL = "X3"
DEFAULT_FIFO  = "/tmp/vayu_imu.fifo"
DEFAULT_RATE  = 200   # Hz

# Pattern: "  x: <float>" inside a "{}" block we tracked by indent.
_RE_FIELD = re.compile(r"^\s+([xyzw]):\s*(-?[\d.eE+-]+)")


def imu_topic(world: str, model: str) -> str:
    return ("/world/{}/model/{}/link/base_link/sensor/"
            "imu_sensor/imu").format(world, model)


def mag_topic(world: str, model: str) -> str:
    return ("/world/{}/model/{}/link/base_link/sensor/"
            "magnetometer/magnetometer").format(world, model)


class LatestState:
    """One-shot last-write-wins store of the most recent IMU + mag."""

    __slots__ = ("_lock", "acc", "gyr_dps", "mag_uT", "temp",
                 "imu_count", "mag_count")

    def __init__(self):
        self._lock = threading.Lock()
        self.acc     = (0.0, 0.0, 9.81)
        self.gyr_dps = (0.0, 0.0, 0.0)
        self.mag_uT  = (25.0, 5.0, 40.0)
        self.temp    = 25.0
        self.imu_count = 0
        self.mag_count = 0

    def set_imu(self, ang_vel, lin_acc):
        with self._lock:
            self.acc = lin_acc
            self.gyr_dps = tuple(v * RAD_TO_DEG for v in ang_vel)
            self.imu_count += 1

    def set_mag(self, field_tesla):
        with self._lock:
            self.mag_uT = tuple(f * TESLA_TO_UT for f in field_tesla)
            self.mag_count += 1

    def snapshot(self):
        with self._lock:
            return self.acc, self.gyr_dps, self.mag_uT, self.temp


def parse_imu_stream(topic: str, state: LatestState) -> None:
    """Run `gz topic -e -t <topic>` and parse text-proto output for IMU.

    Each message arrives as a multi-line text-proto block. We track the
    current top-level field name (angular_velocity / linear_acceleration)
    and accumulate xyz floats inside it."""
    proc = subprocess.Popen(
        ["gz", "topic", "-e", "-t", topic],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

    current_block = None
    ang = [0.0, 0.0, 0.0]
    acc = [0.0, 0.0, 0.0]
    have_ang = have_acc = False

    for line in proc.stdout:
        s = line.rstrip("\n")
        if s.startswith("angular_velocity {"):
            current_block = "ang"
            continue
        if s.startswith("linear_acceleration {"):
            current_block = "acc"
            continue
        if s.startswith("orientation {") or s.startswith("orientation_covariance"):
            current_block = "skip"
            continue
        if s.startswith("angular_velocity_covariance") or s.startswith("linear_acceleration_covariance"):
            current_block = "skip"
            continue
        if s == "}":
            current_block = None
            continue

        m = _RE_FIELD.match(s)
        if m and current_block in ("ang", "acc"):
            axis, val = m.group(1), float(m.group(2))
            idx = {"x": 0, "y": 1, "z": 2}.get(axis)
            if idx is None:
                continue
            if current_block == "ang":
                ang[idx] = val
                have_ang = have_ang or idx == 2
            else:
                acc[idx] = val
                have_acc = have_acc or idx == 2

        # Heuristic: when we've seen both ang.z and acc.z this iteration,
        # the message is complete enough to push.
        if have_ang and have_acc:
            state.set_imu(tuple(ang), tuple(acc))
            have_ang = have_acc = False


def parse_mag_stream(topic: str, state: LatestState) -> None:
    """Run `gz topic -e -t <topic>` and parse Magnetometer field_tesla."""
    proc = subprocess.Popen(
        ["gz", "topic", "-e", "-t", topic],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

    in_field = False
    field = [0.0, 0.0, 0.0]
    seen_z = False

    for line in proc.stdout:
        s = line.rstrip("\n")
        if s.startswith("field_tesla {"):
            in_field = True
            continue
        if s == "}":
            if in_field and seen_z:
                state.set_mag(tuple(field))
                seen_z = False
            in_field = False
            continue

        if in_field:
            m = _RE_FIELD.match(s)
            if m:
                axis, val = m.group(1), float(m.group(2))
                idx = {"x": 0, "y": 1, "z": 2}.get(axis)
                if idx is not None:
                    field[idx] = val
                    if idx == 2:
                        seen_z = True


def pack_frame(acc, gyr, mag, temp) -> bytes:
    return struct.pack(
        "<3f 3f 3f 3f 3f 3f f",
        acc[0], acc[1], acc[2],
        gyr[0], gyr[1], gyr[2],
        mag[0], mag[1], mag[2],
        acc[0], acc[1], acc[2],
        gyr[0], gyr[1], gyr[2],
        mag[0], mag[1], mag[2],
        temp,
    )


def run(fifo_path: str, world: str, model: str, rate_hz: float,
        verbose: bool) -> int:
    waited = 0.0
    while not os.path.exists(fifo_path) and waited < 30.0:
        time.sleep(0.5)
        waited += 0.5
    if not os.path.exists(fifo_path):
        print("ERR: {} did not appear within 30 s. Is the SITL "
              "binary running?".format(fifo_path), file=sys.stderr)
        return 2

    state = LatestState()

    threading.Thread(target=parse_imu_stream,
                     args=(imu_topic(world, model), state),
                     daemon=True).start()
    threading.Thread(target=parse_mag_stream,
                     args=(mag_topic(world, model), state),
                     daemon=True).start()
    print("subscribed (via gz topic -e):")
    print("  ", imu_topic(world, model))
    print("  ", mag_topic(world, model))
    print("writing", fifo_path, "at", rate_hz, "Hz", flush=True)

    fd = os.open(fifo_path, os.O_WRONLY)

    period = 1.0 / rate_hz
    next_send = time.monotonic()
    last_log = next_send
    try:
        while True:
            acc, gyr, mag, temp = state.snapshot()
            try:
                os.write(fd, pack_frame(acc, gyr, mag, temp))
            except BrokenPipeError:
                os.close(fd)
                fd = os.open(fifo_path, os.O_WRONLY)
                continue

            now = time.monotonic()
            if verbose and now - last_log >= 1.0:
                last_log = now
                print("imu_msgs={} mag_msgs={}  acc=({: .2f}{: .2f}{: .2f})"
                      "  gyr_dps=({: .1f}{: .1f}{: .1f})"
                      .format(state.imu_count, state.mag_count,
                              acc[0], acc[1], acc[2],
                              gyr[0], gyr[1], gyr[2]),
                      flush=True)

            next_send += period
            sleep_for = next_send - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_send = time.monotonic()
    finally:
        try:
            os.close(fd)
        except OSError:
            pass
    return 0


def main(argv) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fifo",  default=DEFAULT_FIFO)
    ap.add_argument("--world", default=DEFAULT_WORLD)
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--rate",  type=float, default=DEFAULT_RATE)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    try:
        return run(args.fifo, args.world, args.model, args.rate,
                   verbose=not args.quiet)
    except KeyboardInterrupt:
        print("\ngz_imu_to_vayu: stopped", file=sys.stderr)
        return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


# ---------------------------------------------------------------------
# Phase 4b sketch (motor PWM out -> Gazebo)
# ---------------------------------------------------------------------
# See tools/sim_gazebo/vayu_pwm_to_gz.py.
