#!/usr/bin/env python3
"""
vayu_pwm_to_gz.py - Phase 4b (motor PWM out -> Gazebo).

Reads motor duty cycles from /tmp/vayu_pwm.fifo (one "motor_idx duty\\n"
line per CCRx write, emitted by tools/sim_renode/pwm_extract_mock.py),
maps duty -> rotor velocity, and publishes gz.msgs.Actuators on
/X3/gazebo/command/motor_speed at 200 Hz.

Co-sim layout:
    Renode TIM1 register-write @ 0x40010034..0x40010040 (CCR1..CCR4)
      -> pwm_extract_mock.py computes duty = CCR / (ARR + 1)
      -> "idx duty\\n" line into /tmp/vayu_pwm.fifo
      -> this script picks it up and publishes Actuators
      -> Gazebo MulticopterMotorModel spins rotor_N at velocity

rotor topology (per src/actuator/motor.c):
    motor 0 = TIM1 ch1, GPIO PA8   (motor_outputs.m1)
    motor 1 = TIM1 ch2, GPIO PA9   (motor_outputs.m2)
    motor 2 = TIM1 ch3, GPIO PA10  (motor_outputs.m3)
    motor 3 = TIM1 ch4, GPIO PA11  (motor_outputs.m4)
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

import gz.transport13 as transport
from gz.msgs10.actuators_pb2 import Actuators

DEFAULT_FIFO   = "/tmp/vayu_pwm.fifo"
DEFAULT_TOPIC  = "/X3/gazebo/command/motor_speed"
DEFAULT_RATE   = 200    # Hz
N_MOTORS       = 4

# Map a 0..1 duty cycle to a rotor angular velocity in rad/s. The
# MulticopterMotorModel plugin in tools/sim_gazebo/worlds/vayu_quad.sdf
# uses maxRotVelocity=800. Real ESCs map 1ms..2ms pulses at 400 Hz to
# 0..100% throttle. esc_set_throttle() takes 0..1 directly, so the
# duty -> velocity mapping is linear over [0, MAX_ROT_VEL].
MAX_ROT_VEL_RAD_S = 800.0


class MotorState:
    """Last-write-wins per-motor duty cycle."""

    __slots__ = ("_lock", "duty", "updates")

    def __init__(self):
        self._lock = threading.Lock()
        self.duty    = [0.0] * N_MOTORS
        self.updates = 0

    def set(self, idx: int, duty: float) -> None:
        if 0 <= idx < N_MOTORS:
            with self._lock:
                self.duty[idx] = max(0.0, min(1.0, duty))
                self.updates += 1

    def snapshot(self):
        with self._lock:
            return list(self.duty)


def fifo_reader(path: str, state: MotorState) -> None:
    """Daemon thread: read 'idx duty' lines from the Renode-side FIFO.
    Tolerates the writer end disappearing and reappearing."""
    while True:
        try:
            with open(path, "r") as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) != 2:
                        continue
                    try:
                        idx = int(parts[0])
                        duty = float(parts[1])
                    except ValueError:
                        continue
                    state.set(idx, duty)
        except OSError as e:
            print("vayu_pwm_to_gz: fifo open err:", e, file=sys.stderr)
        time.sleep(0.1)


def test_source(state: MotorState) -> None:
    """Synthetic stand-in for the Renode FIFO: ramps motor 0 alone
    from 0..1 and back, leaves motors 1-3 at hover (0.6). Lets you
    verify the gz-transport half end-to-end without Renode running."""
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        duty0 = 0.5 + 0.5 * math.sin(2 * math.pi * 0.25 * t)
        state.set(0, duty0)
        for i in (1, 2, 3):
            state.set(i, 0.6)
        time.sleep(0.01)


def run(fifo_path: str, topic: str, rate_hz: float, source: str,
        verbose: bool) -> int:
    state = MotorState()
    if source == "test":
        t = threading.Thread(target=test_source, args=(state,), daemon=True)
    else:
        # Wait up to 30 s for the Renode-side peripheral to create the
        # FIFO. Lets us start the bridge before Renode without bailing.
        waited = 0.0
        while not os.path.exists(fifo_path) and waited < 30.0:
            time.sleep(0.5)
            waited += 0.5
        if not os.path.exists(fifo_path):
            print("ERR: {} did not appear within 30 s. Is Renode running "
                  "the vayu.resc with the pwm_extract peripheral?"
                  .format(fifo_path), file=sys.stderr)
            return 2
        t = threading.Thread(target=fifo_reader,
                             args=(fifo_path, state), daemon=True)
    t.start()

    node = transport.Node()
    pub  = node.advertise(topic, Actuators)
    print("publishing to:", topic, "at", rate_hz, "Hz", flush=True)

    period   = 1.0 / rate_hz
    next_pub = time.monotonic()
    last_log = next_pub
    msg      = Actuators()
    while True:
        duty = state.snapshot()
        velocities = [d * MAX_ROT_VEL_RAD_S for d in duty]
        del msg.velocity[:]
        msg.velocity.extend(velocities)
        pub.publish(msg)

        now = time.monotonic()
        if verbose and now - last_log >= 1.0:
            last_log = now
            print("updates={} duty=[{:.2f} {:.2f} {:.2f} {:.2f}] "
                  "vel_rad_s=[{:.0f} {:.0f} {:.0f} {:.0f}]"
                  .format(state.updates, *duty, *velocities), flush=True)

        next_pub += period
        sleep_for = next_pub - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_pub = time.monotonic()


def main(argv) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fifo",   default=DEFAULT_FIFO)
    ap.add_argument("--topic",  default=DEFAULT_TOPIC)
    ap.add_argument("--rate",   type=float, default=DEFAULT_RATE)
    ap.add_argument("--source", default="fifo",
                    choices=("fifo", "test"),
                    help='"fifo" reads from the Renode-side FIFO; "test" '
                         'generates a synthetic ramp on motor 0 so you '
                         'can verify the gz publisher without Renode.')
    ap.add_argument("--quiet",  action="store_true")
    args = ap.parse_args(argv)
    try:
        return run(args.fifo, args.topic, args.rate, args.source,
                   verbose=not args.quiet)
    except KeyboardInterrupt:
        print("\nvayu_pwm_to_gz: stopped", file=sys.stderr)
        return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


# ---------------------------------------------------------------------
# Renode hookup
# ---------------------------------------------------------------------
# Done — see tools/sim_renode/pwm_extract_mock.py. That peripheral
# replaces Renode's stock STM32_Timer at TIM1's base (0x40010000),
# shadows enough of the timer register layout that NavHAL's reads
# still work, computes duty = CCR / (ARR + 1) on every CCRx write,
# and writes "idx duty\n" lines to /tmp/vayu_pwm.fifo. ARR is fixed
# at 2499 in normal operation because hal_pwm_init derives PSC to
# give a 1 MHz timer tick (so a 400 Hz PWM has ARR = 1e6/400 - 1).
