#!/usr/bin/env python3
"""
vayu_pwm_to_gz.py - motor PWM out -> Gazebo bridge.

Reads motor duty cycles from /tmp/vayu_pwm.fifo (one "motor_idx duty\\n"
line per update, written by the host SITL binary), maps duty -> rotor
velocity, and publishes gz.msgs.Actuators on
/X3/gazebo/command/motor_speed at 200 Hz.

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
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity

DEFAULT_FIFO         = "/tmp/vayu_pwm.fifo"
DEFAULT_TOPIC        = "/X3/gazebo/command/motor_speed"
DEFAULT_WRENCH_TOPIC = "/world/vayu_quad_world/wrench/persistent"
DEFAULT_WRENCH_LINK  = "X3::base_link"
DEFAULT_RATE         = 200    # Hz
N_MOTORS             = 4

# motorConstant matches the value in tools/sim_gazebo/worlds/vayu_quad.sdf.
# Per-motor thrust = motorConstant * vel^2; total thrust is the sum.
MOTOR_CONSTANT = 8.54858e-06

# Map an ESC duty cycle to a rotor angular velocity. NavHAL's PWM is
# configured at 400 Hz (period 2.5 ms); esc_set_throttle() converts a
# 0..1 throttle into a 1..2 ms pulse, so the duty cycle vayu writes is
#     duty = (pulse_ms) / 2.5 ms
#            in [1/2.5 = 0.4 (idle / 0% throttle),
#               2/2.5 = 0.8 (max / 100% throttle)]
# We have to undo that offset before mapping to rotor velocity, or 0%
# throttle ends up commanding 0.4 * MAX_ROT_VEL rad/s of phantom thrust
# and 100% throttle only gets 0.8 * MAX_ROT_VEL - leaving the drone
# under-thrusted and unable to lift off at full stick.
PWM_IDLE_DUTY     = 0.4   # 1.0 ms / 2.5 ms
PWM_FULL_DUTY     = 0.8   # 2.0 ms / 2.5 ms
MAX_ROT_VEL_RAD_S = 800.0 # matches <maxRotVelocity> in vayu_quad.sdf


def duty_to_velocity(d: float) -> float:
    """ESC duty (0.4..0.8) -> rotor velocity (0..MAX_ROT_VEL_RAD_S).
    Clamps anything outside the band to its endpoint."""
    if d <= PWM_IDLE_DUTY:
        return 0.0
    if d >= PWM_FULL_DUTY:
        return MAX_ROT_VEL_RAD_S
    throttle = (d - PWM_IDLE_DUTY) / (PWM_FULL_DUTY - PWM_IDLE_DUTY)
    return throttle * MAX_ROT_VEL_RAD_S


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
    """Daemon thread: read 'idx duty' lines from the FIFO.
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
    """Synthetic stand-in for the FIFO writer: ramps motor 0 alone
    from 0..1 and back, leaves motors 1-3 at hover (0.6). Lets you
    verify the gz-transport half end-to-end without the SITL binary
    running."""
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
        # Wait up to 30 s for the SITL binary to create the FIFO. Lets
        # us start the bridge before the SITL binary without bailing.
        waited = 0.0
        while not os.path.exists(fifo_path) and waited < 30.0:
            time.sleep(0.5)
            waited += 0.5
        if not os.path.exists(fifo_path):
            print("ERR: {} did not appear within 30 s. Is the SITL "
                  "binary running?".format(fifo_path), file=sys.stderr)
            return 2
        t = threading.Thread(target=fifo_reader,
                             args=(fifo_path, state), daemon=True)
    t.start()

    node = transport.Node()
    pub  = node.advertise(topic, Actuators)
    print("publishing to:", topic, "at", rate_hz, "Hz", flush=True)

    # Also publish a vertical wrench on base_link via the
    # ApplyLinkWrench system plugin. This is a workaround for
    # Gazebo 8.11's MulticopterMotorModel failing to apply rotor
    # forces under bullet-featherstone (and aborting under dartsim
    # for multirotor worlds). The wrench is the SUM of the four
    # per-motor thrusts, applied upward at base_link's origin. That
    # gives us liftoff in passthrough mode where all four motors are
    # equal; once the firmware-side PID is tuned for sim, per-rotor
    # wrenches at each rotor link can replace this single base_link
    # wrench for proper attitude control.
    wpub = node.advertise(DEFAULT_WRENCH_TOPIC, EntityWrench)
    if wpub:
        print("publishing wrench to:", DEFAULT_WRENCH_TOPIC,
              "on", DEFAULT_WRENCH_LINK, flush=True)
    else:
        print("WARN: wrench advertise failed; multicopter plugin only",
              file=sys.stderr)

    wmsg = EntityWrench()
    wmsg.entity.name = DEFAULT_WRENCH_LINK
    wmsg.entity.type = Entity.LINK

    period   = 1.0 / rate_hz
    next_pub = time.monotonic()
    last_log = next_pub
    msg      = Actuators()
    while True:
        duty = state.snapshot()
        velocities = [duty_to_velocity(d) for d in duty]
        del msg.velocity[:]
        msg.velocity.extend(velocities)
        pub.publish(msg)

        # Vertical lift wrench = sum of per-motor thrusts.
        total_thrust = sum(MOTOR_CONSTANT * (v * v) for v in velocities)
        wmsg.wrench.force.x = 0.0
        wmsg.wrench.force.y = 0.0
        wmsg.wrench.force.z = total_thrust
        wmsg.wrench.torque.x = 0.0
        wmsg.wrench.torque.y = 0.0
        wmsg.wrench.torque.z = 0.0
        if wpub:
            wpub.publish(wmsg)

        now = time.monotonic()
        if verbose and now - last_log >= 1.0:
            last_log = now
            print("updates={} duty=[{:.2f} {:.2f} {:.2f} {:.2f}] "
                  "vel_rad_s=[{:.0f} {:.0f} {:.0f} {:.0f}] "
                  "F_up={:.2f} N"
                  .format(state.updates, *duty, *velocities, total_thrust),
                  flush=True)

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
                    help='"fifo" reads from the SITL FIFO; "test" '
                         'generates a synthetic ramp on motor 0 so you '
                         'can verify the gz publisher standalone.')
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
