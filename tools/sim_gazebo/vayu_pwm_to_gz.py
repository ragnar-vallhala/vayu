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
DEFAULT_CLEAR_TOPIC  = "/world/vayu_quad_world/wrench/clear"
DEFAULT_BASE_LINK    = "X3::base_link"
# 100 Hz is plenty for motor commands (motor_task runs at 500 Hz on the
# firmware side; the bridge being below that is fine because the persistent
# wrench stays active between updates). Higher rates were starving the
# physics engine and pulling real-time-factor down to ~2%.
DEFAULT_RATE         = 100
N_MOTORS             = 4

# motorConstant + momentConstant match tools/sim_gazebo/worlds/vayu_quad.sdf.
# Per-rotor thrust = motorConstant * vel^2 along the rotor's +Z (body up).
# Per-rotor counter-torque on the body = -turningDirection * motorConstant *
#   momentConstant * vel^2 about base_link's Z (aerodynamic drag from the
#   prop spinning against air). Joint can't transmit this naturally because
#   it's exactly aligned with the joint's free axis, so we apply it on the
#   parent (base_link) like the real MulticopterMotorModel does.
MOTOR_CONSTANT  = 8.54858e-06
MOMENT_CONSTANT = 0.016

# Per-motor (x, y) position in base_link's frame and spin direction.
# These DO NOT have to match the X3's actual rotor indices - we apply a
# single aggregated wrench on base_link rather than per-rotor forces, so
# only the moment arms we use here matter. They must match vayu's motor
# mixing convention as declared in src/control/angle_rate_controller.c:
#   M1 = Front Right (FR)
#   M2 = Rear Right  (RR)
#   M3 = Rear Left   (RL)
#   M4 = Front Left  (FL)
# Body frame is X-forward, Y-left, Z-up (Gazebo SDF / FLU). So in body:
#   FR = (+0.13, -0.22)    front, right (= -Y)
#   RR = (-0.13, -0.20)    back,  right
#   RL = (-0.13, +0.20)    back,  left
#   FL = (+0.13, +0.22)    front, left
#
# We aggregate every rotor's force + counter-torque into a single wrench
# on base_link (bullet-featherstone's articulated solver ignores external
# wrenches on non-root links), computing the moment arms ourselves:
#   F_body  = (0, 0, sum_i motorConstant * vel_i^2)
#   tau_x   = sum_i ( pos_y_i  * motorConstant * vel_i^2 )            // roll
#   tau_y   = sum_i ( -pos_x_i * motorConstant * vel_i^2 )            // pitch
#   tau_z   = sum_i ( -spin_i  * motorConstant * momentConstant * vel_i^2 )
#
# ROTOR_SPIN: vayu's yaw mixing makes M1, M3 increase together (one
# diagonal) and M2, M4 increase together (the other), so those diagonals
# spin in opposite directions. With our sign convention (CCW=+1, CW=-1),
# the choice that makes vayu's "+yaw output" produce vayu's intended
# turn direction (right / clockwise from above in the NED-style sticks)
# is M1+M3 CCW, M2+M4 CW.
ROTOR_POS_X = [+0.13, -0.13, -0.13, +0.13]   # M1, M2, M3, M4
ROTOR_POS_Y = [-0.22, -0.20, +0.20, +0.22]
ROTOR_SPIN  = [+1,    -1,    +1,    -1]      # M1+M3 CCW, M2+M4 CW

# The SITL host shim (tools/sim_host/src/host_navhal.c) already strips
# the firmware's ESC 0.4..0.8 pulse-width band, so the FIFO carries a
# direct linear motor command in [0, 1]. We just multiply by the
# rotor's max velocity to get rad/s.
MAX_ROT_VEL_RAD_S = 1200.0 # matches <maxRotVelocity> in vayu_quad.sdf


def duty_to_velocity(d: float) -> float:
    """Linear motor command (0..1) -> rotor velocity."""
    if d <= 0.0:
        return 0.0
    if d >= 1.0:
        return MAX_ROT_VEL_RAD_S
    return d * MAX_ROT_VEL_RAD_S


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

    # Single wrench on X3::base_link. bullet-featherstone's articulated
    # body solver only accepts external wrenches on the root link, so we
    # aggregate every rotor's thrust + counter-torque into one base_link
    # wrench, computing r x F moment arms ourselves from each rotor's
    # known (x, y) position in body frame.
    #
    # The torque vector is in body frame but published as a world-frame
    # wrench by ApplyLinkWrench. While the drone is roughly level this is
    # a good approximation; for large tilt angles the body-vs-world
    # mismatch would call for orientation-tracking and rotating the
    # torque vector each step. For now, level-attitude PID stabilization
    # is the regime we need.
    #
    # The persistent topic keeps applying the most recent wrench every
    # sim step. When motors drop to zero we explicitly publish to the
    # /wrench/clear topic - the plugin's persistent map drops the entry
    # so gravity actually pulls the drone back down. (Otherwise the
    # zero-wrench update is fine for force calc but tests show the
    # plugin can latch when force/torque go all-zero on persistent.)
    wpub = node.advertise(DEFAULT_WRENCH_TOPIC, EntityWrench)
    cpub = node.advertise(DEFAULT_CLEAR_TOPIC, Entity)
    if wpub:
        print("publishing base_link wrench to:", DEFAULT_WRENCH_TOPIC,
              "on", DEFAULT_BASE_LINK, flush=True)
    if cpub:
        print("clear topic:", DEFAULT_CLEAR_TOPIC, flush=True)

    base_msg = EntityWrench()
    base_msg.entity.name = DEFAULT_BASE_LINK
    base_msg.entity.type = Entity.LINK

    clear_msg = Entity()
    clear_msg.name = DEFAULT_BASE_LINK
    clear_msg.type = Entity.LINK

    last_was_active = False  # only publish clear when transitioning to idle

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

        # Per-motor thrust and counter-torque magnitudes.
        thrusts = [MOTOR_CONSTANT * (v * v) for v in velocities]

        F_z = sum(thrusts)
        tau_x = sum(ROTOR_POS_Y[i] * thrusts[i]  for i in range(4))   # roll
        tau_y = sum(-ROTOR_POS_X[i] * thrusts[i] for i in range(4))   # pitch
        tau_z = sum(-ROTOR_SPIN[i] * thrusts[i] * MOMENT_CONSTANT
                    for i in range(4))                                # yaw

        active = (F_z > 1e-6)
        if active:
            base_msg.wrench.force.x  = 0.0
            base_msg.wrench.force.y  = 0.0
            base_msg.wrench.force.z  = F_z
            base_msg.wrench.torque.x = tau_x
            base_msg.wrench.torque.y = tau_y
            base_msg.wrench.torque.z = tau_z
            if wpub:
                wpub.publish(base_msg)
        elif last_was_active:
            # Motors just went idle; release the persistent wrench so
            # gravity takes over cleanly.
            if cpub:
                cpub.publish(clear_msg)
        last_was_active = active

        now = time.monotonic()
        if verbose and now - last_log >= 1.0:
            last_log = now
            print("updates={} duty=[{:.2f} {:.2f} {:.2f} {:.2f}] "
                  "vel=[{:.0f} {:.0f} {:.0f} {:.0f}] "
                  "F_z={:.2f} N tau=[{:+.3f} {:+.3f} {:+.3f}] Nm"
                  .format(state.updates, *duty, *velocities,
                          F_z, tau_x, tau_y, tau_z),
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
