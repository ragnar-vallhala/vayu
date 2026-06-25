#!/usr/bin/env python3
"""sim_drive.py — headless driver for the vsim_d physics daemon.

Demonstrates the full FIFO control surface with no Navigator/Qt and no
firmware: spawn an isolated vsim_d, push control frames (reset / world /
wind), stream motor PWM (the actuator input), and read back the pose +
IMU streams — i.e. drive every input and read all the data from a script.

  Channels (tools/vsim/include/vsim_proto.h), all /tmp/vsim_*$SUFFIX:
    ctl  (write)  reset, world, wind, ... (this script wires a useful subset)
    pwm  (write)  4 motor duties [0,1]   — open-loop actuator command
    pose (read)   full rigid-body state + motor + wind/airspeed/batt (proto v3)
    imu  (read)   acc / gyr / mag / temp  (88 B)

Examples:
  # 0.34 duty (~hover) in a 5 m/s north wind, 3 s, print a summary:
  VSIM_BIN_PATH=tools/vsim/build/vsim_d python3 tools/vsim/tests/sim_drive.py \
      --duty 0.34 --wind 5 0 0 --secs 3
  # dump every pose row to CSV for offline analysis:
  ... --csv /tmp/run.csv
"""
import argparse
import os
import struct
import subprocess
import sys
import time

MAGIC = 0x4D495356
VERSION = 3
FRAME_PWM, FRAME_IMU, FRAME_POSE, FRAME_CTL = 1, 2, 3, 4
CTL_RESET, CTL_SET_WORLD, CTL_SET_WIND = 1, 6, 14

HDR = struct.Struct("<IHHII")                       # magic,ver,type,plen,seq (16)
POSE = struct.Struct("<II3f4f3f3f4f4f3fffffff")     # v3 body = 128 B
IMU = struct.Struct("<22f")                          # acc..temp = 88 B
CTL_BODY = 256
CTL = struct.Struct("<II")                           # subtype, reserved
assert HDR.size == 16 and POSE.size == 128 and IMU.size == 88


def hdr(typ, plen, seq):
    return HDR.pack(MAGIC, VERSION, typ, plen, seq)


def ctl_frame(subtype, body):
    body = body[:CTL_BODY].ljust(CTL_BODY, b"\x00")
    payload = CTL.pack(subtype, 0) + body
    return hdr(FRAME_CTL, len(payload), 0) + payload


def reset_frame(pos=(0, 0, -10.0), seed=1):
    body = struct.pack("<3f4f3f3fI", pos[0], pos[1], pos[2],
                       1.0, 0.0, 0.0, 0.0,        # quat wxyz (level)
                       0, 0, 0,                    # vel
                       0, 0, 0,                    # omega
                       seed)
    return ctl_frame(CTL_RESET, body)


def world_frame(gravity=9.81, ground_z=0.0, restitution=0.0,
                linear_drag=0.20, angular_drag=0.005,
                right_gain=40.0, right_damp=6.0):
    body = struct.pack("<7f", gravity, ground_z, restitution, linear_drag,
                       angular_drag, right_gain, right_damp)
    return ctl_frame(CTL_SET_WORLD, body)


def wind_frame(steady, gust_amp=0.0, gust_period=0.0, turb_sigma=0.0,
               turb_tau=1.0, enable=True):
    body = struct.pack("<3f4fi", steady[0], steady[1], steady[2],
                       gust_amp, gust_period, turb_sigma, turb_tau,
                       1 if enable else 0)
    return ctl_frame(CTL_SET_WIND, body)


def pwm_frame(duty4, seq):
    return hdr(FRAME_PWM, 16, seq) + struct.pack("<4f", *duty4)


def read_latest(fd, parser, typ):
    """Drain a read FIFO, return the newest decoded body of `typ` or None."""
    chunk = b""
    try:
        while True:
            part = os.read(fd, 65536)
            if not part:
                break
            chunk += part
    except BlockingIOError:
        pass
    if not chunk:
        return None
    out, pos = None, 0
    while pos + HDR.size <= len(chunk):
        magic, ver, t, plen, seq = HDR.unpack_from(chunk, pos)
        if magic != MAGIC:
            pos += 1
            continue
        if pos + HDR.size + plen > len(chunk):
            break
        if ver == VERSION and t == typ:
            try:
                out = parser.unpack_from(chunk, pos + HDR.size)
            except struct.error:
                pass
        pos += HDR.size + plen
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duty", type=float, default=0.34, help="motor duty [0,1]")
    ap.add_argument("--wind", type=float, nargs=3, metavar=("N", "E", "D"),
                    default=[0.0, 0.0, 0.0], help="steady wind m/s NED")
    ap.add_argument("--turb", type=float, default=0.0, help="turbulence sigma m/s")
    ap.add_argument("--secs", type=float, default=3.0)
    ap.add_argument("--rate", type=int, default=1000, help="PWM/IMU loop Hz")
    ap.add_argument("--csv", type=str, default="", help="dump pose rows here")
    args = ap.parse_args()

    _repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    binp = os.environ.get(
        "VSIM_BIN_PATH", os.path.join(_repo, "tools", "vsim", "build", "vsim_d"))
    if not os.path.exists(binp):
        print(f"FAIL: vsim_d not found at {binp} (set VSIM_BIN_PATH)")
        return 2
    suffix = f"_drive{os.getpid()}"
    env = dict(os.environ, VSIM_FIFO_SUFFIX=suffix)
    paths = {n: f"/tmp/vsim_{n}{suffix}" for n in ("pwm", "imu", "pose", "ctl")}

    daemon = subprocess.Popen([binp], env=env,
                              stderr=subprocess.DEVNULL)
    try:
        # Wait for the daemon to mkfifo all four channels.
        for _ in range(200):
            if all(os.path.exists(p) for p in paths.values()):
                break
            time.sleep(0.01)
        else:
            print("FAIL: daemon did not create FIFOs")
            return 3
        # O_RDWR|O_NONBLOCK matches the daemon — open never blocks either way.
        fds = {n: os.open(p, os.O_RDWR | os.O_NONBLOCK) for n, p in paths.items()}

        # --- drive inputs: configure, then stream PWM ---------------------
        os.write(fds["ctl"], reset_frame(seed=1))
        os.write(fds["ctl"], world_frame())
        os.write(fds["ctl"], wind_frame(args.wind, turb_sigma=args.turb,
                                        enable=(any(args.wind) or args.turb > 0)))
        time.sleep(0.05)

        duty = (args.duty,) * 4
        csv = open(args.csv, "w") if args.csv else None
        if csv:
            csv.write("t,posN,posE,posD,velN,velE,velD,"
                      "windN,windE,windD,m0,m1,m2,m3\n")
        n_pose = n_imu = 0
        first_pos = last_pose = last_imu = None
        t0 = time.time()
        dt = 1.0 / args.rate
        seq = 0
        next_t = t0
        while time.time() - t0 < args.secs:
            seq += 1
            os.write(fds["pwm"], pwm_frame(duty, seq))
            p = read_latest(fds["pose"], POSE, FRAME_POSE)
            if p:
                n_pose += 1
                last_pose = p
                if first_pos is None:
                    first_pos = p[2:5]
                if csv:
                    csv.write("%.3f,%s,%s,%s\n" % (
                        time.time() - t0,
                        ",".join(f"{v:.4f}" for v in p[2:5]),    # pos NED
                        ",".join(f"{v:.4f}" for v in p[9:12]),   # vel NED
                        ",".join(f"{v:.4f}" for v in p[23:26]) + "," +
                        ",".join(f"{v:.1f}" for v in p[15:19]))) # wind, motor_omega
            im = read_latest(fds["imu"], IMU, FRAME_IMU)
            if im:
                n_imu += 1
                last_imu = im
            next_t += dt
            slp = next_t - time.time()
            if slp > 0:
                time.sleep(slp)
        if csv:
            csv.close()

        # --- report -------------------------------------------------------
        if not last_pose:
            print("FAIL: no pose frames received")
            return 4
        p = last_pose
        print(f"OK: drove {seq} PWM frames, read {n_pose} pose + {n_imu} imu")
        print(f"    duty={args.duty}  wind(N,E,D)={tuple(args.wind)} turb={args.turb}")
        print(f"    pos_w  : ({p[2]:+.3f}, {p[3]:+.3f}, {p[4]:+.3f}) m  (NED)")
        print(f"    vel_w  : ({p[9]:+.3f}, {p[10]:+.3f}, {p[11]:+.3f}) m/s")
        print(f"    wind_w : ({p[23]:+.3f}, {p[24]:+.3f}, {p[25]:+.3f}) m/s  <-- from daemon")
        print(f"    airspd : {p[26]:.3f} m/s   ge×{p[27]:.3f}   batt {p[28]:.2f} V "
              f"(airspeed/ge/batt are reserved -> 0 until Phases 2/5)")
        print(f"    motorω : ({p[15]:.0f}, {p[16]:.0f}, {p[17]:.0f}, {p[18]:.0f}) rad/s")
        if last_imu:
            print(f"    imu acc: ({last_imu[0]:+.3f}, {last_imu[1]:+.3f}, {last_imu[2]:+.3f}) m/s²")
            print(f"    imu gyr: ({last_imu[3]:+.3f}, {last_imu[4]:+.3f}, {last_imu[5]:+.3f}) deg/s")
        if first_pos:
            dN, dE = p[2] - first_pos[0], p[3] - first_pos[1]
            print(f"    horiz drift over run: dN={dN:+.3f} dE={dE:+.3f} m "
                  f"(wind pushes the craft when enabled)")
        if args.csv:
            print(f"    wrote pose CSV -> {args.csv}")
        return 0
    finally:
        daemon.terminate()
        try:
            daemon.wait(timeout=2)
        except subprocess.TimeoutExpired:
            daemon.kill()
        for p in paths.values():
            try:
                os.unlink(p)
            except OSError:
                pass


if __name__ == "__main__":
    sys.exit(main())
