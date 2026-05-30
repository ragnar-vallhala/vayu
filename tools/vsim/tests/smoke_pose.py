#!/usr/bin/env python3
"""vsim_d pose-FIFO smoke test.

Reads /tmp/vsim_pose for a short window, decodes vsim_pose_frame_t frames,
validates the wire header against vsim_proto.h, and prints a physics
sanity summary. With no firmware feeding PWM the drone gets zero thrust,
so under gravity its NED z (down-positive) should increase from the spawn
pose (z=-0.05) toward the ground clamp.

Exit 0 on a healthy stream, non-zero otherwise.
"""
import os
import struct
import sys
import time

POSE = "/tmp/vsim_pose"
MAGIC = 0x4D495356
VERSION = 1
FRAME_POSE = 3

# vsim_hdr_t: magic u32, version u16, type u16, payload_bytes u32, seq u32
HDR = struct.Struct("<IHHII")
# body: tick_lo u32, tick_hi u32, pos[3]f, quat[4]f, vel[3]f, omega[3]f,
#       motor_omega[4]f, motor_duty[4]f  = 8 + 92? -> 2*4 + (3+4+3+3+4+4)*4
BODY = struct.Struct("<II3f4f3f3f4f4f")
FRAME_SIZE = HDR.size + BODY.size

assert HDR.size == 16, HDR.size
assert FRAME_SIZE == 108, FRAME_SIZE  # 16 + 92

def main():
    if not os.path.exists(POSE):
        print(f"FAIL: {POSE} does not exist (daemon not running / FIFO not created)")
        return 2
    fd = os.open(POSE, os.O_RDONLY | os.O_NONBLOCK)
    buf = b""
    frames = []
    deadline = time.time() + 1.5
    while time.time() < deadline and len(frames) < 200:
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            chunk = b""
        if chunk:
            buf += chunk
        # parse complete frames, resyncing on magic
        pos = 0
        while pos + HDR.size <= len(buf):
            magic, ver, typ, plen, seq = HDR.unpack_from(buf, pos)
            if magic != MAGIC:
                pos += 1
                continue
            need = HDR.size + plen
            if pos + need > len(buf):
                break
            if ver == VERSION and typ == FRAME_POSE and need == FRAME_SIZE:
                body = BODY.unpack_from(buf, pos + HDR.size)
                frames.append((seq, body))
            pos += need
        buf = buf[pos:]
        if not chunk:
            time.sleep(0.01)
    os.close(fd)

    if not frames:
        print("FAIL: no valid pose frames decoded")
        return 3

    seqs = [f[0] for f in frames]
    first_z = frames[0][1][4]   # pos_w[2]
    last_z = frames[-1][1][4]
    last = frames[-1][1]
    pos_w = last[2:5]
    quat = last[5:9]
    vel_w = last[9:12]

    print(f"OK: decoded {len(frames)} pose frames")
    print(f"    seq range:   {seqs[0]} .. {seqs[-1]} (monotonic={all(b>a for a,b in zip(seqs,seqs[1:]))})")
    print(f"    pos_w (NED): ({pos_w[0]:+.3f}, {pos_w[1]:+.3f}, {pos_w[2]:+.3f}) m")
    print(f"    vel_w (NED): ({vel_w[0]:+.3f}, {vel_w[1]:+.3f}, {vel_w[2]:+.3f}) m/s")
    print(f"    quat wxyz:   ({quat[0]:+.3f}, {quat[1]:+.3f}, {quat[2]:+.3f}, {quat[3]:+.3f})")
    print(f"    z drift:     {first_z:+.4f} -> {last_z:+.4f} m (down-positive)")

    qn = (quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2) ** 0.5
    problems = []
    if not all(b > a for a, b in zip(seqs, seqs[1:])):
        problems.append("seq not monotonic")
    if abs(qn - 1.0) > 1e-2:
        problems.append(f"quaternion not unit (|q|={qn:.4f})")
    # zero thrust -> must fall (z increases, down-positive). Allow tiny window.
    if last_z <= first_z - 1e-4:
        problems.append(f"drone rose with zero thrust (z {first_z:.4f}->{last_z:.4f})")
    if problems:
        print("FAIL: " + "; ".join(problems))
        return 4
    print("PASS: stream healthy, physics plausible (falls under gravity)")
    return 0

if __name__ == "__main__":
    sys.exit(main())
