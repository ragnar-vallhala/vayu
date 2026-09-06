#!/usr/bin/env python3
"""Record + narrate a hand-lift height-mode run on real hardware.

ONE process binds :14555 (a second binder splits the ESP bridge's coalesced
datagrams and makes the link look dead). Writes the raw stream to a VREC .bin
so it replays through the same tooling as a Navigator export, and prints a live
line whenever something that matters CHANGES -- arm state, height-mode bits,
ToF validity -- plus a periodic sample so a steady phase still shows progress.

    python3 tools/telemetry/handlift_record.py OUT.bin
"""
import os, sys, socket, struct, time, collections

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "navlink", "sim"))
sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))
import frame, navlink_msgs as N  # noqa: E402

# HEARTBEAT.nav_state is the INDEX of the set bit in the firmware's one-hot
# sys_state_t (navlink_tx.c: __builtin_ctz), not the mask itself.
NAV = {0: "UNINIT", 1: "INIT", 2: "STANDBY", 3: "PREARM", 4: "ARMED",
       5: "IN_AIR", 6: "FAILSAFE", 7: "TERM", 8: "CALIB"}
HMODE = {0: "OFF", 1: "HOLD", 2: "LAND"}
HBITS = [(0x04, "ENGAGED"), (0x08, "FAILED"), (0x10, "LANDED"),
         (0x20, "HANDBACK"), (0x40, "ARMED_OK"), (0x80, "BLOCKED")]


def hstate_str(b):
    out = HMODE.get(b & 0x03, "?")
    for m, n in HBITS:
        if b & m:
            out += "|" + n
    return out


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "handlift.bin"
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind(("0.0.0.0", 14555))
    s.settimeout(0.2)
    f = open(out, "wb")
    f.write(struct.pack("<IIIQ", 0x56524543, 1, 1, int(time.time() * 1000)))

    t0 = time.time()
    last_hello = 0.0
    last_print = 0.0
    n_dg = 0
    prev = {}          # what we last announced
    motors = [0.0] * 4
    nav = None
    vs = None
    hdr = (f"{'t':>6} {'state':>8} {'height':>22} {'agl_tof':>8} {'tof':>4} "
           f"{'alt':>8} {'climb':>7} {'bias':>7} {'unh':>4} {'motors':>25}")
    print(hdr, flush=True)

    def line(tag):
        if vs is None:
            return
        m = " ".join(f"{x:5.2f}" for x in motors)
        print(f"{time.time()-t0:6.1f} {NAV.get(nav,str(nav)):>8} "
              f"{hstate_str(vs.height_state):>22} {vs.agl_tof:8.3f} "
              f"{vs.tof_valid:>4} {vs.altitude:8.2f} {vs.climb_rate:+7.3f} "
              f"{vs.accel_bias:+7.3f} {vs.accel_unhealthy:>4} {m:>25}  {tag}",
              flush=True)

    try:
        while True:
            now = time.time()
            if now - last_hello > 1.0:
                try:
                    s.sendto(b"GCS-HELLO", ("10.42.0.255", 14555))
                except OSError:
                    pass
                last_hello = now
            try:
                data, _ = s.recvfrom(4096)
            except socket.timeout:
                continue
            if data.startswith(b"GCS-HELLO"):
                continue
            f.write(struct.pack("<QI", int((now - t0) * 1e6), len(data)))
            f.write(data)
            n_dg += 1
            off = 0
            while len(data) - off >= frame.HDR_LEN + 2:
                if data[off] != 0x56 or data[off + 1] != 0x02:
                    off += 1
                    continue
                flen = frame.HDR_LEN + data[off + 2] + 2
                if len(data) - off < flen:
                    break
                d = frame.decode(data[off:off + flen])
                off += flen
                if not d.ok:
                    continue
                if d.msgid == N.Heartbeat.MSGID:
                    nav = N.Heartbeat.unpack(d.payload).nav_state
                elif d.msgid == N.MotorTelemetry.MSGID:
                    motors = list(N.MotorTelemetry.unpack(d.payload).cmd)[:4]
                elif d.msgid == N.VerticalState.MSGID:
                    vs = N.VerticalState.unpack(d.payload)
            if vs is None:
                continue
            # Announce only on a CHANGE, so the transitions stand out in the log.
            key = (nav, vs.height_state, bool(vs.tof_valid), bool(vs.accel_unhealthy))
            if key != prev.get("k"):
                prev["k"] = key
                line("<-- CHANGE")
                last_print = now
            elif now - last_print >= 1.0:
                line("")
                last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
        f.close()
        print(f"\n[rec] {n_dg} datagrams -> {out}  ({os.path.getsize(out)/1024:.0f} KiB)",
              flush=True)


if __name__ == "__main__":
    main()
