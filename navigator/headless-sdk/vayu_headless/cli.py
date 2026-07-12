"""vayu-headless CLI: serve | do | run.

  serve   boot the persistent session ONCE (attach the GCS a single time),
          then accept flight commands over the control socket.
  do      send one command to a running session (text or JSON), print the reply.
  run     one-shot: boot, fly a course for N seconds, print a summary.
"""
import argparse
import time

from . import client, paths
from .autopilot import Pilot
from .server import SessionServer, course_from
from .session import SitlSession


def _add_world_opts(ap):
    ap.add_argument("--conf", default="", help="GCS .conf to match vveh/vworld "
                    "(default: ~/.config/Vayu/Vayu GCS.conf)")
    ap.add_argument("--wind", type=float, nargs=3, default=[0, 0, 0],
                    metavar=("N", "E", "D"))
    ap.add_argument("--turb", type=float, default=0.0)


def _cmd_serve(args):
    conf = args.conf or paths.gcs_conf_default()
    lab = SitlSession(attach=True, gcs=True, conf=conf,
                      wind=tuple(args.wind), turb=args.turb)
    pilot = Pilot(lab, alt=args.alt)
    print("engine on DEFAULT /tmp/vsim_* (pose=/tmp/vsim_pose)")
    print("  ▶ In Navigator (ONCE): click 'Attach Ext' to render, and "
          "Connect → 'SITL UART2' for telemetry.")
    print(f"    (telemetry bridge pty: {lab.gcs_path})")
    print(f"  control socket: {paths.SOCK_PATH}")
    if args.gcs_wait > 0:
        print(f"  waiting {args.gcs_wait:.0f}s for you to attach + connect…")
        time.sleep(args.gcs_wait)
    print("  session ready — send commands with: vayu-headless do '<cmd>'")
    SessionServer(lab, pilot).serve_forever(paths.SOCK_PATH)
    print("session closed")
    return 0


def _cmd_do(args):
    return client.main(" ".join(args.command))


def _cmd_run(args):
    conf = args.conf or paths.gcs_conf_default()
    if args.course:
        wps = course_from(args.course)
    else:
        d = args.box
        wps = [(d, 0), (d, d), (0, d), (0, 0), (d, 0)]
    with SitlSession(attach=True, gcs=args.gcs, conf=conf,
                     wind=tuple(args.wind), turb=args.turb) as lab:
        print(f"flying {len(wps)} waypoints @ alt {args.alt} m for {args.secs:.0f}s…")
        if args.gcs_wait > 0:
            time.sleep(args.gcs_wait)
        pilot = Pilot(lab, alt=args.alt)
        pilot.arm_takeoff(alt=args.alt)
        pilot.goto(wps, alt=args.alt)
        t0 = time.time()
        while time.time() - t0 < args.secs and not pilot.reached_last():
            time.sleep(0.2)
        last = pilot.last or {}
        print(f"  pos=({last.get('x', 0):+.1f},{last.get('y', 0):+.1f}) "
              f"alt={-last.get('z', 0):+.1f}m wp={last.get('wp', 0)}/{len(wps)-1} "
              f"reached={pilot.reached_last()}")
        print("  FC telemetry:",
              ", ".join(f"{k}×{v}" for k, v in sorted(lab.telem_counts.items()))
              or "(none)")
    return 0


def build_parser():
    ap = argparse.ArgumentParser(prog="vayu-headless")
    sub = ap.add_subparsers(dest="sub", required=True)

    s = sub.add_parser("serve", help="boot the persistent session")
    s.add_argument("--alt", type=float, default=-5.0, help="hold altitude, NED z (<0=up)")
    s.add_argument("--gcs-wait", type=float, default=0.0)
    _add_world_opts(s)
    s.set_defaults(func=_cmd_serve)

    d = sub.add_parser("do", help="send a command to a running session")
    d.add_argument("command", nargs=argparse.REMAINDER,
                   help="e.g. fly 8,0\\;8,8 -5 40  |  status  |  land  |  quit")
    d.set_defaults(func=_cmd_do)

    r = sub.add_parser("run", help="one-shot: boot, fly a course, summarise")
    r.add_argument("--alt", type=float, default=-5.0)
    r.add_argument("--box", type=float, default=8.0, help="default-course box side [m]")
    r.add_argument("--course", default="", help='waypoints "N,E;N,E;..."')
    r.add_argument("--secs", type=float, default=40.0)
    r.add_argument("--gcs", action="store_true")
    r.add_argument("--gcs-wait", type=float, default=0.0)
    r.add_argument("--csv", default="")
    _add_world_opts(r)
    r.set_defaults(func=_cmd_run)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
