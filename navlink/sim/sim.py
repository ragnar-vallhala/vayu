#!/usr/bin/env python3
"""NavLink v2 link simulator.

Spawns two host processes — a flight-controller model and a ground-station model —
that talk NavLink v2 over UDP, with a configurable impairment layer between them
(latency, jitter, loss, duplication, reorder, bit-corruption, bandwidth cap).
The GCS reports real-world link stats: rates, throughput, CRC errors, seq-gap
loss estimate, reorders, PING RTT, and command-ACK latency.

Examples:
    python3 sim.py                                   # clean link, 10 s
    python3 sim.py --scenario telemetry_radio
    python3 sim.py --scenario lossy --duration 20
    python3 sim.py --latency-ms 50 --jitter-ms 20 --loss 0.05 --corrupt 0.01
    python3 sim.py --scenario satellite --json       # machine-readable output
"""
import argparse
import json
import os
import socket
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ENDPOINT = os.path.join(HERE, "endpoint.py")

# Preset link conditions (knobs map to link.LinkConfig).
SCENARIOS = {
    "clean":           {},
    "wifi":            {"latency_ms": 5,  "jitter_ms": 3,  "loss": 0.001},
    "telemetry_radio": {"latency_ms": 40, "jitter_ms": 15, "loss": 0.02,
                        "reorder": 0.01, "rate_bytes": 4000},
    "lossy":           {"latency_ms": 80, "jitter_ms": 40, "loss": 0.10,
                        "dup": 0.02, "reorder": 0.05, "corrupt": 0.01},
    "satellite":       {"latency_ms": 600, "jitter_ms": 50, "loss": 0.03},
}


def free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def build_config(args):
    cfg = dict(SCENARIOS.get(args.scenario, {}))
    for k in ("latency_ms", "jitter_ms", "loss", "dup", "reorder", "corrupt", "rate_bytes"):
        v = getattr(args, k)            # argparse maps --latency-ms → latency_ms, etc.
        if v is not None:
            cfg[k] = v
    cfg["duration"] = args.duration
    cfg["seed"] = args.seed
    return cfg


def spawn(role, listen, peer, cfg):
    return subprocess.Popen(
        [sys.executable, ENDPOINT, "--role", role, "--listen", str(listen),
         "--peer", str(peer), "--config", json.dumps(cfg)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def parse_report(out):
    for line in out.splitlines():
        if line.startswith("REPORT "):
            return json.loads(line[len("REPORT "):])
    return None


def human(cfg, fc, gcs):
    L = []
    p = L.append
    knobs = {k: cfg[k] for k in cfg if k not in ("duration", "seed") and cfg[k]}
    p("─" * 60)
    p(f"NavLink v2 link simulation · {cfg['duration']}s · seed {cfg['seed']}")
    p(f"link: {knobs or 'clean'}")
    p("─" * 60)
    if fc:
        e = fc["egress"]
        p(f"FC  → telemetry {fc['tx_frames']}; egress transmitted {e['sent']} "
          f"(dropped {e['dropped']}, dup {e['duped']}, corrupted {e['corrupted']})")
        p(f"     answered {fc['cmds_acked']} commands, echoed {fc['pings_echoed']} pings")
    if gcs:
        p(f"GCS ← {gcs['rx_frames']} frames / {gcs['rx_bytes']} B over {gcs['rx_span_s']}s")
        p(f"     {gcs['rx_rate_hz']} Hz, {gcs['rx_throughput_Bps']} B/s")
        loss_line = f"     est loss {gcs['est_loss_pct']}% ({gcs['lost_est']} frames), reorders {gcs['reorders']}"
        if fc:                       # compare GCS estimate to the link's actual drops
            truth = fc["egress"]["dropped"] + fc["egress"]["corrupted"]
            loss_line += f"   [link truth: {truth} dropped+corrupted on egress]"
        p(loss_line)
        p(f"     CRC errors {gcs['crc_errors']}, undecodable {gcs['undecodable']}, "
          f"non-frame {gcs['non_frame']}")
        pg = gcs["ping"]
        if pg["count"]:
            p(f"     PING RTT min/avg/max {pg['min_ms']}/{pg['avg_ms']}/{pg['max_ms']} ms "
              f"(lost {pg['lost']}/{pg['sent']})")
        else:
            p(f"     PING: 0/{pg['sent']} returned")
        cm = gcs["cmd"]
        p(f"     CMD ack {cm['acked']}/{cm['sent']} ({cm['ack_pct']}%), "
          f"ack latency avg {cm['ack_lat_ms_avg']} ms")
        if gcs["by_msgid"]:
            top = sorted(gcs["by_msgid"].items(), key=lambda kv: -kv[1])
            p("     by msgid: " + ", ".join(f"{n}={c}" for n, c in top))
    p("─" * 60)
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", choices=list(SCENARIOS), default="clean")
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--latency-ms", type=float)
    ap.add_argument("--jitter-ms", type=float)
    ap.add_argument("--loss", type=float, help="P(drop), 0..1")
    ap.add_argument("--dup", type=float, help="P(duplicate), 0..1")
    ap.add_argument("--reorder", type=float, help="P(reorder), 0..1")
    ap.add_argument("--corrupt", type=float, help="P(bit-flip), 0..1")
    ap.add_argument("--rate-bytes", type=float, help="egress cap (B/s); 0=unlimited")
    ap.add_argument("--json", action="store_true", help="emit combined JSON instead of a table")
    args = ap.parse_args()

    cfg = build_config(args)
    fc_port, gcs_port = free_port(), free_port()

    fc_proc = spawn("fc", fc_port, gcs_port, cfg)
    gcs_proc = spawn("gcs", gcs_port, fc_port, cfg)

    timeout = cfg["duration"] + 15.0
    try:
        fc_out, fc_err = fc_proc.communicate(timeout=timeout)
        gcs_out, gcs_err = gcs_proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        fc_proc.kill()
        gcs_proc.kill()
        sys.exit("endpoint timed out")

    fc_rep, gcs_rep = parse_report(fc_out), parse_report(gcs_out)
    for tag, err in (("fc", fc_err), ("gcs", gcs_err)):
        if err.strip():
            sys.stderr.write(f"[{tag} stderr]\n{err}\n")

    if args.json:
        print(json.dumps({"config": cfg, "fc": fc_rep, "gcs": gcs_rep}, indent=2))
    else:
        print(human(cfg, fc_rep, gcs_rep))

    if fc_rep is None or gcs_rep is None:
        sys.exit("missing endpoint report (see stderr)")


if __name__ == "__main__":
    main()
