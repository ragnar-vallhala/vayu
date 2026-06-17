"""vayu-headless CLI entrypoint.

Phase 0: stub so `pip install -e` resolves the console script. The real
serve/do/run subcommands land in Phase 2 (see PLAN.md §7).
"""
import sys


def main(argv=None):
    sys.stderr.write(
        "vayu-headless: CLI not implemented yet (Phase 2). "
        "Use tools/sim_host/sitl_lab.py for now.\n")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
