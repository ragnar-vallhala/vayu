#!/usr/bin/env python3
"""DEPRECATED — superseded by the `vayu-headless` CLI (vayu_headless.cli).

The headless harness now lives in the installable package at
navigator/headless-sdk (see its README/PLAN). This file is a thin forwarder that
translates the old flags to the new subcommands so existing
`sitl_lab.py --serve` / `--do ...` invocations keep working. No logic lives
here anymore; prefer `vayu-headless serve|do|run` directly.
"""
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_ROOT, "software", "headless-sdk"))
from vayu_headless import cli  # noqa: E402

# legacy flags that carry a value, forwarded verbatim to serve/run.
_PASS = {"--alt", "--conf", "--gcs-wait", "--turb", "--course", "--box", "--secs"}


def _translate(argv):
    """Map the legacy flag set to a `vayu-headless` subcommand argv."""
    if "--do" in argv:
        return ["do"] + argv[argv.index("--do") + 1:]
    serve = "--serve" in argv
    out = ["serve"] if serve else ["run"]
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in _PASS and i + 1 < len(argv):
            # serve doesn't take --course/--box/--secs; drop those for serve
            if not (serve and a in {"--course", "--box", "--secs"}):
                out += [a, argv[i + 1]]
            i += 2
            continue
        if a == "--wind" and i + 3 < len(argv):
            out += [a, argv[i + 1], argv[i + 2], argv[i + 3]]
            i += 4
            continue
        if a == "--gcs" and not serve:
            out += [a]
        i += 1
    return out


def main():
    sys.stderr.write("sitl_lab.py is DEPRECATED — use `vayu-headless`. Forwarding…\n")
    return cli.main(_translate(sys.argv[1:]))


if __name__ == "__main__":
    raise SystemExit(main())
