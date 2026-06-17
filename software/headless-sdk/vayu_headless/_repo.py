"""Locate the Vayu repo root (the dir containing navlink/, tools/, software/).

Shared by the modules that need to resolve binary/codec paths. Phase 2's
paths.py will build the full path-resolution policy on top of this.
"""
import os


def repo_root():
    env = os.environ.get("VAYU_REPO_ROOT")
    if env and os.path.isdir(os.path.join(env, "navlink")):
        return env
    d = os.path.abspath(os.path.dirname(__file__))
    while True:
        if os.path.isdir(os.path.join(d, "navlink")) and \
           os.path.isdir(os.path.join(d, "tools")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            raise RuntimeError("could not locate Vayu repo root")
        d = parent
