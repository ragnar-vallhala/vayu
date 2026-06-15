"""Shared bootstrap for the NavLink simulator: locate (and if needed generate)
the Python codec, and import it once."""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                       # navlink/
GEN_PY = os.path.join(ROOT, "generated", "python")


def load_nl():
    """Import the generated navlink_msgs module, regenerating it if absent."""
    if not os.path.exists(os.path.join(GEN_PY, "navlink_msgs.py")):
        subprocess.run([sys.executable, os.path.join(ROOT, "generate.py")], check=True)
    if GEN_PY not in sys.path:
        sys.path.insert(0, GEN_PY)
    import navlink_msgs
    return navlink_msgs
