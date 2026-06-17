"""Thin client: send one command to a running `serve` session, print the reply."""
import socket
import sys

from . import paths


def send_command(line, sock_path=None, timeout=120.0):
    """Connect, send one command line, return the reply string (no trailing NL).
    Raises ConnectionError if no session is listening."""
    sock_path = sock_path or paths.SOCK_PATH
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect(sock_path)
    except (FileNotFoundError, ConnectionRefusedError) as e:
        raise ConnectionError(f"no session at {sock_path}") from e
    try:
        s.sendall((line + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
    finally:
        s.close()
    return buf.decode(errors="replace").strip()


def main(line, sock_path=None):
    try:
        print(send_command(line, sock_path))
        return 0
    except ConnectionError as e:
        print(f"err: {e} — start one with: vayu-headless serve", file=sys.stderr)
        return 1
