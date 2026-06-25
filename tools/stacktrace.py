import os
import subprocess
import sys

# ELF defaults to <repo>/build/main (this script lives in <repo>/tools/).
# Override with the VAYU_ELF env var or a path argument:  stacktrace.py [elf].
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ELF_FILE = (
    sys.argv[1]
    if len(sys.argv) > 1
    else os.environ.get("VAYU_ELF", os.path.join(_REPO_ROOT, "build", "main"))
)

# Paste your addresses here
addresses = [
 "0x20003FE0",
 "0x20017FE0",
 "0x20017FE0",
 "0x20017FE0",
 "0x60000000",
 "0x01500801",
 "0x08002B13",
 "0x08000100",
 "0x0801529C",
 "0x08001433"
]

def resolve_address(addr):
    try:
        result = subprocess.run(
            ["arm-none-eabi-addr2line", "-e", ELF_FILE, "-f", "-C", addr],
            capture_output=True,
            text=True
        )
        return result.stdout.strip()
    except Exception as e:
        return f"Error resolving {addr}: {e}"

def main():
    print("=== Backtrace Symbolization ===\n")
    for addr in addresses:
        print(f"{addr} ->")
        print(resolve_address(addr))
        print()

if __name__ == "__main__":
    main()