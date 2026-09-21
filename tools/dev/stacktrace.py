# Copyright (C) 2026 NAVRobotec Pvt Ltd
# Author: Ragnar Vallhala
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import subprocess
import sys

# ELF defaults to <repo>/build/main (this script lives in <repo>/tools/).
# Override with the VAYU_ELF env var or a path argument:  stacktrace.py [elf].
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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