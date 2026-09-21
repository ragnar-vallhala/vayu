#!/usr/bin/env python3
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
"""Download-only pull of the on-target gcov dump (0:cov.gcd) over the xfer
FILE provider, reusing fs_xfer_udp_test's Bridge/discover/download. Saves it as
cov.gcda (host-side name) ready for arm-none-eabi-gcov-tool merge-stream."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fs_xfer_udp_test as X  # noqa: E402

PATH = "0:cov.gcd"
OUT = os.path.join(os.path.dirname(os.path.dirname(HERE)),
                   "firmware", "build_hwtest_cov", "cov.gcda")
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 14555

br = X.Bridge(PORT)
if not X.discover_and_sync(br):
    print("discover/sync FAILED (bridge powered + on net?)", file=sys.stderr)
    sys.exit(2)

X.fs_probe(br, PATH)
rx, kbps = X.download(br, 1, PATH, 65536)
if rx is None:
    print("download FAILED", file=sys.stderr)
    sys.exit(1)

with open(OUT, "wb") as f:
    f.write(rx)
print(f"[ok] pulled {len(rx)} bytes @ {kbps:.1f} KiB/s -> {OUT}")
