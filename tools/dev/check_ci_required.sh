#!/usr/bin/env bash
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
#
# Every job in a workflow must be reachable from that workflow's fan-in job.
#
# Branch protection requires the fan-in contexts ("CI required", "Gates
# required") instead of a dozen literal job names, because literal names drop
# out of enforcement silently when a job is renamed. The fan-in closes that --
# a `needs` entry pointing at a job that no longer exists is a workflow syntax
# error, so a rename cannot pass unnoticed.
#
# It does NOT close the other direction. A job ADDED to a workflow and not added
# to `needs` runs, reports, and is simply not required: a gate that looks
# enforced on the PR page and is not. This is the check for that, and it belongs
# to the fan-in job itself -- the aggregator verifies its own coverage before
# claiming the workflow is green.
set -uo pipefail

rc=0
for wf in .github/workflows/*.yml; do
  mapfile -t out < <(python3 - "$wf" <<'PY'
import re, sys
path = sys.argv[1]
text = open(path).read()
m = re.search(r"^jobs:\s*$", text, re.M)
if not m:
    sys.exit(0)
body = text[m.end():]
jobs = [j.group(1) for j in re.finditer(r"^  ([A-Za-z0-9_-]+):\s*$", body, re.M)]
if "required" not in jobs:
    print("NOFANIN")
    sys.exit(0)
seg = body[re.search(r"^  required:\s*$", body, re.M).end():]
nx = re.search(r"^  [A-Za-z0-9_-]+:\s*$", seg, re.M)
seg = seg[: nx.start()] if nx else seg
nm = re.search(r"^    needs:\s*$((?:\n\s+-\s+\S+)+)", seg, re.M)
needs = re.findall(r"-\s+(\S+)", nm.group(1)) if nm else []
for j in jobs:
    if j != "required" and j not in needs:
        print("MISSING", j)
PY
)
  name="${wf##*/}"
  if [ "${out[0]:-}" = "NOFANIN" ]; then
    printf '  %-16s %s\n' "$name" "no fan-in job -- nothing to verify"
    continue
  fi
  if [ "${#out[@]}" -eq 0 ]; then
    printf '  %-16s every job is required\n' "$name"
  else
    for line in "${out[@]}"; do
      printf '  %-16s NOT REQUIRED: %s\n' "$name" "${line#MISSING }"
    done
    rc=1
  fi
done

if [ "$rc" != 0 ]; then
  cat >&2 <<'MSG'

A job runs in CI but is not required to pass.

Branch protection requires the fan-in context, so a job missing from its
`needs:` list reports on the pull request and blocks nothing. Add it to the
`required` job's `needs:` in the same workflow -- that is the whole list of
what protection enforces.
MSG
  exit 1
fi
echo "ci-required: every job is covered by its workflow's fan-in"
