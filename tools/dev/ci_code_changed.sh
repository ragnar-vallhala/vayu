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
# Did anything that can break a build change between two commits?
#
# `true` unless every changed path is prose. This used to be a `paths-ignore:`
# on ci.yml's trigger, which was cheaper but wrong once "CI required" became a
# required status check: a workflow that does not TRIGGER never reports its
# context, and a required context that never reports blocks the pull request
# forever rather than passing it. A docs-only change was unmergeable.
#
# So the workflow always triggers and the filter moved here. The heavy jobs skip
# on prose, the fan-in still runs, and it already counts skipped as success --
# which is precisely what that rule was for.
#
#   usage: ci_code_changed.sh <base-ref> <head-ref>
#   prints: true | false
set -uo pipefail

if [ "$#" -ne 2 ]; then
  echo "usage: $0 <base-ref> <head-ref>" >&2
  exit 2
fi

# Prose. Kept in ONE place so the list cannot drift between the push and
# pull_request paths the way the duplicated paths-ignore blocks did.
PROSE='(\.md$|(^|/)docs/)'

changed="$(git diff --name-only "$1" "$2" --)" || exit 2

if [ -z "$changed" ]; then
  # No diff at all. Nothing to build, but say true rather than guess: an empty
  # diff here means the refs are not what we think, and skipping the suite on a
  # bad assumption is the expensive mistake, not running it.
  echo true
  exit 0
fi

if printf '%s\n' "$changed" | grep -qvE "$PROSE"; then
  echo true
else
  echo false
fi
