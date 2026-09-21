#!/usr/bin/env bash
# Format the tree, or check it (--check, what CI runs).
#
# The scope is the fiddly part and it is why this is a script rather than a
# command in a comment: CI and a developer must exclude the same things, and
# a bare `git ls-files | xargs clang-format` excludes neither.
#
#   firmware/docs/  the log-analysis journals archive source snapshots that are
#                   EVIDENCE of what a file looked like when it was analysed.
#                   Reformatting them rewrites the record.
#   extern/         vendored; a submodule, so not in this index anyway.
#
# A .clang-format-ignore would be the tidy way to say this, but the pip
# clang-format wheel (22.1.1) silently ignores that file -- verified, including
# in the target's own directory -- so the exclusion lives here.
#
# Version: the tree is clean under both apt clang-format-18 (what CI pins) and
# the 22.1.1 wheel, measured over all 453 files. $CLANG_FORMAT overrides.
#
# Scope can be narrowed with trailing pathspecs:
#   run_clang_format.sh --check firmware sim
# so one component can be gated on its own. With none it takes the whole index,
# which is what the monorepo does and what each repo will do after the split --
# the same script, unchanged, in either shape.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
FMT="${CLANG_FORMAT:-clang-format}"

CHECK=0
if [ "${1:-}" = "--check" ]; then CHECK=1; shift; fi

# Trailing args are pathspecs; none means the whole index.
mapfile -t files < <(git ls-files ':!:extern/' ':!:firmware/docs/' "$@" \
                     | grep -E '\.(c|h|cpp|hpp|cc)$')

if [ ${#files[@]} -eq 0 ]; then
  # A pathspec that matches nothing is a typo, not a clean tree. Saying "0
  # files, all good" would make a mis-scoped gate look green forever.
  echo "run_clang_format.sh: no sources matched${*:+ for: $*}" >&2
  exit 1
fi

if [ "$CHECK" = 1 ]; then
  # --dry-run -Werror reports and exits non-zero; names the files, not the diff.
  if ! printf '%s\n' "${files[@]}" \
       | xargs -P"$(nproc)" -I{} "$FMT" --dry-run -Werror {} 2>&1 \
       | grep -oE '^[^ ]+\.(c|h|cpp|hpp|cc):' | sort -u | sed 's/:$//' > /tmp/$$.bad; then
    :
  fi
  if [ -s /tmp/$$.bad ]; then
    echo "not formatted ($(wc -l < /tmp/$$.bad) files) -- run tools/dev/run_clang_format.sh" >&2
    cat /tmp/$$.bad >&2
    rm -f /tmp/$$.bad
    exit 1
  fi
  rm -f /tmp/$$.bad
  echo "clang-format: ${#files[@]} files clean"
else
  printf '%s\n' "${files[@]}" | xargs -P"$(nproc)" -I{} "$FMT" -i {}
  echo "clang-format: formatted ${#files[@]} files"
fi
