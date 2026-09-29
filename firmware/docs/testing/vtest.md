# Running Vayu's suites with an installed vtest

vtest can now be installed once per machine instead of compiled by every repo.
This page is the change `tools/scripts/vayu.sh` needs to use it: **an installed
`vtest` if there is one, otherwise build the pinned `vtest/` submodule exactly
as today.** The root `vtest.conf` already loads under the current vtest.

## Install vtest (once per machine)

```sh
git clone https://github.com/ragnar-vallhala/vtest.git ~/src/vtest   # or use vtest/
cd ~/src/vtest
cmake -S . -B build && cmake --build build && cmake --install build --prefix ~/.local
vtest --version          # e.g. vtest v1.3.0-7-g7f22748
```

This puts `vtest` and `vtest-loc` (vtest's `loc.sh`) in `~/.local/bin`, which
must be on `PATH`. Re-run the same three commands after pulling vtest.

## `tools/scripts/vayu.sh`

`build_vtest` becomes "find a vtest": it sets `VTEST` (the binary) and
`VTEST_LOC` (the line counter) instead of always compiling.

```bash
# An installed vtest if there is one (VTEST=path forces a specific binary);
# otherwise build the pinned submodule.
build_vtest() {
  if [ -n "${VTEST:-}" ]; then
    :
  elif command -v vtest >/dev/null 2>&1; then
    VTEST=vtest
    VTEST_LOC=vtest-loc
    say "vtest -> $(command -v vtest) ($(vtest --version))"
  else
    say "vtest -> build_vtest/vtest (from the vtest/ submodule)"
    if [ ! -f vtest/vtest.c ]; then
      die "no vtest installed and vtest/ is empty -- install vtest" \
          "(firmware/docs/testing/vtest.md) or: git submodule update --init vtest"
    fi
    mkdir -p build_vtest
    cc -std=c11 -O2 -Wall -Wextra vtest/vtest.c -o build_vtest/vtest
    VTEST=./build_vtest/vtest
    VTEST_LOC=vtest/loc.sh
  fi
  export VTEST_LOC="${VTEST_LOC:-vtest-loc}"
}
```

And the last line of `run_vtest` runs whichever it found:

```bash
  if [ -t 1 ]; then "$VTEST"; else "$VTEST" --run; fi
```

`vayu.sh build vtest` keeps working: with vtest installed it just reports which
one it found.

## `vtest.conf`: the `[loc]` check

`[loc]` calls `vtest/loc.sh`, which is missing when the submodule was never
initialised -- exactly the case an installed vtest is for:

```ini
[loc]
adapter = check
cmd     = ${VTEST_LOC:-vtest-loc} firmware sim navlink tools
```

## The other vtest in this tree

`navlink/` is a submodule with its own `vtest/` pin and its own
`scripts/vtest.sh`. It gets the same change in the navlink repo; bumping the
navlink pin here brings it in. `tools/dev/run_clang_tidy.sh` lints
`vtest/vtest.c` -- keep that line while the `vtest/` submodule stays.

## Which vtest am I running?

```sh
command -v vtest && vtest --version            # the installed one
VTEST=./build_vtest/vtest tools/scripts/vayu.sh test   # force the submodule build
```

CI has no installed vtest, so it takes the submodule branch: the `vtest/` pin
stays the version CI tests against. A local installed vtest newer than the pin
behaves differently in these ways, all deliberate:

- A suite whose runner crashes, or exits non-zero while every parsed case
  passed, is **FAILED** (older vtest could print ALL PASSED).
- Ctrl-C (and a CI cancel) kills the running suite instead of leaving it
  running after vtest exits.
- An unknown key or adapter in `vtest.conf` is an error, so an older vtest
  reading a newer conf says so instead of guessing.
