# Splitting the monorepo into per-product repositories

## Why, and why this is not the usual "monorepos are messy" ask

Two drivers, both of which genuinely need separate repositories:

- **Separate visibility / licensing** — parts are to be shared or published, parts
  kept private. Git has no per-directory access control, so this cannot be done
  inside one repo. This is the driver that makes a split *necessary* rather than
  merely tidy.
- **Independent versioning / release** — firmware, GCS and the protocol shipping
  on their own cadences with their own version numbers.

Nothing else here argues for splitting, and one thing argues hard against a naive
one: **32 % of the last 308 commits touch two or more top-level areas.** Every one
of those becomes a multi-repo change plus a pointer bump. The boundary below is
chosen to cut that number down, not to mirror the current directory names.

## The boundary

The obvious split is `firmware/ | navigator/ | sim/ | navlink/`. **That one is
wrong**, because `sim/` is not one thing:

| | `sim/host` (75 files) | `sim/vsim` (42 files) |
|---|---|---|
| What it is | SITL — the firmware's own host test rig | physics + renderer the GCS drives |
| Firmware coupling | **compiles 59 firmware `.c` files** across 12 subsystems | 4 files reference firmware at all |
| Belongs with | firmware | navigator |

Splitting `sim/` off as its own repo would put a 59-file source dependency across
a repo boundary — the single worst coupling in the tree, made permanent. Splitting
it *down the middle* deletes that coupling instead of paying for it forever.

### Proposed repositories

1. **`vayu-navlink`** — `navlink/`. A protocol dialect plus code generator: 41
   files, 107 commits, no inbound dependencies, three consumers. The natural
   public artifact and the natural *first* extraction. Each consumer pins a tag
   and generates its own codec from the pinned `dialect.json`.

2. **`vayu-firmware`** — `firmware/` **+ `sim/host/`** + the firmware-facing half
   of `tools/` (`tools/telemetry`, `tools/calib`). SITL ships with the firmware
   because it *is* the firmware, compiled for host.

3. **`vayu-navigator`** — `navigator/` (incl. `headless-sdk`) + `sim/vsim/` +
   `tools/autotune`. The likely-private/commercial side.

4. `vtest/` (3 files) goes wherever it is run from, or becomes a trivial fourth
   repo. It is a cross-repo runner after the split, not a component.

`sim/gazebo` (4 files) looks dead — confirm and delete rather than migrate.

## The one real engineering task

`navigator` currently **builds `sim/host` in-process** (`navigator/CMakeLists.txt`
sets `VAYU_SITL_DIR` and `add_subdirectory`s it). Across a repo boundary it must
instead consume a *released* firmware artifact.

The seam already half-exists: `headless-sdk` locates the SITL binary through
`VAYU_SITL_RTOS_BIN` rather than building it. Widen that to the Qt app and the
coupling becomes "navigator runs a published `vayu_sitl_rtos`", which is also
what makes independent release cadence real.

This is the prerequisite for the split, and it is worth doing **before** any repo
is created — it is a normal refactor inside one repo, and impossible to bisect
once it spans three.

## Blockers to clear first

**Licensing — settled: `vayu` stays private, so it needs no license.** Zero
copyright headers in `firmware/src`, `navigator/src`, `sim`; no `LICENSE` file,
and none required while nothing here is published. The two repos that *were*
published carry their own: `navlink` and `vtest` are both Apache-2.0 / NAVRobotec
Pvt Ltd, headers added at extraction. Any future extraction that is to be public
adds its license at extraction time, the same way.

**History carries ~590 MB of binaries.** `.git` is 711 MB against a 120 MB tracked
tree. The largest blobs are datasheets (16 MB BMX160, 9.7 MB F401 reference) and
log captures (15 MB, 3.6 MB, 2.0 MB…). Extract each repo with `git filter-repo`
scoped to its own paths so it inherits only its own history — a naive split gives
three repos that each still clone 711 MB. Decide separately whether datasheets and
`.bin` captures belong in git at all, or in LFS / an assets repo.

**Repo-wide gates fracture.** `trace.py` (req↔code, 174 requirements spanning
firmware + docs), the docs-link gate (199 pages), the coverage ratchet (per-component
floors in one gate), clang-tidy (four databases spanning firmware/sim/navigator),
and `vtest` all assume one tree. Each needs a per-repo home, and the trace gate in
particular needs deciding: requirements currently cross the firmware/navigator line.

## Order

0. **`vtest`** — DONE. Its own public repo, pinned here as a submodule (`v1.3.0`).
   It came out first because the test story blocked on it, ahead of the order below.
1. **Add licensing** — DROPPED for `vayu` (stays private); see the blockers above.
2. **Break the navigator→`sim/host` source dependency**, in-repo, while it is
   still bisectable. ← next
3. **Extract `vayu-navlink`.** DONE. Public repo, Apache-2.0, pinned here as a
   submodule at `v2.0.0`. It proved the pin-and-generate workflow: the submodule
   mounts at the same `navlink/` path the tree already used, so every consumer
   (firmware, navigator, sim/host, tools) needed no change, and the generated
   codec is byte-identical to the vendored one.
4. **Split the gates** to per-repo homes; decide the trace-gate boundary.
5. **Extract `vayu-navigator`** (+ `sim/vsim`, `tools/autotune`).
6. **`vayu-firmware` last** — it is what everything else pins, so it moves when
   the pins are already proven.

## What this costs

Be honest about the tax: a third of commits become multi-repo, `navlink` version
skew becomes possible where today it is structurally impossible (three builds
generate from one file), and skew in that contract has already caused two
hardware bugs — the harness TIME_SYNC gate and the mixer-geometry mismatch. The
version pin plus a CI check that consumers are on the current tag is the price of
admission, not an optional extra.

If the visibility requirement were to go away, none of this would be worth doing.
