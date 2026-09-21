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

| | `sim/host` | `sim/vsim` |
|---|---|---|
| What it is | SITL — the firmware's own host test rig | physics + the `vsim_proto` wire structs |
| Firmware coupling | **compiles 59 firmware `.c` files** across 12 subsystems | none; it is what the SITL links |
| Belongs with | firmware | **firmware** |

An earlier draft of this table said `sim/vsim` was "physics + renderer the GCS
drives" and sent it to navigator. That was wrong on both counts. There is no
renderer in `sim/vsim` — the renderer is `navigator/src/vsim/` (31 files:
ChunkStreamer, SimRendererWidget, procgen…). And `sim/host` *compiles* four of
`sim/vsim`'s `.cpp` files into its physics library, so sending the two halves to
different repos would have made the dependency **circular**.

Both halves ship with the firmware. `vsim_proto.h` moved to `sim/host/sdk/`,
where it is part of the published contract rather than an internal header.

### Proposed repositories

1. **`vayu-navlink`** — `navlink/`. A protocol dialect plus code generator: 41
   files, 107 commits, no inbound dependencies, three consumers. The natural
   public artifact and the natural *first* extraction. Each consumer pins a tag
   and generates its own codec from the pinned `dialect.json`.

2. **`vayu-firmware`** — `firmware/` **+ `sim/host/`** + the firmware-facing half
   of `tools/` (`tools/telemetry`, `tools/calib`). SITL ships with the firmware
   because it *is* the firmware, compiled for host.

3. **`vayu-navigator`** — `navigator/` (incl. `headless-sdk`) + `tools/autotune`.
   The likely-private/commercial side. `sim/vsim` does **not** come here; see the
   boundary table above.

4. `vtest/` (3 files) goes wherever it is run from, or becomes a trivial fourth
   repo. It is a cross-repo runner after the split, not a component.

`sim/gazebo` (4 files) was dead — confirmed and deleted rather than migrated. It
was an alternative physics backend that never got wired up: nothing outside the
plan docs referenced it, and its last substantive change was a path sweep during
the 2026-06 restructure.

## The one real engineering task — DONE

`navigator` **built `sim/host` in-process** and statically linked
`vayu_sitl_rtos_core`, which froze the firmware into the GCS binary at build
time and was the coupling that blocked the split.

The fix was not the one sketched here originally ("widen `VAYU_SITL_RTOS_BIN` to
the Qt app"). That would have turned the in-app sim into a subprocess client and
undone the 2026-07 consolidation, which deleted the `vsim_d` daemon precisely to
get one deterministic in-process stepper. The Qt app *links* the engine; the
headless SDK *runs a binary*. Different couplings, and the looser seam does not
generalise to the tighter one.

Instead the engine is built a third way: **`libvayu_sitl`, a shared module loaded
at runtime**. It exports one symbol, `vayu_sitl_get_api`, returning a vtable
behind a single ABI version gate. Navigator compiles against two headers
(`vayu_sitl_abi.h`, `vsim_proto.h`) and `dlopen`s the rest through `QLibrary`.

That keeps the in-process determinism, and it means navigator needs **no source
dependency on the firmware at all** — not even a submodule pin. The firmware
publishes `vayu-sitl-sdk.tar.gz` (the `.so`, the two headers, a build id); the
GCS consumes it. Independent cadence is then real rather than aspirational.

Two things fell out of doing it:

- Navigator was calling firmware functions directly (mixer geometry, flight-mode
  release, PID apply), bypassing the firmware's own command gates. Those are
  gone; it now sends NavLink frames through `uart2_rx` and the firmware's real
  parser, router and time-sync gate. This is the seam contract, enforced instead
  of asserted.
- Navigator had `sim/host/include` on its include path through a PUBLIC usage
  requirement — a directory of NavHAL port shims that shadow system headers.

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

**Repo-wide gates fracture — measured, and smaller than it looked.** Every gate
was mapped to a side:

| Gate | Spans | Home | Work |
|---|---|---|---|
| build (arm), cppcheck | `firmware/` | firmware | none |
| sitl-tests, sitl-module, sanitizers | `sim/host` | firmware | none |
| coverage ratchet | floors are all `firmware/src` | firmware | none |
| trace gate | `firmware/{src,include,tests}`, `sim/host/tests`, `tools/` | firmware | none |
| clang-tidy | takes build dirs as arguments | both | none — already split |
| navigator build + tests | `navigator/` | navigator | none |
| commit-msgs | commit messages | both | duplicate the job |
| clang-format | whole index | both | takes pathspecs |
| docs-links | 174 pages | both | 17 links cross the line |

**The trace gate does not cross the firmware/navigator line.** That was the
stated worry and it is wrong: `navigator/` carries its own
`docs/reference/requirements.md` and **zero** `@implements`/`@verifies` tags.
Every tag outside `firmware/` is in `sim/host/tests` or `tools/`, both of which
ship with the firmware. `OWNED_ROOTS` needs no change.

**What actually splits is documentation.** 17 doc links cross the
firmware↔navigator boundary and will not resolve once the repos are siblings.
Links into `navlink`/`vtest` are NOT in that count: those are submodules and sit
at the same path either side of the split. `docs_html.py --check` now reports
the sibling crossings and fails if they exceed `MAX_CROSS_REPO_LINKS`, so the
bill is visible and cannot grow by accident. Converting them to URLs is the
cheap fix, but it needs the repo URLs, so it waits for the split itself.

Root-level pages (`ARCHITECTURE.md`, `README.md`, `build.md`) are exempt from
that count and are their own decision: they describe the whole stack, so they
belong either to whichever repo becomes the entry point, or to neither.

## Order

0. **`vtest`** — DONE. Its own public repo, pinned here as a submodule (`v1.3.0`).
   It came out first because the test story blocked on it, ahead of the order below.
1. **Add licensing** — DROPPED for `vayu` (stays private); see the blockers above.
2. **Break the navigator→`sim/host` source dependency.** DONE, via the runtime
   module rather than a subprocess — see "The one real engineering task". The
   GCS binary now contains no firmware symbols.
3. **Extract `vayu-navlink`.** DONE. Public repo, Apache-2.0, pinned here as a
   submodule at `v2.0.0`. It proved the pin-and-generate workflow: the submodule
   mounts at the same `navlink/` path the tree already used, so every consumer
   (firmware, navigator, sim/host, tools) needed no change, and the generated
   codec is byte-identical to the vendored one.
4. **Split the gates.** DONE — every gate is mapped to a side (table above),
   `clang-format` takes pathspecs so one component can be gated alone, and the
   docs gate now ratchets the 17 sibling-crossing links. The trace-gate question
   resolved itself: it never crossed the line. ← the split is now unblocked
5. **Extract `vayu-navigator`** (+ `tools/autotune`). ← next
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
