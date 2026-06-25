# `docs/store/rig_tune.json` — Why a Separate Motor Geometry?

**What this is:** analysis of the `docs/store/*.json` tune store, specifically why
the tune carries a `motor_geometry` block that must be pushed to the FC
separately, and how that geometry differs from the firmware core's compiled-in
mix. Generated 2026-06-23 against the current tree.

## The file

There is one file, `docs/store/rig_tune.json` — an **operator artifact, not
firmware**. It is the working on-hardware tune (captured 2026-06-22) for the
X-quad-on-a-free-rotating-rig, re-applied each session via
`tools/sysid/apply_tune.py` over NavLink/UDP. It bundles four things:

- `motor_geometry` — pos_x/pos_y/spin sign arrays (the subject here)
- `rate_pid` — inner rate-loop gains (controller=1)
- `angle_pid` — outer angle-loop gains (controller=0)
- `identified_plant` — the sysid fit (K, tau, bandwidth, R²) — **documentation
  only, never transmitted**

## What the geometry block actually is

```json
"motor_geometry": { "pos_x":[1,-1,-1,1], "pos_y":[1,1,-1,-1], "spin":[-1,1,-1,1] }
```

Send path: `apply_tune.py` → `CmdSetMotorGeometry` →
`navlink_router.c:on_cmd_set_motor_geometry` →
`angle_rate_controller_apply_geometry_command` →
`angle_rate_controller_set_motor_geometry`.

The firmware reads **only the sign** of each value
(`src/control/angle_rate_controller.c:103-105`):

```c
s_mix_roll[i]  = (pos_y[i] >= 0) ? -1 : +1;   /* -sign(y) */
s_mix_pitch[i] = (pos_x[i] >= 0) ? +1 : -1;   /*  sign(x) */
s_mix_yaw[i]   = (spin[i]  >= 0) ? +1 : -1;   /*  spin    */
```

So the `1.0` magnitudes are irrelevant — the JSON really carries **12 sign bits**.
They become the per-motor mix used at `angle_rate_controller.c:351`:

```
out_i = throttle + roll·s_mix_roll[i] + pitch·s_mix_pitch[i] + yaw·s_mix_yaw[i]
```

## How it differs from the FC core

The core has a **compiled-in default** mix (`angle_rate_controller.c:95-97`) —
the legacy X-quad (FR=M1, RR=M2, RL=M3, FL=M4):

| sign array | core default | this JSON produces | diff |
|---|---|---|---|
| `s_mix_roll`  | `{-1,-1,+1,+1}` | `{-1,-1,+1,+1}` | **identical** |
| `s_mix_pitch` | `{+1,-1,-1,+1}` | `{+1,-1,-1,+1}` | **identical** |
| `s_mix_yaw`   | `{+1,-1,+1,-1}` | `{-1,+1,-1,+1}` | **fully negated** |

The geometry's *only* net effect on the core is to **flip the yaw mix sign**.
Roll/pitch are already what the core does by default. This matches the JSON's own
comment: the rig's real motor spins (M1,M3 CW / M2,M4 CCW) are **inverted vs the
firmware's assumed default spin convention**, so the yaw loop closed with
*positive* feedback → yaw runaway. Pushing the geometry corrects the sign at
runtime.

## Why it must be passed separately

Two independent reasons:

1. **Not baked into the binary for this airframe.** The compiled default is
   deliberately a generic legacy X-quad. The design intent
   (`angle_rate_controller.c:87-94`) is that **one external geometry source** —
   the GCS vehicle / loaded `.vveh`, or this tune file — drives *both* the sim
   physics and the firmware mix, so firmware + sim stay consistent for any
   layout. The rig's spin convention isn't the compiled default, so it must be
   overridden.

2. **The FC does not persist geometry.** `on_cmd_set_motor_geometry` only writes
   the RAM `s_mix_*` arrays — **no SD write**. (Contrast `on_cmd_set_pid` /
   `pid_config`, which *does* persist to `0:pid.bin`.) So every power cycle the
   mix reverts to the backwards-yaw compiled default. The file's top comment
   ("NOTHING on the FC persists across power cycles … re-apply after every
   battery cycle") reflects this — `apply_tune.py` re-pushes geometry **first**,
   then the gains, on each boot.

## Relation to the SITL stub seam

Same seam as the mixer-frame issue: the mix signs must be fed the
**physics-frame** geometry, and the command only applies after the §10.5
TIME_SYNC handshake — which is why `apply_tune.py:96` sends `TimeSync` before any
`CmdSetMotorGeometry` / `CmdSetPid`. In SITL the geometry arrives from the GCS
vehicle / `.vveh` over the same NavLink path; on this hardware rig it arrives
from `rig_tune.json`. Same command, same RAM-only override, different source.

## One-line answer

`rig_tune.json` carries a per-airframe geometry because the FC core ships a
generic, RAM-only default mix whose spin/yaw signs are backwards for this rig —
geometry has no persistence, so it must be re-pushed (sign-only, a yaw-flip in
this case) over NavLink on every boot before flight.

## References
- `docs/store/rig_tune.json`
- `src/control/angle_rate_controller.c:87-123` (default mix + geometry setter)
- `src/comm/navlink_router.c:225-239` (CMD_SET_MOTOR_GEOMETRY handler)
- `tools/sysid/apply_tune.py` (re-apply workflow)
- Sibling report: `docs/scratch/sitl-fc-stub-inventory.md`
- Memory: mixer-frame-vs-physics-frame, onhw-tune-logreport, onhw-sysid-phase0
