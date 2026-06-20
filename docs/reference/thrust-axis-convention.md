# Thrust-Axis Convention (Standard)

Status: **normative**. This is the single source of truth for what a motor's
thrust axis means and which way it points. Every component that touches motor
geometry — the Vehicle-tab editor, the persisted config, the `vsim_d` physics,
and the firmware mixer — MUST obey it. Violations cause silent roll/pitch
inversion (positive feedback → "throw-off" on excitation) and/or a craft that
thrusts into the ground instead of lifting.

See also [coordinate_ref.md](coordinate_ref.md) for the frame definitions this
builds on.

## 1. Frames (recap)

- **Body frame: FRD** — `+X` Front, `+Y` Right, `+Z` Down. Right-handed.
- **World frame: NED** — `+Z` is Down; gravity is `+Z` (`physics_core.cpp`,
  `a += (0,0,gravity)`).
- "Up" (lift) is **`-Z`** in both frames.
- Sign table (from `coordinate_ref.md`): nose-up `+pitch`, right-wing-down
  `+roll`, turn-right `+yaw`.

## 2. Definition of the thrust axis

Each motor `i` has a **unit thrust axis `aᵢ`**, a vector in the **body frame**
pointing **in the direction the rotor pushes the airframe** (the direction of
the thrust *force*, not the prop's spin axis ambiguity).

The body thrust force is:

```
F_body = Σ aᵢ · |thrustᵢ|         with |thrustᵢ| = k_thrust · ωᵢ²  ≥ 0
```

(`motor_model.cpp`). Because thrust magnitude is non-negative, **the axis sign
carries the direction**.

## 3. The canonical axis: `(0, 0, -1)`

For a normal lifting multirotor, every motor's thrust axis is:

```
aᵢ = (0, 0, -1)        // body -Z = up
```

A level craft then pushes itself along `-Z` (up), opposing `+Z` gravity. This is
the editor's spinbox default (`GeometryEditorWidget.cpp`, `az` default `-1`).

**Invariant:** for a lifting multirotor, **`aᵢ.z < 0`** (≈ `-1`). An axis with
`aᵢ.z ≥ 0` points the thrust down/sideways — the craft cannot hover and the
attitude loop inverts. This MUST be rejected or warned on at edit time.

## 4. Derived torque (why the sign matters)

With `aᵢ = (0,0,-1)`, `Fᵢ = (0,0,-Tᵢ)` and the body torque is:

```
τ = Σ rᵢ × Fᵢ  −  Σ aᵢ · (k_moment · ωᵢ² · spinᵢ)

  roll   τ_x = −Σ yᵢ·Tᵢ      → +roll (right wing down) ⇒ more thrust where y<0 (left)
  pitch  τ_y = +Σ xᵢ·Tᵢ      → +pitch (nose up)        ⇒ more thrust where x>0 (front)
  yaw    τ_z = +Σ km·ωᵢ²·spinᵢ → +yaw (turn right)      ⇒ +spin
```

**Flipping `aᵢ.z` to `+1` flips `τ_x` and `τ_y`** (the `rᵢ × Fᵢ` term) — roll and
pitch invert, while the mixer (below) does not, so the loop becomes positive
feedback. This is the exact "violent rotation on excitation" failure.

## 5. The canonical mixer (must match §4)

For the standard X-quad with `aᵢ = (0,0,-1)` the per-motor mix signs are:

```
outᵢ = throttle + roll·(−sign(yᵢ)) + pitch·(+sign(xᵢ)) + yaw·(+sign(spinᵢ))
```

This is consistent with §4 **only for `aᵢ.z < 0`**. The firmware mixer
(`angle_rate_controller_set_motor_geometry`) currently derives these signs from
`sign(posᵢ)` and `sign(spinᵢ)` **and ignores `aᵢ` entirely** — i.e. it hard-
assumes `aᵢ.z < 0`. That assumption is the bug surface.

## 6. Single-source-of-truth rules

1. **One vector, no per-stage negation.** The same `aᵢ` flows editor → persisted
   config → `vsim_d` (`VSIM_CTL_SET_GEOMETRY`) → firmware
   (`CMD_SET_MOTOR_GEOMETRY`). No component may negate or re-interpret it.
2. **Editor transforms must preserve §3.** `GeometryEditorWidget` applies the
   body transform to the axis (`m.axis = xi.mapVector(axis)`) when a motor is
   moved/rotated with the 3D tool — a body rotation can flip `aᵢ.z`. Edits MUST
   re-assert the §3 invariant (clamp/normalize so `aᵢ.z < 0`, or warn).
3. **Firmware mixer MUST be axis-aware.** Derive the roll/pitch mix sign
   including `sign(aᵢ.z)` (equivalently: fold `sign(aᵢ.z)` into the roll & pitch
   terms), so a non-canonical axis stays consistent with the physics instead of
   silently inverting. Yaw already follows `aᵢ` via the reaction term.

## 7. Conformance checklist

| Component | Rule | Status |
| :-- | :-- | :-- |
| `vsim_d` physics | `F=Σaᵢ·\|T\|`, `τ=Σr×F − a·km·ω²·spin` | ✅ conforms |
| Editor default | `az = -1` | ✅ conforms |
| Editor edits | re-assert `aᵢ.z < 0` after transform; warn otherwise | ❌ TODO (can flip silently) |
| Persisted config | store `aᵢ` verbatim | ✅ (but can hold a bad `+1`) |
| Firmware mixer | roll/pitch signs include `sign(aᵢ.z)` | ❌ TODO (ignores `aᵢ`) |

## 8. Immediate remediation

- Any saved airframe with `m*_az = +1` (e.g. the current `Vayu GCS.conf`) is
  **non-conformant**: set every motor axis to `(0,0,-1)` and re-tune.
- Apply the two ❌ fixes above so a flipped axis can't recur silently.
