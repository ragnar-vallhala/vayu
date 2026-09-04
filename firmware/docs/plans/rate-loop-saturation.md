# Rate-loop saturation — why the airframe flies away, and how the reference autopilots stop it

## Context

On 2026-09-04 the first flight of the ch6 height mode ended with the airframe
climbing fast into the ceiling and breaking several props. The height controller
was not the cause: it commands the **collective**, and under saturation the
collective is no longer the height loop's to give. This plan records the
mechanism, what PX4 and ArduPilot do about it, and the fix order.

This is a design plan. No code is written here. File:line references are pointers
to the tree at the time of writing, not contracts.

**Blocking:** the ch6 height mode
([`altitude-hold-and-in-air-plan.md`](altitude-hold-and-in-air-plan.md)) must not
be flown again until at least items 1 and 2 of §4 land.

---

## 1. The budget does not balance

The rate PID's output per axis can reach **P + `I_MAX` + `D_MAX`**. With the
compiled defaults (`variables.h`): `I_MAX = 0.2`, `D_MAX = 0.25`, and P
contributing ~0.3 at a large rate error, that is **~0.75 in mixer units per
axis**, and three axes share one motor set.

Available room at hover is about **0.35** — motors sit near 0.5, the idle floor
is `MOTOR_IDLE_FLOOR = 0.15`, the ceiling is 1.0. A single axis can therefore
demand roughly **twice** what the airframe can deliver. `angle_rate_controller.c`
already records this as a known property: *"vayu's budget is ~6x its headroom"*.

Saturation is not an edge case here. It is the designed-in steady state for any
substantial disturbance.

### Measured, on the real FC (2026-09-04, armed, height mode engaged, on the bench)

```
MotorTelemetry.cmd peaks = [1.00, 0.88, 0.40, 0.53]
```

One motor pinned at full with a wide asymmetric spread, while the airframe sat
still. That is an attitude fight, not a collective command.

### Reproduced in SITL, on clean HEAD with the feature stashed

```
cmd=[0.15, 1.0, 1.0, 0.15]  ->  [1.0, 0.15, 0.15, 1.0]  (flipping every sample)
```

Pure pitch under `s_mix_pitch = {+1,-1,-1,+1}`. Reported roll/pitch stay at
~0.0° throughout — **an attitude-only check looks healthy**; the tell is in the
motor commands. Altitude ran away to 157 m in 3.3 s at any commanded throttle
(157.05 m on clean HEAD vs 157.31 m on the feature branch — identical, so this is
not a regression from recent work). See
`memory/sitl-pitch-limit-cycle-blocks-testing.md`.

---

## 2. Why saturation becomes a flyaway

`MIXER_AIRMODE_RP` resolves an impossible request by **shifting collective** to
preserve roll/pitch (`mixer.c:147-156`, `motor[i] += kt * dthr[i]`), then by
sacrificing yaw. Roll/pitch are never scaled.

Measured directly against the real mixer at the FC's geometry and airmode:

| commanded collective | pitch demand | motors | mean duty |
|---|---|---|---|
| 0.25 | 0.0 | `[0.25 0.25 0.25 0.25]` | 0.250 |
| 0.25 | 0.4 | `[0.95 0.15 0.15 0.95]` | **0.550** |
| 0.25 | 0.8 | `[1.00 0.15 0.15 1.00]` | **0.575** |
| 0.25 | 1.6 | `[1.00 0.15 0.15 1.00]` | **0.575** |

The height loop asked for 0.25; the motors got a mean of 0.575. **The difference
is lift.** A saturated attitude loop generates climb as a side effect, and no
gain, clamp, or guard inside the height controller can counter it — attitude
outranks altitude in the allocator, which is the correct priority and precisely
why airmode RP was chosen over the uniform scaler.

## 3. Why it oscillates rather than settling

Once motors are pinned at 0.15 and 1.0 the loop is a **relay**, not a
proportional controller: output sits at one limit or the other regardless of
error magnitude. A relay plus phase lag self-oscillates at the frequency where
loop phase reaches −180° — the ~1.4 Hz pitch cascade already on record.

A relay limit cycle's amplitude is set by the **saturation limits and the
plant**, not by the loop gain. That is why detuning slightly never cured it, and
why the earlier finding recorded it as "NOT gain, NOT delay"
(`memory/indi-rate-loop.md`).

Pitch reaches the limits first because the airframe's own system-ID found a
high-K pitch plant: the same command produces more rate response there.

---

## 4. What the reference autopilots do

Sources read at `~/Documents/Drone/stack/resources/{ardupilot,PX4-Autopilot}`.
Three layers, all of which vayu currently lacks.

### 4.1 Feed mixer saturation back into the rate PID — **both do this**

**ArduPilot.** The mixer records what it could not deliver
(`AP_MotorsMatrix.cpp`: `limit.set_rpy(true)`, `limit.yaw`,
`limit.throttle_upper`), and the attitude controller passes the flag straight
into the PID (`AC_AttitudeControl_Multi.cpp:473-479`):

```cpp
_motors.set_pitch(get_rate_pitch_pid().update_all(ang_vel_body.y, gyro_rads.y, dt,
                                                  _motors.limit.pitch, ...));
```

`AC_PID::update_i` (`AC_PID.cpp:340-350`) then integrates only when not limited,
**or when the error opposes the integrator** — so the I-term can unwind during
saturation but never wind further in:

```cpp
if (!limit || ((is_positive(_integrator) && is_negative(_error)) ||
               (is_negative(_integrator) && is_positive(_error)))) {
    _integrator += ((float)_error * _ki) * i_scale * dt;
}
```

**PX4.** Same idea, per-axis and signed. The allocator publishes saturation;
`MulticopterRateControl.cpp:196-215` converts it to
`saturation_positive/negative` and calls `setSaturationStatus()`;
`RateControl::updateIntegral` (`rate_control.cpp:88-99`) clips the error itself
before integrating:

```cpp
if (_control_allocator_saturation_positive(i)) { rate_error(i) = math::min(rate_error(i), 0.f); }
if (_control_allocator_saturation_negative(i)) { rate_error(i) = math::max(rate_error(i), 0.f); }
```

**Vayu today:** `pid.c` freezes the integrator only when *its own* P+I hits the
PID's internal clamp. It never learns the mixer could not deliver the torque, so
during saturation the I-term keeps charging against thrust that never arrived —
the mechanism that sustains the limit cycle. This is the same problem the
optional INDI loop solves from the other direction ("stops INDI from integrating
against thrust it never got").

### 4.2 Bound the collective shift so desaturation cannot cause a climb — **ArduPilot**

`AP_MotorsMatrix.cpp:255-261`, verbatim:

> *"Situation #2 ensure we never increase the throttle above hover throttle
> unless the pilot has commanded this. Situation #2b allows us to raise the
> throttle above what the pilot commanded but not so far that it would actually
> cause the copter to rise."*

ArduPilot's desaturation may shift collective, but explicitly **not far enough to
make the aircraft climb**. It also scales roll/pitch down when the combined
demand exceeds the range (`rpy_scale = 1.0f / (rpy_high - rpy_low)`) and reserves
yaw headroom (`MOT_YAW_HEADROOM`, with `yaw_allowed` computed from the room
actually left after roll/pitch).

PX4's `ControlAllocationSequentialDesaturation::desaturateActuators` applies a
computed gain along the thrust direction with an `increase_only` guard and a
half-step second pass; `mixAirmodeRP()` mixes without yaw, desaturates along
thrust, then mixes yaw separately.

**Vayu today:** the collective shift is unbounded — the table in §2 is the
consequence.

### 4.3 Size authority against headroom, and back off when the loop rings — **ArduPilot**

- IMAX is documented as a function of available authority:
  `AP_MotorsMatrix.cpp:265-272` lists per-frame `MOT_YAW_HEADROOM` +
  `ATC_RAT_*_IMAX` combinations.
- `_slew_limiter` / `Dmod` (`AC_PID.cpp:273-279`) measures output slew rate and
  scales **P and D down when the loop starts ringing** — an automatic detune at
  oscillation onset.
- `_kpdmax` (`AC_PID.cpp:287-295`) caps |P + D| combined, separately from the
  output clamp.

**Vayu today:** `I_MAX 0.2` + `D_MAX 0.25` per axis, chosen arbitrarily rather
than derived from the ~0.35 of room.

---

## 5. Fix order

1. **Saturation feedback into the integrator.** Smallest change, biggest effect,
   and both references converge on it. `mixer_allocate` already computes
   everything needed — have it report which axes it could not satisfy (and in
   which direction), and gate integration in `v_pid_update` on that.
2. **Bound the airmode collective shift** so desaturation cannot produce a climb
   (ArduPilot's rule, §4.2). This is the specific fix for the flyaway. A clamp on
   the existing `kt` shift in `mixer.c`.
3. **PD sum cap** (`_kpdmax` equivalent), then the slew-limiter detune.
4. **Re-derive `I_MAX`/`D_MAX` from measured headroom** rather than leaving them
   at 0.2/0.25 against 0.35 of room.
5. **Then** revisit the optional INDI inner loop (`RATE_CTRL_ALGO_USED`,
   `memory/indi-rate-loop.md`) — it needs its `b` identified on the bench, and
   items 1–2 may make it unnecessary.

Note this supersedes an earlier instinct to simply cut `I_MAX`/`D_MAX` first:
that treats the symptom by weakening the controller everywhere, whereas 1 and 2
close the structural gaps and keep authority where it is usable.

---

## 6. Related finding — the board has no persisted tune

`pid.bin` pulled from the FC on 2026-09-04 decodes to **all zeros with
`valid = 0` on every axis** (magic `PID5`, 216 B). The FC is flying the
compiled-in defaults, not any tuned values from earlier sessions. Any gain work
under §5.4 has to start by getting gains to persist.

---

## 7. Verification

Bench, props off, current-limited:

- `MotorTelemetry.cmd` must stop pinning at `[1.0, 0.15]` under attitude
  disturbance. That is the primary pass/fail signal — **not** the reported
  roll/pitch, which read ~0.0° throughout the failure.
- Mean duty must track commanded collective (§2 table) rather than exceeding it.

SITL is **not** usable for this until the same behaviour is fixed there: the
stock GCS-configured vehicle reproduces the limit cycle on clean HEAD and flies
away at any throttle (`memory/sitl-pitch-limit-cycle-blocks-testing.md`).
