# Vayu FC — Why Motors Can Run With No RC Link

**What this is:** a flight-safety trace of how the four motors can be commanded
(spin) while there is no live RC link, despite the arm/failsafe interlocks.
Generated 2026-06-23 from `src/` at HEAD. Sibling of
`resource-ownership-map.md` (the C1 VFS-contention finding is referenced below).
Target: STM32F401RE real firmware (not SITL).

## The gate is sound — so the question is "how do you stay ARMED with a dead link?"

`motor_task` zeroes all four outputs unless the state is `ARMED` or `IN_AIR`
(`src/actuator/motor.c:60-66`), and the state-transition table has **no
`FAILSAFE → ARMED` edge** (`src/sys/state.c:30-48`). So once the link-loss
watchdog trips FAILSAFE, re-arming is impossible. You also cannot *reach* ARMED
with zero RC frames on real hardware:

- Arming runs only inside `rc_apply_frame()`, which executes once per **complete
  parsed iBus frame** (`src/comm/rc_task.c:183-184`).
- The GCS software-arm command (`CMD_ARM`) only sets a latch
  (`g_sw_arm_request`, `src/comm/comm_processor.c:122`) that the *frame-driven*
  path evaluates — it never sets state directly
  (`src/comm/navlink_router.c:85`).

So the real exposure is **staying in (or holding) ARMED/IN_AIR while the link is
dead**. Four holes, one primary + three contributing.

---

## 1. Armed-then-dropout latency window *(primary)*

Once ARMED, RC loss is caught only by the **staleness watchdog**, which fires
after `RC_LOSS_TIMEOUT_MS` (1000 ms) plus up to one `watchdog_tick`
(`RC_LOSS_TIMEOUT_MS/4` = 250 ms) of latency (`rc_task.c:125,190`;
`include/comm/ibus.h:33`). Throughout that window `motor_task` keeps driving the
ESCs at the **last held command** (`prev_motor_outputs`, `motor.c:50-52`).

→ **Motors spin for up to ~1.25 s with a dead link** before FAILSAFE zeroes them.

## 2. FlySky throttle-failsafe blind spot widens the window

The fast failsafe (`rc_throttle_failsafe_step`, `rc_safety.c:85-117`) needs a
*detectable jump* to >1900 V held for `RC_FAILSAFE_HOLD_FRAMES`. Its own
docstring concedes: "if the pilot is already near the preset when the link drops,
the jump may be too small to see." Many FlySky receivers also **hold last value**
on loss (no jump at all). When no jump is seen, the only backstop is the 1 s
staleness watchdog from #1 — so the no-RC spin window is the **full ~1.25 s**,
not the fast ~100 ms (`RC_LOSS_DETECT_MS`).

## 3. `motor_task` holds last output if the control pipeline stalls

```c
if (!spsc_read(&motor_angle_rate2motor_queue, &motor_outputs, 1)) {
    motor_outputs = prev_motor_outputs;   // hold last
}
```
(`motor.c:50-52`). The state gate stays ARMED, so if the angle/rate controllers
stop *producing* — e.g. the IMU→estimator→controller chain stalls (the I2C
single-owner-bus stall risk C5 in the resource map, or any control-task
starvation) — the motors **latch at the last non-zero command indefinitely**,
with no fresh RC and no ramp-down. There is no "stale control output → zero"
timeout; only the state gate protects.

## 4. Boot-grace seed makes `rc_has_signal()` lie for 1 s

`rc_ibus_task` calls `rc_mark_frame_valid()` **unconditionally at startup**
(`rc_task.c:118`) so a cold vehicle gets the full timeout to acquire its first
frame. Side effect: for the first 1000 ms `rc_has_signal()` returns **true with
zero frames ever received**, so both `arm_preconditions_met()`
(`rc_safety.c:138-141`) and `rc_watchdog_step()` believe the link is healthy. The
interlock meant to keep a no-RC craft disarmed is effectively disabled during
that window. On real HW a frame is still required to arm, so this is a latent
contributor — but it is the same "safety check trusts an uninitialized
timestamp" class as the DWT-wrap and VFS findings.

## Related — sticky GCS latch (adjacent, not strictly "no RC")

`rc_arm_engaged()` is `ch5 > 1500 || g_sw_arm_request` (`rc_safety.c:155-157`).
If armed via the GCS latch, the physical RC kill-switch going **down does not
disarm** — the disarm branch requires `rc_arm_engaged() == false`
(`rc_task.c:73-78`). Only `CMD_DISARM` clears it. So "RC says disarm" can be
ignored while armed.

---

## Ruled out

The SITL synthetic-RC fallback is **not** an arm path: `host_rc_feeder`'s
`fill_hover` sets throttle = 1300, which fails `arm_preconditions_met`'s
`channels[2] < 1100` check → it routes to FAILSAFE, not ARMED.
(`sim/host/src/host_rc_feeder.c`, `rc_safety.c:136-141`.)

## Bottom line

With no RC you cannot *arm* on real hardware, but:
- (a) an armed craft keeps its motors live for up to **~1.25 s** after a dropout
  — extended to the full watchdog timeout whenever the receiver does hold-last
  instead of a throttle jump; and
- (b) if the control pipeline stalls while armed, `motor_task` latches the last
  command **indefinitely**.

### Suggested mitigations
1. **Control-output staleness timeout in `motor_task`:** zero (or ramp down) if
   no fresh command arrived in N ms, instead of holding `prev_motor_outputs`.
2. **Tighten the watchdog:** shorten `watchdog_tick` and/or wire the fast
   `RC_LOSS_DETECT_MS` (100 ms) path into the armed-motor cut, not just a
   degraded-link flag.
3. **Distinguish "never had a frame" from "had a frame, now stale":** so the
   boot-grace seed can't double as a healthy-link signal feeding
   `arm_preconditions_met`.
4. **Reconsider the sticky GCS latch** so a physical kill-switch always disarms.

## References
- `src/actuator/motor.c` (motor gate + hold-last)
- `src/comm/rc_task.c` (frame-driven arm/disarm, watchdog cadence, boot seed)
- `src/comm/rc_safety.c` (watchdog, throttle-failsafe, arm preconditions, latch)
- `src/comm/comm_processor.c`, `src/comm/navlink_router.c` (CMD_ARM latch path)
- `src/sys/state.c` (transition table — no FAILSAFE→ARMED)
- `include/comm/ibus.h` (RC_LOSS_* constants)
- Sibling: `firmware/docs/scratch/resource-ownership-map.md` (C5 bus stall, C1 VFS)
