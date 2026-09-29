/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef VAYU_CONTROL_TUNING_H
#define VAYU_CONTROL_TUNING_H

/**
 * @file control/tuning.h
 * @brief How this airframe flies: gains, filters, limits and mode bindings.
 *
 * This was variables.h, which 31 translation units included for whatever they
 * happened to need -- control gains, loop rates, storage filenames, buffer
 * sizes and an I2C timeout in one header. Changing a PID default recompiled
 * the comm layer. Everything that was not a tune has gone to the thing that
 * owns it:
 *
 *   loop rates      -> control/loop_rates.h
 *   storage paths   -> storage/paths.h
 *   calibration     -> calib/calib_params.h
 *   link sizing     -> comm/comm_limits.h
 *   I2C, timer      -> the drivers that own those peripherals
 *   clock, heartbeat-> sys/clock.h, sys/heartbeat.h
 *
 * What is left is one subject: the numbers you change when the aircraft flies
 * badly. They are the FALLBACK -- a persisted tune (storage/paths.h,
 * PID_CONFIG_FILE_PATH) overrides any of them per slot, so the same pid.bin
 * yields identical behaviour in sim and on the board.
 */

// LPF Configurations
#define LPF_ACC_ALPHA 0.34f
#define LPF_GYR_ALPHA 0.51f
#define GYRO_BIAS_ALPHA 0.0034f

// Sensor Fusion Parameters
#define SF_COMPLEMENTARY_ALPHA 0.98f
// Yaw fuses the MAGNETOMETER (not gravity), so it needs a heavier correction
// than roll/pitch: at 0.98 the mag only nudges heading 2%/update, so gyro-bias
// drift wins and the heading wanders (and yaw-hold chases it). A lower alpha
// pulls harder toward the mag heading so it actually holds.
#define SF_YAW_COMPLEMENTARY_ALPHA 0.92f
#define SF_MAHONY_KP 3.0f
#define SF_MAHONY_KI 0.0025f
// Weight on the magnetometer error term in the Mahony update. The mag error
// feeds all three axes, so at full strength (Kp=3.0) mag noise/bias leaks into
// roll & pitch and can diverge the estimate during a maneuver. The mag's job is
// only the heading (yaw) reference, so fuse it gently — strong enough to hold
// heading, weak enough not to disturb the gravity-referenced tilt.
#define SF_MAHONY_MAG_WEIGHT 0.30f
/* Active attitude filter. One of: SF_COMPLEMENTARY, SF_MAHONY, SF_EKF
 * (6-state attitude + gyro bias), SF_EKF_ACCEL_BIAS (9-state, also accel
 * bias). EKF tunables live in include/est/ekf.h. */
#define SF_FILTER_USED SF_EKF

/* Active inner (rate) loop algorithm. One of: RATE_CTRL_PID (the shipped per-
 * axis PID, default) or RATE_CTRL_INDI (incremental dynamic inversion; see
 * include/control/rate_indi.h). Selected like SF_FILTER_USED — same compile-
 * time token-substitution. INDI tunables (b/k/lpf) live in rate_indi.h. */
#define RATE_CTRL_ALGO_USED RATE_CTRL_PID

// PID
#define NUM_AXES 3
// Gnereric Filtering and Deadbands
#define PID_GYRO_DEADBAND 0.1f // in deg/sec
#define PID_RC_DEADBAND 10     // in PWM
#define PID_RC2ANGLE_RATE_MODE NORMALIZED_RC2ANGLE_RATE_CUBIC
#define MIN_ARMED_THROTTLE 0.1f
/* Per-motor minimum-spin thrust held while ARMED (~15%, as a real ESC
 * does with MOTOR_STOP=false). The props keep turning so an ESC never
 * stalls/desyncs and the next command doesn't cold-start the motor, and
 * the low side keeps bidirectional authority instead of clipping against
 * zero. In SITL the non-zero value also distinguishes "armed and alive"
 * from the disarmed motors=0 condition the bridge gravity-cancellation
 * floor keys off (any value > 0 suffices). */
#define MOTOR_IDLE_FLOOR 0.15f
/* Below this throttle the rate-PID outputs are ramped from 0 (at
 * MIN_ARMED_THROTTLE) to full authority. The point is to keep the PID
 * silent while the drone is still ground-bound: an attitude correction
 * the airframe can't physically execute would otherwise just torque
 * the ground reaction, the mahony filter would track the resulting
 * wobble, and the loop diverges before the pilot ever lifts off. The
 * SITL X3 hovers around ~0.55 throttle, so the gate sits a bit below
 * that; on real vayu hardware TWR is high and hover is closer to 0.5,
 * but we also want the SAFE behavior of "PID quiet until you commit
 * to taking off".
 *
 * The hardware endpoint was 0.30, which on a high-TWR airframe is barely below
 * hover — so the whole liftoff transient ran on a fraction of the rate loop
 * (50% at throttle 0.2) and the drone left the ground before it could hold
 * attitude. 0.20 keeps the ramp's actual purpose (quiet while sitting on the
 * gear at idle, MOTOR_IDLE_FLOOR = 0.15) and reaches full authority well before
 * the airframe is light. The reference Carbon-Aeronautics controller has no
 * ramp at all: full PID above its cutoff, motors floored at 18%. */
#ifdef VAYU_SIM
#define PID_FULL_AUTHORITY_THROTTLE 0.45f
#else
#define PID_FULL_AUTHORITY_THROTTLE 0.20f
#endif
// PID gain defaults — UNIFIED across the SITL and hardware builds. The sim flies
// the REAL geometry pushed at runtime via VSIM_CTL_SET_GEOMETRY, so a single
// baseline keeps SITL and hardware on the same fallback (a build-divergent
// baseline would let any gain NOT in the persisted tune resolve far apart
// between builds). Roll/pitch are the on-hardware rig tune reconciled from
// firmware/docs/store/rig_tune.json (captured 2026-06-22): roll is sysid-tuned, pitch is
// seeded from roll (its own sysid still pending). Yaw stays the S500 autotune
// seed. They are only the FALLBACK: a persisted tune (0:pid.bin, loaded by
// pid_config_init() before the controllers init) overrides any of them per slot
// — so the SAME pid.bin yields identical behaviour in sim and on the board.
// Rate-loop seeds retuned 2026-07-14 against PX4's multicopter defaults (see
// firmware/docs/store/px4-gain-comparison.md). Converting vayu's deg/s gains to
// PX4's rad/s convention (x57.3) showed the old inner loop running 3-5x PX4 on P
// and 5-15x on D — which drove the ~8 Hz saturation/relay limit cycle seen in the
// 20260713 hand-test log (roll/yaw out slamming +-1). These land the rate P/D
// within ~1x of PX4; the outer (angle) loop is raised to compensate. I-gains left
// as-is (they don't drive the limit cycle) pending a flight-test trim.
#define DEAFULT_ROLL_ANGLE_RATE_KP 0.003f
#define DEAFULT_ROLL_ANGLE_RATE_KI 0.00811f
#define DEAFULT_ROLL_ANGLE_RATE_KD 0.00005f
#define DEAFULT_ROLL_ANGLE_RATE_KFF 0.0f
#define DEAFULT_ROLL_ANGLE_RATE_I_MAX 0.2f
#define DEAFULT_ROLL_ANGLE_RATE_D_MAX 0.25f
// D-term LPF time constant. An RC of 0.3 puts the cutoff at 1/(2*pi*RC) ~= 0.5 Hz,
// which filters the derivative path down to near-nothing — too aggressive once
// Kd is non-zero. 0.004 s ~= 40 Hz passes useful lead while still rejecting
// gyro noise.
#define DEAFULT_ROLL_ANGLE_RATE_D_LPF_RC 0.004f
#define DEAFULT_ROLL_ANGLE_RATE_OUT_MIN -1.0f
#define DEAFULT_ROLL_ANGLE_RATE_OUT_MAX 1.0f

/* Pitch rate: retuned toward PX4 (see the roll block). The previous set (Kp 0.008,
 * Kd 0.0008 == ~3x / ~15x PX4) did NOT damp the oscillation as its old comment
 * claimed — the 20260713 log shows it still limit-cycling once the D-LPF was opened
 * to 40 Hz (which un-inerted that oversized Kd). Kp/Kd now ~1x PX4. */
#define DEAFULT_PITCH_ANGLE_RATE_KP 0.0025f
#define DEAFULT_PITCH_ANGLE_RATE_KI 0.003f
#define DEAFULT_PITCH_ANGLE_RATE_KD 0.00005f
#define DEAFULT_PITCH_ANGLE_RATE_KFF 0.0f
#define DEAFULT_PITCH_ANGLE_RATE_I_MAX 0.2f
#define DEAFULT_PITCH_ANGLE_RATE_D_MAX 0.25f
#define DEAFULT_PITCH_ANGLE_RATE_D_LPF_RC                                      \
  0.004f // see roll: ~40 Hz so the rig-tune Kd isn't filtered to zero
#define DEAFULT_PITCH_ANGLE_RATE_OUT_MIN -1.0f
#define DEAFULT_PITCH_ANGLE_RATE_OUT_MAX 1.0f

// Yaw rate gains. Quad yaw authority comes from
// rotor reaction torque (k_moment << k_thrust) so it's weaker than roll/pitch;
// these mirror the roll/pitch seeds as a starting point — retune (e.g. via the
// SITL autotuner) for the actual airframe. Output limits MUST be non-zero or
// the PID clamps yaw to 0 regardless of gain.
#define DEAFULT_YAW_ANGLE_RATE_KP 0.004f
#define DEAFULT_YAW_ANGLE_RATE_KI 0.008f
#define DEAFULT_YAW_ANGLE_RATE_KD 0.0f
#define DEAFULT_YAW_ANGLE_RATE_KFF 0.0f
#define DEAFULT_YAW_ANGLE_RATE_I_MAX 0.2f
#define DEAFULT_YAW_ANGLE_RATE_D_MAX 0.25f
#define DEAFULT_YAW_ANGLE_RATE_D_LPF_RC 0.3f
#define DEAFULT_YAW_ANGLE_RATE_OUT_MIN -1.0f
#define DEAFULT_YAW_ANGLE_RATE_OUT_MAX 1.0f

// Angle (outer) controller, retuned 2026-07-14. Raised from 1.6898 toward PX4's
// MC_ROLL_P/MC_PITCH_P = 4.0 (both are rate-setpoint-per-angle-error, unit 1/s, so
// directly comparable). The old value was ~0.42x PX4 — sluggish leveling — and was
// only kept low to avoid a cascade against the OLD hot inner loop; now that the rate
// loop is calmed to ~PX4 levels, the outer loop can be assertive. 3.0 leaves margin
// under PX4's 4.0 for this rig.
#define DEAFULT_ROLL_ANGLE_KP 3.0f
#define DEAFULT_ROLL_ANGLE_TARGET_MAX 100.0f
#define DEAFULT_ROLL_ANGLE_OUT_MAX 100.0f

#define DEAFULT_PITCH_ANGLE_KP 3.0f
#define DEAFULT_PITCH_ANGLE_TARGET_MAX 100.0f
#define DEAFULT_PITCH_ANGLE_OUT_MAX 100.0f

// Yaw angle gain is unused in flight: yaw is RATE-controlled in both stabilise
// and acro (a centered stick holds the current heading; see angle_controller.c).
// Kept only so the angle PID slot is well-defined.
#define DEAFULT_YAW_ANGLE_KP 1.2f
#define DEAFULT_YAW_ANGLE_TARGET_MAX 100.0f
#define DEAFULT_YAW_ANGLE_OUT_MAX 100.0f

#define MAX_ANGLE_CUTOFF 70.0f
/* Bank-angle upset handling, angle mode only.
 *
 * This USED to call system_state_set(FAILSAFE), which motor.c turns into "all
 * four motors to zero" — and the state table has no path from FAILSAFE back to
 * IN_AIR, so it was a one-way trip. A single sample past 70 deg permanently
 * killed thrust in mid-air, turning any recoverable upset into a crash (it did
 * exactly that on 2026-09-04: pilot chopped throttle -> the rate loop's own
 * authority ramp faded the stabiliser out -> the airframe tipped -> failsafe ->
 * it fell inverted).
 *
 * In flight the FC now takes over and flies out of it: level attitude demand at
 * hover collective until the craft is back inside MAX_ANGLE_RECOVER. On the
 * ground (any state but IN_AIR) the old cut is kept — there, stopping the props
 * IS the right answer.
 *
 * RECOVERY_TIMEOUT_MS bounds the attempt. If the craft is still past the limit
 * after that, it is not coming back (inverted, broken prop, lost a motor) and
 * holding hover thrust would only drive it into the ground harder — so the cut
 * happens after all. */
#define MAX_ANGLE_RECOVER 45.0f /* hysteresis: exit recovery below this tilt */
#define RECOVERY_TIMEOUT_MS 2000u
/* Recovery holds the MEASURED hover (est/hover_estimate.h), not a constant.
 * There used to be a RECOVERY_THROTTLE 0.50f here, described as "roughly hover
 * for this airframe" — which it was for the 10in S500 and is not for the 5in
 * racer: (0.50/0.38)^2 = 1.73x hover thrust, about +7 m/s^2, held for up to
 * RECOVERY_TIMEOUT_MS. That is ~14 m of climb while the FC levels the craft,
 * i.e. the same arithmetic and the same mistake as the hover guess that flew it
 * into the ceiling. Anything that holds collective must key off the one live
 * hover number. */

/* Acro (rate) mode: a flight-mode toggle on RC channel ACRO_SWITCH_CH (0-based;
 * 5 == channel 6). When the channel reads above ACRO_SWITCH_US the attitude
 * loop is bypassed and the sticks command body rate directly (deg/s at full
 * stick) — no bank-angle limit, and the MAX_ANGLE_CUTOFF failsafe is suppressed
 * so the airframe can flip/roll continuously. */
/* DISABLED on this airframe: the transmitter only has 6 channels, 5 of which are
 * essential (roll/pitch/throttle/yaw/arm), so the one spare — channel 6 — is
 * given to the height mode below instead. Acro is still fully reachable from the
 * GCS via CMD_SET_FLIGHT_MODE, which outranks the RC source anyway
 * (control/flight_mode.h); only the physical switch binding is gone.
 *
 * Setting the channel out of range (>= IBUS_MAX_CHANNELS) is what disables it —
 * angle_controller.c already guards the read with that test. Restore this to 5
 * (or any free channel) on a transmitter with a spare, but NOT while the height
 * mode also lives on it: the acro threshold (>1500) and the height-mode UP
 * threshold (>1700) would both fire on the same stick position, and acro
 * force-disengages the height mode, so they must never share a channel. */
#define ACRO_SWITCH_CH 0xFF
#define ACRO_SWITCH_US 1500

/* Height mode on a 3-POSITION RC switch, ALT_MODE_CH (0-based; 5 == channel 6).
 * If your mode switch is on a different channel, this define is the only thing
 * to change.
 *
 * Channel 6 is this transmitter's only spare, so it is shared with nothing —
 * see the ACRO_SWITCH_CH note above for why the two cannot coexist here.
 * It MUST be a 3-position switch (on a FlySky i6, SwC): with a 2-position
 * switch there is no centre detent, so the mode could never be armed (the
 * interlock needs to see centre) and the low position would read as LAND.
 *
 *   centre  -> OFF   the sticks are entirely the pilot's, as they always were
 *   up      -> HOLD  lift off to HEIGHT_TARGET_M (1 m) and hold, or hold the
 *                    current height if already flying
 *   down    -> LAND  descend and settle, then idle the motors
 *
 * The collective stick is only taken while a mode is selected; roll/pitch/yaw
 * are never touched. See control/height_controller.h.
 *
 * A mode is armed only after the switch has been seen CENTRED since arming, so
 * arming with the switch already up cannot fly the craft off the ground on its
 * own — the pilot has to deliberately pass through centre. */
#define ALT_MODE_CH 5

/* THIS TRANSMITTER'S SwC IS INVERTED: switch UP reads ~1000 us and DOWN ~2000
 * (measured on the bench 2026-09-04 — flipping SwC to the top drove ch6 to
 * 1000, which the original "up = high us" mapping read as LAND; on the ground
 * that engaged, instantly latched touchdown, and idled the motors, so the
 * switch appeared to do nothing at all).
 *
 * The thresholds are therefore named for what the switch DOES, not for
 * microsecond direction — the mapping below is the single place that knows the
 * radio is reversed. If you ever reverse ch6 on the transmitter instead, swap
 * these two values back. */
#define ALT_MODE_HOLD_US 1300 /* BELOW this -> HOLD (SwC up on this radio)  */
#define ALT_MODE_LAND_US 1700 /* ABOVE this -> LAND (SwC down)             */
/* A channel reading below this is not a real RC value (no link, unmapped
 * channel, decode gap). It MUST NOT be read as the low position, because the
 * low position is now the one that commands a lift-off. */
#define ALT_MODE_VALID_MIN_US 900
#define DEAFULT_ROLL_ACRO_RATE_MAX 200.0f /* deg/s at full stick */
#define DEAFULT_PITCH_ACRO_RATE_MAX 200.0f
#define DEAFULT_YAW_ACRO_RATE_MAX 200.0f

#endif // VAYU_CONTROL_TUNING_H
