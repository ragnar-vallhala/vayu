#ifndef VAYU_VARIABLES_H
#define VAYU_VARIABLES_H

// Include HAL Layer
#ifndef CORTEX_M4
#define CORTEX_M4
#endif // !CORTEX_M4
#include "navhal.h"
#include "sensor/bmx160.h"

// Clock Freq
#define SYS_CLOCK_FREQ 84000000 // 84MHz

/* Seconds between two DWT cycle-counter stamps (wrap-safe unsigned delta),
 * clamped to a sane range. IMU samples and attitude estimates carry their
 * acquisition cycle stamp (bmx160_all_converted_reading_t.timestamp,
 * attitude_t.timestamp); the control loops and the fusion step derive dt from
 * deltas of those instead of reading DWT at execution time, so dt is the true
 * inter-sample interval, immune to scheduler jitter. Floor prevents a div-by-0
 * in the PID derivative on a duplicate/first stamp; ceil bounds a wrap or stall.
 */
static inline float vayu_dt_from_cycles(uint32_t now_cyc, uint32_t prev_cyc) {
  uint32_t d = now_cyc - prev_cyc; /* wrap-safe */
  float dt = (float)d / (float)SYS_CLOCK_FREQ;
  if (dt < 1e-4f)
    dt = 1e-4f;
  if (dt > 0.1f)
    dt = 0.1f;
  return dt;
}

// Physical Heartbeat LED
#define _BLUE_LED_PIN GPIO_PB12
#define _GREEN_LED_PIN GPIO_PB13
#define _RED_LED_PIN GPIO_PB14
#define _BUZZER_PIN GPIO_PA05
#define _HEARTBEAT_DEFAULT_TIMEPERIOD 1000 // 1000ms

// I2C Control
#define MAX_I2C_DEVICES 10
#define I2C_MAX_TX_LEN 32
#define I2C_MAX_RX_LEN 64
#define I2C_BUS HAL_I2C_1
#define I2C_MODE HAL_I2C_SPEED_FAST
#define I2C_PIN_1 GPIO_PB08
#define I2C_PIN_2 GPIO_PB09
#define I2C_DR_REG_ADDR (uint32_t)(0x40005400 + 0x10)
#define I2C_MANAGER_SEMAPHORE_TIMEOUT 3 // ms
#define I2C_MANAGER_DMA_TIMEOUT 3       // ms
// IMU Sensor
#define BMX160_I2C_ADDR 0x68

// ODR Configurations (using bmx160_odr_t enums)
#define BMX_ACC_ODR BMX160_ODR_1600HZ
#define BMX_ACC_BWP BMX_BWP_OSR4
#define BMX_ACC_RANGE BMX160_ACC_8G

#define BMX_GYR_ODR BMX160_ODR_1600HZ
#define BMX_GYR_BWP BMX_BWP_OSR4
#define BMX_GYR_RANGE BMX160_GYR_1000

#define BMX_MAG_ODR BMX160_ODR_50HZ

// Comm settings
#define MAX_SERIAL_HANDLERS 3
#define INCOMING_PACKET_BUFFER 3

// Timer Callbacks
#define MAX_TIMER_CALLBACKS 4
#define HIGH_FREQ_TIMER_FREQ 10000 // 10kHz

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
#define RATE_CTRL_ALGO_USED RATE_CTRL_INDI

#define RADIO_AVOID_BAND 10

// PID
#define PID_FILE_PATH "0:pid.bin"
#define PID_FILE_SIZE 1024 // 1KB Preallocated
#define NUM_AXES 3
// Gnereric Filtering and Deadbands
#define PID_GYRO_DEADBAND 0.1f // in deg/sec
#define PID_RC_DEADBAND 10     // in PWM
#define PID_RC2ANGLE_RATE_MODE NORMALIZED_RC2ANGLE_RATE_CUBIC
#define MIN_ARMED_THROTTLE 0.1f
/* Per-motor "alive" thrust floor applied while ARMED. Below this
 * level the motors are held at the floor instead of going to zero.
 * Matches what a real ESC does when MOTOR_STOP=false (props keep
 * spinning slowly so the next throttle command doesn't have to
 * cold-start the motor), and -- in SITL -- gives the Gazebo bridge
 * a non-zero motor signal to detect "armed and alive" vs the
 * disarmed motors=0 condition (the bridge gravity-cancellation
 * floor keys off this). */
#define MOTOR_IDLE_FLOOR 0.005f
/* Below this throttle the rate-PID outputs are ramped from 0 (at
 * MIN_ARMED_THROTTLE) to full authority. The point is to keep the PID
 * silent while the drone is still ground-bound: an attitude correction
 * the airframe can't physically execute would otherwise just torque
 * the ground reaction, the mahony filter would track the resulting
 * wobble, and the loop diverges before the pilot ever lifts off. The
 * SITL X3 hovers around ~0.55 throttle, so the gate sits a bit below
 * that; on real vayu hardware TWR is high and hover is closer to 0.5,
 * but we also want the SAFE behavior of "PID quiet until you commit
 * to taking off". */
#ifdef VAYU_SIM
#define PID_FULL_AUTHORITY_THROTTLE 0.45f
#else
#define PID_FULL_AUTHORITY_THROTTLE 0.30f
#endif
// PID gain defaults — UNIFIED across the SITL and hardware builds. The sim flies
// the REAL geometry pushed at runtime via VSIM_CTL_SET_GEOMETRY, so a single
// baseline keeps SITL and hardware on the same fallback (a build-divergent
// baseline would let any gain NOT in the persisted tune resolve far apart
// between builds). Roll/pitch are the on-hardware rig tune reconciled from
// docs/store/rig_tune.json (captured 2026-06-22): roll is sysid-tuned, pitch is
// seeded from roll (its own sysid still pending). Yaw stays the S500 autotune
// seed. They are only the FALLBACK: a persisted tune (0:pid.bin, loaded by
// pid_config_init() before the controllers init) overrides any of them per slot
// — so the SAME pid.bin yields identical behaviour in sim and on the board.
#define DEAFULT_ROLL_ANGLE_RATE_KP 0.012f
#define DEAFULT_ROLL_ANGLE_RATE_KI 0.00811f
#define DEAFULT_ROLL_ANGLE_RATE_KD 0.00025f
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

#define DEAFULT_PITCH_ANGLE_RATE_KP 0.012f
#define DEAFULT_PITCH_ANGLE_RATE_KI 0.005f
#define DEAFULT_PITCH_ANGLE_RATE_KD 0.0001f
#define DEAFULT_PITCH_ANGLE_RATE_KFF 0.0f
#define DEAFULT_PITCH_ANGLE_RATE_I_MAX 0.2f
#define DEAFULT_PITCH_ANGLE_RATE_D_MAX 0.25f
#define DEAFULT_PITCH_ANGLE_RATE_D_LPF_RC 0.004f  // see roll: ~40 Hz so the rig-tune Kd isn't filtered to zero
#define DEAFULT_PITCH_ANGLE_RATE_OUT_MIN -1.0f
#define DEAFULT_PITCH_ANGLE_RATE_OUT_MAX 1.0f

// Yaw rate gains. Quad yaw authority comes from
// rotor reaction torque (k_moment << k_thrust) so it's weaker than roll/pitch;
// these mirror the roll/pitch seeds as a starting point — retune (e.g. via the
// SITL autotuner) for the actual airframe. Output limits MUST be non-zero or
// the PID clamps yaw to 0 regardless of gain.
#define DEAFULT_YAW_ANGLE_RATE_KP 0.018f
#define DEAFULT_YAW_ANGLE_RATE_KI 0.008f
#define DEAFULT_YAW_ANGLE_RATE_KD 0.0f
#define DEAFULT_YAW_ANGLE_RATE_KFF 0.0f
#define DEAFULT_YAW_ANGLE_RATE_I_MAX 0.2f
#define DEAFULT_YAW_ANGLE_RATE_D_MAX 0.25f
#define DEAFULT_YAW_ANGLE_RATE_D_LPF_RC 0.3f
#define DEAFULT_YAW_ANGLE_RATE_OUT_MIN -1.0f
#define DEAFULT_YAW_ANGLE_RATE_OUT_MAX 1.0f

// Angle controller. Roll/pitch reconciled from the on-hardware rig tune
// (docs/store/rig_tune.json, 2026-06-22): roll is sysid loop-shaped; pitch is
// kept low to avoid a cascade runaway against the soft inner loop (its own
// sysid still pending).
#define DEAFULT_ROLL_ANGLE_KP 1.6898f
#define DEAFULT_ROLL_ANGLE_TARGET_MAX 100.0f
#define DEAFULT_ROLL_ANGLE_OUT_MAX 100.0f

#define DEAFULT_PITCH_ANGLE_KP 1.6898f
#define DEAFULT_PITCH_ANGLE_TARGET_MAX 100.0f
#define DEAFULT_PITCH_ANGLE_OUT_MAX 100.0f

// Yaw angle gain is unused in flight: yaw is RATE-controlled in both stabilise
// and acro (a centered stick holds the current heading; see angle_controller.c).
// Kept only so the angle PID slot is well-defined.
#define DEAFULT_YAW_ANGLE_KP 1.2f
#define DEAFULT_YAW_ANGLE_TARGET_MAX 100.0f
#define DEAFULT_YAW_ANGLE_OUT_MAX 100.0f

#define MAX_ANGLE_CUTOFF 70.0f

/* Control-loop rates. The inner (rate) loop is hard-pinned to INNER_LOOP_FREQ_HZ
 * with a drift-free periodic wait (task_delay_until); the outer (angle) loop
 * runs at inner / OUTER_LOOP_DECIM. Each loop reads the freshest IMU sample /
 * attitude (the producers run at sensor rate and the OVERWRITE rings keep the
 * latest) and integrates with a CONSTANT dt = 1/freq, so the PID sees a fixed
 * timestep regardless of execution jitter or sensor-rate variation.
 *
 * NOTE: with the 1 ms SysTick the loop period is quantised to whole ms, so
 * INNER_LOOP_FREQ_HZ is effectively capped at 1000 and should divide 1000
 * (1000, 500, 250, ...). Override INNER_LOOP_FREQ_HZ before this header to
 * retune. */
#ifndef INNER_LOOP_FREQ_HZ
#define INNER_LOOP_FREQ_HZ 1000 /* rate (inner) loop, Hz */
#endif

/* IMU acquisition rate. The accel/gyro (FAST) reads are paced to this off the
 * HIGH_FREQ_TIMER (the 1 ms SysTick can't time sub-ms periods), decoupling the
 * sensor rate from the I2C free-run speed (~2.8 kHz). Default oversamples the
 * 1 kHz control loop 2x for gyro anti-aliasing / fusion fidelity while freeing
 * the CPU the extra ~0.8 kHz of per-sample work was burning. Must be <= the
 * I2C-bound ceiling (~2.8 kHz) and divide cleanly into 1e6 us. */
#ifndef IMU_SAMPLE_FREQ_HZ
#define IMU_SAMPLE_FREQ_HZ 2000
#endif
#define IMU_FAST_PERIOD_US (1000000u / IMU_SAMPLE_FREQ_HZ)
#define OUTER_LOOP_DECIM 4 /* outer = inner / OUTER_LOOP_DECIM */
#define OUTER_LOOP_FREQ_HZ (INNER_LOOP_FREQ_HZ / OUTER_LOOP_DECIM)

/* Periods in SysTick ticks (1 tick = SYSTICK_PERIOD us = 1 ms by default). */
#define INNER_LOOP_PERIOD_TICKS MS_TO_TICKS(1000 / INNER_LOOP_FREQ_HZ)
#define OUTER_LOOP_PERIOD_TICKS (INNER_LOOP_PERIOD_TICKS * OUTER_LOOP_DECIM)

/* Constant integration timesteps (seconds) derived from the pinned rates. */
#define INNER_LOOP_DT (1.0f / (float)INNER_LOOP_FREQ_HZ)
#define OUTER_LOOP_DT ((float)OUTER_LOOP_DECIM / (float)INNER_LOOP_FREQ_HZ)

/* Acro (rate) mode: a flight-mode toggle on RC channel ACRO_SWITCH_CH (0-based;
 * 5 == channel 6). When the channel reads above ACRO_SWITCH_US the attitude
 * loop is bypassed and the sticks command body rate directly (deg/s at full
 * stick) — no bank-angle limit, and the MAX_ANGLE_CUTOFF failsafe is suppressed
 * so the airframe can flip/roll continuously. */
#define ACRO_SWITCH_CH 5
#define ACRO_SWITCH_US 1500
#define DEAFULT_ROLL_ACRO_RATE_MAX  200.0f /* deg/s at full stick */
#define DEAFULT_PITCH_ACRO_RATE_MAX 200.0f
#define DEAFULT_YAW_ACRO_RATE_MAX   200.0f

typedef struct __attribute__((packed)) {
  float roll_angle_sp;
  float pitch_angle_sp;
  float yaw_angle_sp;
  float roll_angle_curr;
  float pitch_angle_curr;
  float yaw_angle_curr;
  float roll_rate_sp;
  float pitch_rate_sp;
  float yaw_rate_sp;
  float roll_rate_curr;
  float pitch_rate_curr;
  float yaw_rate_curr;
  float roll_out;
  float pitch_out;
  float yaw_out;
  float thro_out;
  float outer_dt;
  float inner_dt;
} control_telemetry_t;

// Calibration
#define CALIBRATION_FILE_PATH "0:cal.bin"
#define CALIBRATION_FILE_SIZE 1024 // 1KB Preallocated
#define CALIBRATION_WAIT_USER_TIME_PRE_CALIBRATION                             \
  2000 // 2 seconds, waits before recording once user has reached the direction
       // orientation
#define CALIBRATION_WAIT_USER_TIME_POST_CALIBRATION                            \
  1000 // 1 seconds, waits after recording before saving the calibration
#define CALIBRATION_SAMPLE_COUNT                                               \
  500 // Number of samples to take for calibration
#define MAG_FIT_MIN_SAMPLES                                                    \
  400 // minimum valid samples required to attempt the mag ellipsoid fit

/* Pose-tolerant full-3x3 accel calibration (calib engine, point-set fit). */
#define ACCEL_CAL_POSES                                                        \
  12 // 6 faces + 6 edges/corners — enough spread directions for a 9-DOF fit
#define ACCEL_CAL_MIN_POSES                                                    \
  9 // minimum captured poses to attempt the fit (9 DOF)
#define ACCEL_POSE_STILL_SAMPLES                                               \
  100 // contiguous static samples averaged per pose (~2 s at the 50 Hz cal poll)
#define ACCEL_CAL_GYRO_STILL_DPS                                               \
  3.0f // |gyro| below this (per axis sum-of-squares) counts the board as still

/* Stillness-gated gyro bias capture (calib engine, bias fit). */
#define GYRO_CAL_STILL_SAMPLES                                                 \
  300 // still samples to average for the bias (~6 s at the 50 Hz cal poll)
#define GYRO_CAL_MAX_TICKS                                                     \
  1500 // ~30 s budget; if the board never settles, fail (keep the old offset)
#define GYRO_CAL_VAR_MAX                                                       \
  1.0f // dps^2 per-axis variance ceiling on the accepted window

// Global telemetry channel and mutex
#include "comm/channel.h"
#include "ipc.h"
extern channel_t g_telemetry_channel;

// Telemetry
#define ENABLE_BINARY_NAVLINK_PKT 1
#define ENABLE_BINARY_NAVLINK_PKT_LOGGING 1
#define NAVLINK_LOGGING_FILENAME "0:v_nav.bin"
#define SYS_LOGGING_FILENAME "0:v_sys.bin"
#define GENERAL_LOGGING_FILENAME "0:v_gen.bin"
#ifdef VAYU_SIM
// SITL build: keep log files small so init does not stall on
// pre-allocation against a simulated SD backend.
#define NAVLINK_LOGGING_FILE_SIZE (64 * 1024)
#define SYS_LOGGING_FILE_SIZE     (64 * 1024)
#define GENERAL_LOGGING_FILE_SIZE (64 * 1024)
#else
#define NAVLINK_LOGGING_FILE_SIZE 1024 * 1024 * 10 // 10MB Preallocated
#define SYS_LOGGING_FILE_SIZE 1024 * 1024 * 10 // 10MB Preallocated
#define GENERAL_LOGGING_FILE_SIZE 1024 * 1024 * 10 // 10MB Preallocated
#endif
#define NAVLINK_HEADER_SIZE 8
#define NAVLINK_MAX_PAYLOAD_SIZE 256
#define NAVLINK_CRC_SIZE 4
#define NAVLINK_MAX_SIZE                                                       \
  (NAVLINK_HEADER_SIZE + NAVLINK_MAX_PAYLOAD_SIZE + NAVLINK_CRC_SIZE)
#endif //! VAYU_VARIABLES_H
