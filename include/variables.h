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
#define SF_MAHONY_KP 3.0f
#define SF_MAHONY_KI 0.0025f
#define SF_FILTER_USED SF_MAHONY
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
// Gains - VAYU_SIM overrides ship the SITL build with gentler
// gains because the X3 in Gazebo has lower inertia
// (Ixx=0.025, Iyy=0.009) than the real vayu drone and the controller
// otherwise applies too much corrective motor swing per degree of
// attitude error, slamming motors into saturation on every transient.
//
// Iterated tuning from the SITL logs:
//   1x firmware defaults -> drone flipped immediately on arm
//   4x reduction          -> drone armed briefly, hit FAILSAFE in ~2 s
//   16x reduction (here)  -> single mahony-attitude glitch can no
//                            longer drive any motor to saturation
//                            even on the X3's low-inertia airframe
#ifdef VAYU_SIM
#define DEAFULT_ROLL_ANGLE_RATE_KP 0.005f
#define DEAFULT_ROLL_ANGLE_RATE_KI 0.001f
#define DEAFULT_ROLL_ANGLE_RATE_KD 0.0005f
#define DEAFULT_ROLL_ANGLE_RATE_KFF 0.0f
#else
#define DEAFULT_ROLL_ANGLE_RATE_KP 0.08f
#define DEAFULT_ROLL_ANGLE_RATE_KI 0.04f
#define DEAFULT_ROLL_ANGLE_RATE_KD 0.01f
#define DEAFULT_ROLL_ANGLE_RATE_KFF 0.1f
#endif
#define DEAFULT_ROLL_ANGLE_RATE_I_MAX 0.2f
#define DEAFULT_ROLL_ANGLE_RATE_D_MAX 0.25f
#define DEAFULT_ROLL_ANGLE_RATE_D_LPF_RC 0.3f
#define DEAFULT_ROLL_ANGLE_RATE_OUT_MIN -1.0f
#define DEAFULT_ROLL_ANGLE_RATE_OUT_MAX 1.0f

#ifdef VAYU_SIM
#define DEAFULT_PITCH_ANGLE_RATE_KP 0.005f
#define DEAFULT_PITCH_ANGLE_RATE_KI 0.001f
#define DEAFULT_PITCH_ANGLE_RATE_KD 0.0005f
#define DEAFULT_PITCH_ANGLE_RATE_KFF 0.0f
#else
#define DEAFULT_PITCH_ANGLE_RATE_KP 0.08f
#define DEAFULT_PITCH_ANGLE_RATE_KI 0.04f
#define DEAFULT_PITCH_ANGLE_RATE_KD 0.01f
#define DEAFULT_PITCH_ANGLE_RATE_KFF 0.1f
#endif
#define DEAFULT_PITCH_ANGLE_RATE_I_MAX 0.2f
#define DEAFULT_PITCH_ANGLE_RATE_D_MAX 0.25f
#define DEAFULT_PITCH_ANGLE_RATE_D_LPF_RC 0.3f
#define DEAFULT_PITCH_ANGLE_RATE_OUT_MIN -1.0f
#define DEAFULT_PITCH_ANGLE_RATE_OUT_MAX 1.0f

#define DEAFULT_YAW_ANGLE_RATE_KP 0.0f
#define DEAFULT_YAW_ANGLE_RATE_KI 0.00f
#define DEAFULT_YAW_ANGLE_RATE_KD 0.00f
#define DEAFULT_YAW_ANGLE_RATE_KFF 0.0f
#define DEAFULT_YAW_ANGLE_RATE_I_MAX 0.0f
#define DEAFULT_YAW_ANGLE_RATE_D_MAX 0.0f
#define DEAFULT_YAW_ANGLE_RATE_D_LPF_RC 0.0f
#define DEAFULT_YAW_ANGLE_RATE_OUT_MIN 0.0f
#define DEAFULT_YAW_ANGLE_RATE_OUT_MAX 0.0f

// Angle controller
#ifdef VAYU_SIM
#define DEAFULT_ROLL_ANGLE_KP 0.25f
#else
#define DEAFULT_ROLL_ANGLE_KP 4.0f
#endif
#define DEAFULT_ROLL_ANGLE_TARGET_MAX 100.0f
#define DEAFULT_ROLL_ANGLE_OUT_MAX 100.0f

#ifdef VAYU_SIM
#define DEAFULT_PITCH_ANGLE_KP 0.25f
#else
#define DEAFULT_PITCH_ANGLE_KP 4.0f
#endif
#define DEAFULT_PITCH_ANGLE_TARGET_MAX 100.0f
#define DEAFULT_PITCH_ANGLE_OUT_MAX 100.0f

#define DEAFULT_YAW_ANGLE_KP 0.0f
#define DEAFULT_YAW_ANGLE_TARGET_MAX 100.0f
#define DEAFULT_YAW_ANGLE_OUT_MAX 100.0f

#define MAX_ANGLE_CUTOFF 70.0f

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
