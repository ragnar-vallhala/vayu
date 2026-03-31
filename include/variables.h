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
#define I2C_BUS I2C1
#define I2C_MODE FAST_MODE
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
extern MutexHandle_t g_comm_mutex;

// Telemetry
#define ENABLE_BINARY_NAVLINK_PKT 1
#define ENABLE_BINARY_NAVLINK_PKT_LOGGING 1
#define NAVLINK_LOGGING_FILENAME "0:v_nav.bin"
#define NAVLINK_LOGGING_FILE_SIZE 1024 * 1024 * 10 // 10MB Preallocated
#define SYS_LOGGING_FILENAME "0:v_sys.bin"
#define SYS_LOGGING_FILE_SIZE 1024 * 1024 * 10 // 10MB Preallocated
#define GENERAL_LOGGING_FILENAME "0:v_gen.bin"
#define GENERAL_LOGGING_FILE_SIZE 1024 * 1024 * 10 // 10MB Preallocated
#define NAVLINK_HEADER_SIZE 8
#define NAVLINK_MAX_PAYLOAD_SIZE 256
#define NAVLINK_CRC_SIZE 4
#define NAVLINK_MAX_SIZE                                                       \
  (NAVLINK_HEADER_SIZE + NAVLINK_MAX_PAYLOAD_SIZE + NAVLINK_CRC_SIZE)
#endif //! VAYU_VARIABLES_H
