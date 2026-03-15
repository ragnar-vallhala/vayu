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
#define _BUZZER_PIN GPIO_PB15
#define _HEARTBEAT_DEFAULT_TIMEPERIOD 1000 // 1000ms

// I2C Control
#define I2C_BUS I2C1
#define I2C_MODE FAST_MODE
#define I2C_PIN_1 GPIO_PB08
#define I2C_PIN_2 GPIO_PB09
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
#define LPF_ACC_ALPHA 0.05f
#define LPF_GYR_ALPHA 0.01f
#define GYRO_BIAS_ALPHA 0.001f

// Sensor Fusion Parameters
#define SF_COMPLEMENTARY_ALPHA 0.98f
#define SF_MAHONY_KP 1.5f
#define SF_MAHONY_KI 0.005f
#define SF_FILTER_USED SF_MAHONY

// Global telemetry channel and mutex
#include "comm/channel.h"
#include "ipc.h"
extern channel_t g_telemetry_channel;
extern MutexHandle_t g_comm_mutex;

#endif //! VAYU_VARIABLES_H