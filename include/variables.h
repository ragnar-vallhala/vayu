#ifndef VAYU_VARIABLES_H
#define VAYU_VARIABLES_H

// Include HAL Layer
#ifndef CORTEX_M4
#define CORTEX_M4
#endif // !CORTEX_M4
#include "navhal.h"
#include "sensor/bmx160.h"

// Physical Heartbeat LED
#define _HEARTBEAT_LED_PIN GPIO_PA05
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

// Sensor Fusion Parameters
#define SF_COMPLEMENTARY_ALPHA 0.98f
#define SF_MAHONY_KP 2.0f
#define SF_MAHONY_KI 0.005f
#define SF_FILTER_USED SF_MAHONY
#endif //! VAYU_VARIABLES_H