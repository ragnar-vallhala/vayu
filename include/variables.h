#ifndef VAYU_VARIABLES_H
#define VAYU_VARIABLES_H

// Include HAL Layer
#ifndef CORTEX_M4
#define CORTEX_M4
#endif // !CORTEX_M4
#include "navhal.h"

// Physical Heartbeat LED
#define _HEARTBEAT_LED_PIN GPIO_PB10
#define _HEARTBEAT_DEFAULT_TIMEPERIOD 1000 // 1000ms

// I2C Control
#define I2C_BUS I2C1
#define I2C_MODE STANDARD_MODE
#define I2C_PIN_1 GPIO_PB08
#define I2C_PIN_2 GPIO_PB09
// IMU Sensor
#define BMX160_I2C_ADDR 0x68

// Control Declarations
#define BMX160_ACC_LOGGING_UART 1
#define BMX160_GYR_LOGGING_UART 1
#define BMX160_MAG_LOGGING_UART 1

// Comm settings
#define MAX_SERIAL_HANDLERS 3
#endif //! VAYU_VARIABLES_H