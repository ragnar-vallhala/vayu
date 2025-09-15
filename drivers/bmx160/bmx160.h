#ifndef VAYU_BMX160_H
#define VAYU_BMX160_H

#include "navhal.h"
#include <stdint.h>

#define BMX160_CHIP_ID_ADDR 0x00
#define BMX160_CHIP_ID 0b11011000
#define BMX160_ERR_REG_ADDR 0x02
#define IS_DROPPED_TO_CMD(val) (val & (1 << 6))
#define IS_FATAL_ERR(val) (val & 1)

typedef enum {
  NO_ERR = 0,
  ERR0 = 1,
  ERR1 = 2,
  LP_MODE_USE_PFD = 3,
  ODR_NO_MATCH_HEAD_LESS = 6,
  PFD_USE_IN_LP = 7,
  INVALID_DATA_READ = 8

} bmx160_err_type;

#define BMX160_PMU_STAT_ADDR 0x03

#define BMX160_MAGX_LOW_ADDR 0x04
#define BMX160_MAGX_HIGH_ADDR 0x05
#define BMX160_MAGY_LOW_ADDR 0x06
#define BMX160_MAGY_HIGH_ADDR 0x07
#define BMX160_MAGZ_LOW_ADDR 0x08
#define BMX160_MAGZ_HIGH_ADDR 0x09

#define BMX160_RHALL_LOW_ADDR 0x0A
#define BMX160_RHALL_HIGH_ADDR 0x0B

#define BMX160_GYRX_LOW_ADDR 0x0C
#define BMX160_GYRX_HIGH_ADDR 0x0D
#define BMX160_GYRY_LOW_ADDR 0x0E
#define BMX160_GYRY_HIGH_ADDR 0x0F
#define BMX160_GYRZ_LOW_ADDR 0x10
#define BMX160_GYRZ_HIGH_ADDR 0x11

#define BMX160_ACCX_LOW_ADDR 0x12
#define BMX160_ACCX_HIGH_ADDR 0x13
#define BMX160_ACCY_LOW_ADDR 0x14
#define BMX160_ACCY_HIGH_ADDR 0x15
#define BMX160_ACCZ_LOW_ADDR 0x16
#define BMX160_ACCZ_HIGH_ADDR 0x17

#define BMX160_SENSORTIME_0_ADDR 0x18
#define BMX160_SENSORTIME_1_ADDR 0x19
#define BMX160_SENSORTIME_2_ADDR 0x1A

#define BMX160_STATUS_ADDR 0x1B
#define BMX160_INT_STATUS_0_ADDR 0x1C
#define BMX160_INT_STATUS_1_ADDR 0x1B
#define BMX160_INT_STATUS_2_ADDR 0x1D
#define BMX160_INT_STATUS_3_ADDR 0x1F

#define BMX160_TEMPERATURE_0_ADDR 0x20
#define BMX160_TEMPERATURE_1_ADDR 0x21

#define BMX160_FIFO_LEN_0_ADDR 0x22
#define BMX160_FIFO_LEN_1_ADDR 0x23 // Only bottom 3 bits

#define BMX160_FIFO_DATA_ADDR 0x24

#define BMX160_CMD_REG 0x7E
#define BMX160_PWR_CONF_REG 0x6B
#define BMX160_PWR_CTRL_REG 0x6C

#define BMX160_ACC_CONF_REG 0x40
#define BMX160_ACC_RANGE_REG 0x41
#define BMX160_GYR_CONF_REG 0x42
#define BMX160_GYR_RANGE_REG 0x43

// BMX160 commands
#define BMX160_CMD_ACC_NORMAL 0x11
#define BMX160_CMD_GYR_NORMAL 0x15
#define BMX160_CMD_MAG_NORMAL 0x19

hal_i2c_status_t bmx160_init(void);
uint16_t bmx160_get_chip_id(void);
int16_t bmx160_read_temp_raw(void);
bmx160_err_type bmx160_convert_raw_temp_to_celcius(int16_t raw_temp,
                                                   float *celcius);
bmx160_err_type bmx160_read_temp_celcius(float *celcius);

bmx160_err_type bmx160_read_acc_raw(int16_t *raw);
bmx160_err_type bmx160_read_gyr_raw(int16_t *raw);
bmx160_err_type bmx160_read_mag_raw(int16_t *raw);

#endif // !VAYU_BMX160_H
