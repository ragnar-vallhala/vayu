#ifndef VAYU_BMX160_H
#define VAYU_BMX160_H
#include "maths/sensor_fusion.h"
#include "navhal.h"
#include <stdint.h>

#define BMX160_CHIP_ID_ADDR 0x00
#define BMX160_CHIP_ID 0b11011000
#define BMX160_ERR_REG_ADDR 0x02
#define IS_DROPPED_TO_CMD(val) (val & (1 << 6))
#define IS_FATAL_ERR(val) (val & 1)

#define BMX160_PMU_STAT_ADDR 0x03
#define BMX160_PMU_STAT_ACC_NORMAL (0x02 << 4)
#define BMX160_PMU_STAT_GYR_NORMAL (0x02 << 2)
#define BMX160_PMU_STAT_MAG_NORMAL (0x02 << 0)
#define BMX160_PMU_STAT_ACC_MASK (0x03 << 4)
#define BMX160_PMU_STAT_GYR_MASK (0x03 << 2)
#define BMX160_PMU_STAT_MAG_MASK (0x03 << 0)

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

#define BMX160_ACC_CONF_ADDR 0x40
#define BMX160_ACC_RANGE_ADDR 0x41
#define BMX160_GYR_CONF_ADDR 0x42
#define BMX160_GYR_RANGE_ADDR 0x43
#define BMX160_MAG_CONF_ADDR 0x44

#define BMX160_PWR_CTRL_ADDR 0x7D

#define BMX160_MAG_IF_3_DATA_ADDR 0x4F
#define BMX160_MAG_IF_2_REG_ADDR 0x4E
#define BMX160_MAG_IF_1_READ_ADDR 0x4D
#define BMX160_MAG_IF_0_CONF_ADDR 0x4C

#define BMX160_IF_CONF_ADDR 0x6B
#define BMX160_PWR_CONF_ADDR 0x7C

#define BMM150_ADDR 0x10

#define BMX160_CMD_ADDR 0x7E

// BMX160 commands
#define BMX160_CMD_ACC_NORMAL 0x11
#define BMX160_CMD_GYR_NORMAL 0x15
#define BMX160_CMD_MAG_NORMAL 0x19
// Error options

typedef enum {
  NO_ERR = 0,
  ERR0 = 1,
  ERR1 = 2,
  LP_MODE_USE_PFD = 3,
  ODR_NO_MATCH_HEAD_LESS = 6,
  PFD_USE_IN_LP = 7,
  INVALID_DATA_READ = 8
} bmx160_err_type;
// Option enums
typedef enum {
  BMX160_ACC_2G = 2,
  BMX160_ACC_4G = 4,
  BMX160_ACC_8G = 8,
  BMX160_ACC_16G = 16,
} bmx160_acc_range_t;

typedef enum {
  BMX160_GYR_2000 = 2000,
  BMX160_GYR_1000 = 1000,
  BMX160_GYR_500 = 500,
  BMX160_GYR_250 = 250,
  BMX160_GYR_125 = 125,
} bmx160_gyr_range_t;

typedef enum {
  BMX160_ODR_0_78HZ = 0x01,
  BMX160_ODR_1_56HZ = 0x02,
  BMX160_ODR_3_125HZ = 0x03,
  BMX160_ODR_6_25HZ = 0x04,
  BMX160_ODR_12_5HZ = 0x05,
  BMX160_ODR_25HZ = 0x06,
  BMX160_ODR_50HZ = 0x07,
  BMX160_ODR_100HZ = 0x08,
  BMX160_ODR_200HZ = 0x09,
  BMX160_ODR_400HZ = 0x0A,
  BMX160_ODR_800HZ = 0x0B,
  BMX160_ODR_1600HZ = 0x0C,
  BMX160_ODR_3200HZ = 0x0D,
} bmx160_odr_t;

typedef enum {
  BMX_BWP_OSR4 = 0,
  BMX_BWP_OSR2 = 1,
  BMX_BWP_NORMAL = 2,
  BMX_BWP_OSR8 = 3,
  BMX_BWP_OSR16 = 4,
  BMX_BWP_OSR32 = 5,
} bmx_bandwidth_t;

// BMM150 trim data structure
typedef struct {
  int8_t dig_x1;
  int8_t dig_y1;
  int8_t dig_x2;
  int8_t dig_y2;
  uint16_t dig_z1;
  int16_t dig_z2;
  int16_t dig_z3;
  int16_t dig_z4;
  uint8_t dig_xy1;
  int8_t dig_xy2;
  uint16_t dig_xyz1;
} bmm150_trim_data_t;

// Config structures
typedef struct {
  // Accelerometer configuration
  bmx160_odr_t bmx160_acc_odr; // ODR selection
  uint8_t bmx160_acc_bwp;      // 3 bit defining the bandwidth pass
  uint8_t bmx160_acc_us;       // 1 bit defining under-sampling
  bmx160_acc_range_t bmx160_acc_range;
  // Gyro configuration
  bmx160_odr_t bmx160_gyr_odr; // ODR selection
  uint8_t bmx160_gyr_bwp;      // 2 bit defining the bandwidth pass
  bmx160_gyr_range_t bmx160_gyr_range;
  // Mag configuration
  bmx160_odr_t bmx160_mag_odr; // ODR selection
  uint8_t bmx160_mag_bwp;      // 3 bit defining the bandwidth pass
  uint8_t bmx160_mag_opmode;   // 2 bit defining the operation mode
} bmx160_config_t;

// Reading structures
typedef struct {
  int16_t acc[3];
  int16_t gyr[3];
  int16_t mag[3];
  uint16_t rhall;
  int16_t temp;
} bmx160_all_raw_reading_t;

typedef struct {
  float acc[3];
  float gyr[3];
  float mag[3];
  float temp;
} bmx160_all_converted_reading_t;

typedef union {
  bmx160_all_converted_reading_t converted;
  bmx160_all_raw_reading_t raw;
} bmx160_all_reading_t;

// Control APIs
hal_i2c_status_t bmx160_init(void);
uint16_t bmx160_get_chip_id(void);
// Sensor reading trigger and callback

void wake_imu_read_task();
void bmx160_initiate_read(void *args);
void bmx160_dma_callback(void);

// soft reset
bmx160_err_type bmx160_soft_reset(void); //[TODO]
// Power APIs
bmx160_err_type bmx160_sleep(void);  //[TODO]
bmx160_err_type bmx160_wakeup(void); //[TODO]

// Temp APIs
int16_t bmx160_read_temp_raw(void);
bmx160_err_type bmx160_read_temp_celcius(float *celcius);

// IMU raw APIs
bmx160_err_type bmx160_read_acc_raw(int16_t *raw);
bmx160_err_type bmx160_read_gyr_raw(int16_t *raw);
bmx160_err_type bmx160_read_mag_raw(int16_t *raw);
bmx160_err_type bmx160_read_all_raw(bmx160_all_reading_t *raw);

// IMU converted APIs
bmx160_err_type bmx160_read_acc_mps2(float *data);
bmx160_err_type bmx160_read_gyr_dps(float *data);
bmx160_err_type bmx160_read_mag_uT(float *data);
bmx160_err_type bmx160_read_all_converted(bmx160_all_reading_t *data);

// Config APIs
bmx160_err_type bmx160_read_config(bmx160_config_t *config);
bmx160_err_type bmx160_read_acc_config(bmx160_config_t *config);
bmx160_err_type bmx160_read_gyr_config(bmx160_config_t *config);
bmx160_err_type bmx160_read_mag_config(bmx160_config_t *config);

bmx160_err_type bmx160_write_config(bmx160_config_t *config);
bmx160_err_type bmx160_write_acc_config(bmx160_config_t *config);
bmx160_err_type bmx160_write_gyr_config(bmx160_config_t *config);
bmx160_err_type bmx160_write_mag_config(bmx160_config_t *config);

// Config helpers
float bmx160_raw_acc_to_mps2(int16_t raw);
float bmx160_raw_gyr_to_dps(int16_t raw);
float bmx160_raw_mag_to_uT(int16_t raw);

float bmx160_acc_odr_to_hz(uint8_t raw); //[TODO]
float bmx160_acc_bwp_to_hz(uint8_t raw); //[TODO]

float bmx160_gyr_odr_to_hz(uint8_t raw); //[TODO]
float bmx160_gyr_bwp_to_hz(uint8_t raw); //[TODO]

float bmx160_mag_odr_to_hz(uint8_t raw); //[TODO]

// Getters and Setter for static variables
bmx160_config_t bmx160_get_current_config(void);
void bmx160_set_current_config(bmx160_config_t *cfg);
void bmx160_get_attitude(attitude_t *att);

// Calibration definitions
#define IMU_CALIBRATION_SAMPLES 500

typedef struct {
  float imu_id;
  float type;
} calibration_args_t;

void calibration_task(void *args);

#endif // !VAYU_BMX160_H