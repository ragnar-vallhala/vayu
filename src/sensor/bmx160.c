#include "config.h"
#include "sensor/bmx160.h"
#include "navhal.h"
#include "utils.h"
#include "vaios.h"
#include "maths/sensor_fusion.h"
#include "variables.h"
#include <stdint.h>

uint8_t tx_buf[2];
uint8_t rx_buf[12];

// Static helper functions and variables
static uint8_t status = 0;
// Default BMX160 configuration
static bmx160_config_t bmx160_cfg = {
    // Accelerometer defaults
    .bmx160_acc_us = 0,    // undersampling disabled
    .bmx160_acc_bwp = 2,   // normal mode
    .bmx160_acc_odr = 8,   // 100 Hz
    .bmx160_acc_range = 5, // ±4g

    // Gyroscope defaults
    .bmx160_gyr_bwp = 2,   // normal mode
    .bmx160_gyr_odr = 9,   // 200 Hz
    .bmx160_gyr_range = 2, // ±500 dps

    // Magnetometer defaults
    .bmx160_mag_odr = 3 // 20 Hz
};

static void bmx160_set_mag_conf();
static bmx160_err_type bmx160_convert_raw_temp_to_celcius(int16_t raw_temp,
                                                          float *celcius);
hal_i2c_status_t bmx160_init(void)
{
    // I2C configuration
    hal_i2c_config_t i2c_config = {.clock_speed = STANDARD_MODE,
                                   .own_address = I2C_MASTER,
                                   .acknowledge = true};
    // Configure GPIO for I2C1 (PB8=SCL, PB9=SDA)
    hal_gpio_set_alternate_function(I2C_PIN_1, GPIO_FUNC_I2C);
    hal_gpio_set_alternate_function(I2C_PIN_2, GPIO_FUNC_I2C);
    hal_gpio_set_output_type(I2C_PIN_1, GPIO_OPEN_DRAIN);
    hal_gpio_set_output_type(I2C_PIN_2, GPIO_OPEN_DRAIN);
    hal_gpio_set_output_speed(I2C_PIN_1, GPIO_VERY_HIGH_SPEED);
    hal_gpio_set_output_speed(I2C_PIN_2, GPIO_VERY_HIGH_SPEED);
    hal_i2c_status_t ts = hal_i2c_init(I2C_BUS, &i2c_config);

    // BMX160 Configuration
    if ((ts == HAL_I2C_OK || ts == HAL_I2C_ERR_REINIT))
    {
        tx_buf[0] = BMX160_PWR_CONF_ADDR;
        tx_buf[1] = 0x00;
        hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);

        // Put accelerometer and gyroscope into normal mode
        tx_buf[0] = BMX160_CMD_ADDR;
        tx_buf[1] = BMX160_CMD_ACC_NORMAL;
        hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
        v_delay(10);

        tx_buf[0] = BMX160_CMD_ADDR; // register address (0x7E)
        tx_buf[1] = BMX160_CMD_GYR_NORMAL;
        hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
        v_delay(100);

        tx_buf[0] = BMX160_CMD_ADDR; // register address (0x7E)
        tx_buf[1] = BMX160_CMD_MAG_NORMAL;
        hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
        v_delay(100);
        bmx160_set_mag_conf();
    }

    // Config for components
    bmx160_read_config(&bmx160_cfg);
    return status;
}

static void bmx160_set_mag_conf()
{
    tx_buf[0] = BMX160_MAG_IF_0_ADDR;
    tx_buf[1] = 0x80;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
    v_delay(50);

    // sleep
    tx_buf[0] = BMX160_MAG_IF_3_ADDR;
    tx_buf[1] = 0x01;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);

    tx_buf[0] = BMX160_MAG_IF_2_ADDR;
    tx_buf[1] = 0x4B;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);

    tx_buf[0] = BMX160_MAG_IF_3_ADDR;
    tx_buf[1] = 0x04;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);

    tx_buf[0] = BMX160_MAG_IF_2_ADDR;
    tx_buf[1] = 0x51;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);

    tx_buf[0] = BMX160_MAG_IF_3_ADDR;
    tx_buf[1] = 0x0E;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);

    tx_buf[0] = BMX160_MAG_IF_2_ADDR;
    tx_buf[1] = 0x52;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);

    tx_buf[0] = BMX160_MAG_IF_3_ADDR;
    tx_buf[1] = 0x02;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);

    tx_buf[0] = BMX160_MAG_IF_2_ADDR;
    tx_buf[1] = 0x4C;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);

    tx_buf[0] = BMX160_MAG_IF_1_ADDR;
    tx_buf[1] = 0x42;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);

    tx_buf[0] = BMX160_MAG_CONF_ADDR;
    tx_buf[1] = 0x08;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);

    tx_buf[0] = BMX160_MAG_IF_0_ADDR;
    tx_buf[1] = 0x03;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
    v_delay(50);
}

uint16_t bmx160_get_chip_id(void)
{
    uint8_t reg = BMX160_CHIP_ID_ADDR;
    hal_i2c_status_t ret;

    ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 1);
    if (ret != HAL_I2C_OK)
    {
        return 0xFFFF;
    }
    int16_t chip_id = (int16_t)(rx_buf[0]);
    return (uint16_t)chip_id;
}

int16_t bmx160_read_temp_raw(void)
{
    if (status != 1)
    {
        // v_log(LOG_ERROR, "BMX160 not initialized but called
        // bmx160_read_temp_raw"); return 0xFFFF; // return invalid value
    }

    tx_buf[0] = BMX160_TEMPERATURE_1_ADDR;
    hal_i2c_status_t ret;

    // Read 2 bytes from temperature registers (0x20, 0x21)
    ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1);
    // Combine MSB and LSB (two's complement, little-endian)
    if (ret != HAL_I2C_OK)
    {
        return 0xFFFF;
    }
    int16_t temp_raw = (int16_t)(rx_buf[0] << 8);

    tx_buf[0] = BMX160_TEMPERATURE_1_ADDR;
    ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1);
    // Combine MSB and LSB (two's complement, little-endian)
    if (ret != HAL_I2C_OK)
    {
        return 0xFFFF;
    }
    // Combine MSB and LSB (two's complement, little-endian)
    temp_raw = (int16_t)(temp_raw | rx_buf[1]);

    return temp_raw; // keep raw for now
}

static bmx160_err_type bmx160_convert_raw_temp_to_celcius(int16_t raw_temp,
                                                          float *celcius)
{
    if ((uint16_t)raw_temp == 0x8000)
        return INVALID_DATA_READ;
    float raw = (float)raw_temp;
    raw /= 512.0f;
    raw += 23;
    *celcius = raw;
    return NO_ERR;
}

bmx160_err_type bmx160_read_temp_celcius(float *celcius)
{
    int16_t raw = bmx160_read_temp_raw();
    bmx160_err_type err = bmx160_convert_raw_temp_to_celcius(raw, celcius);
    if (err == NO_ERR)
    {
        v_log(LOG_INFO, "[BMX160] TEMP: %f", *celcius);
    }
    return ERR0;
}

bmx160_err_type bmx160_read_acc_raw(int16_t *raw)
{
    uint8_t reg = BMX160_ACCX_LOW_ADDR; // Start address of accel X LSB
    hal_i2c_status_t ret;

    // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 6);
    if (ret != HAL_I2C_OK)
    {
        return ERR0;
    }

    // Combine little-endian pairs into signed 16-bit values
    raw[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]); // X
    raw[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]); // Y
    raw[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]); // Z
    return NO_ERR;
}
bmx160_err_type bmx160_read_gyr_raw(int16_t *raw)
{
    uint8_t reg = BMX160_GYRX_LOW_ADDR; // Start address of accel X LSB
    hal_i2c_status_t ret;

    // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 6);
    if (ret != HAL_I2C_OK)
    {
        return ERR0;
    }

    // Combine little-endian pairs into signed 16-bit values
    raw[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]); // X
    raw[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]); // Y
    raw[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]); // Z

    return NO_ERR;
}
bmx160_err_type bmx160_read_mag_raw(int16_t *raw)
{
    uint8_t reg = BMX160_MAGX_LOW_ADDR; // Start address of accel X LSB
    hal_i2c_status_t ret;

    // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 6);
    if (ret != HAL_I2C_OK)
    {
        return ERR0;
    }

    // Combine little-endian pairs into signed 16-bit values
    raw[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]); // X
    raw[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]); // Y
    raw[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]); // Z
    return NO_ERR;
}

bmx160_err_type bmx160_read_all_raw(bmx160_all_reading_t *raw)
{
    int16_t data[3];
    bmx160_err_type status = bmx160_read_acc_raw(data);
    if (status != NO_ERR)
        return status;
    raw->raw.acc[0] = data[0];
    raw->raw.acc[1] = data[1];
    raw->raw.acc[2] = data[2];

    status = bmx160_read_gyr_raw(data);
    if (status != NO_ERR)
        return status;
    raw->raw.gyr[0] = data[0];
    raw->raw.gyr[1] = data[1];
    raw->raw.gyr[2] = data[2];

    status = bmx160_read_mag_raw(data);
    if (status != NO_ERR)
        return status;
    raw->raw.mag[0] = data[0];
    raw->raw.mag[1] = data[1];
    raw->raw.mag[2] = data[2];

    return NO_ERR;
}

bmx160_err_type bmx160_read_acc_mps2(float *data)
{
    int16_t raw[3];
    bmx160_err_type err = bmx160_read_acc_raw(raw); // <-- raw must be [3]
    if (err != NO_ERR)
        return err;

    data[0] = bmx160_raw_acc_to_mps2(raw[0]);
    data[1] = bmx160_raw_acc_to_mps2(raw[1]);
    data[2] = bmx160_raw_acc_to_mps2(raw[2]);

    return NO_ERR;
}

bmx160_err_type bmx160_read_gyr_dps(float *data)
{
    int16_t raw[3];
    bmx160_err_type err = bmx160_read_gyr_raw(raw);
    if (err != NO_ERR)
        return err;

    data[0] = bmx160_raw_gyr_to_dps(raw[0]);
    data[1] = bmx160_raw_gyr_to_dps(raw[1]);
    data[2] = bmx160_raw_gyr_to_dps(raw[2]);

    return NO_ERR;
}

bmx160_err_type bmx160_read_mag_uT(float *data)
{
    int16_t raw[3];
    bmx160_err_type err = bmx160_read_mag_raw(raw);
    if (err != NO_ERR)
        return err;

    data[0] = bmx160_raw_mag_to_uT(raw[0]);
    data[1] = bmx160_raw_mag_to_uT(raw[1]);
    data[2] = bmx160_raw_mag_to_uT(raw[2]);

    return NO_ERR;
}

bmx160_err_type bmx160_read_all_converted(bmx160_all_reading_t *data)
{
    float calculated[3];
    bmx160_err_type err = bmx160_read_acc_mps2(calculated);
    if (err != NO_ERR)
        return err;
    data->converted.acc[0] = calculated[0];
    data->converted.acc[1] = calculated[1];
    data->converted.acc[2] = calculated[2];
#if BMX160_ACC_LOGGING_UART
    v_log(LOG_INFO, "[BMX160] ACC: %f, %f, %f", data->converted.acc[0],
          data->converted.acc[1], data->converted.acc[2]);
#endif
    err = bmx160_read_gyr_dps(calculated);
    if (err != NO_ERR)
        return err;
    data->converted.gyr[0] = calculated[0];
    data->converted.gyr[1] = calculated[1];
    data->converted.gyr[2] = calculated[2];
#if BMX160_GYR_LOGGING_UART
    v_log(LOG_INFO, "[BMX160] GYR: %f, %f, %f", data->converted.gyr[0],
          data->converted.gyr[1], data->converted.gyr[2]);
#endif

    err = bmx160_read_mag_uT(calculated);
    if (err != NO_ERR)
        return err;
    data->converted.mag[0] = calculated[0];
    data->converted.mag[1] = calculated[1];
    data->converted.mag[2] = calculated[2];
#if BMX160_MAG_LOGGING_UART
    v_log(LOG_INFO, "[BMX160] MAG: %f, %f, %f", data->converted.mag[0],
          data->converted.mag[1], data->converted.mag[2]);
#endif
    return NO_ERR;
}

#define GET_ACC_US(x) ((x >> 7U) & 1U)     // bit <7>
#define GET_ACC_BWP(x) ((x >> 4U) & 7U)    // bits <6:4>
#define GET_ACC_ODR(x) ((x >> 0U) & 15U)   // bits <3:0>
#define GET_ACC_RANGE(x) ((x >> 0U) & 15U) // bits <3:0>

#define GET_GYR_ODR(x) ((x >> 0U) & 15U)  // bits <3:0>
#define GET_GYR_BWP(x) ((x >> 4U) & 3U)   // bits <5:4>
#define GET_GYR_RANGE(x) ((x >> 0U) & 7U) // bits <2:0>

#define GET_MAG_ODR(x) ((x >> 0U) & 15U) // bits <3:0>

bmx160_err_type bmx160_read_acc_config(bmx160_config_t *config)
{
    // Reading ACC conf
    tx_buf[0] = BMX160_ACC_CONF_ADDR;
    if (hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
        HAL_I2C_OK)
    {
        config->bmx160_acc_us = GET_ACC_US(rx_buf[0]);
        config->bmx160_acc_bwp = GET_ACC_BWP(rx_buf[0]);
        config->bmx160_acc_odr = GET_ACC_ODR(rx_buf[0]);
    }
    else
        return ERR0;

    tx_buf[0] = BMX160_ACC_RANGE_ADDR;
    if (hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
        HAL_I2C_OK)
    {
        config->bmx160_acc_range = GET_ACC_RANGE(rx_buf[0]);
    }
    else
        return ERR0;
    return NO_ERR;
}

bmx160_err_type bmx160_read_gyr_config(bmx160_config_t *config)
{
    // Reading GYR conf
    tx_buf[0] = BMX160_GYR_CONF_ADDR;
    if (hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
        HAL_I2C_OK)
    {
        config->bmx160_gyr_bwp = GET_GYR_BWP(rx_buf[0]);
        config->bmx160_gyr_odr = GET_GYR_ODR(rx_buf[0]);
    }
    else
        return ERR0;

    tx_buf[0] = BMX160_GYR_RANGE_ADDR;
    if (hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
        HAL_I2C_OK)
    {
        config->bmx160_gyr_range = GET_GYR_RANGE(rx_buf[0]);
    }
    else
        return ERR0;
    return NO_ERR;
}

bmx160_err_type bmx160_read_mag_config(bmx160_config_t *config)
{
    // Reading MAG conf
    tx_buf[0] = BMX160_MAG_CONF_ADDR;
    if (hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
        HAL_I2C_OK)
    {
        config->bmx160_mag_odr = GET_MAG_ODR(rx_buf[0]);
    }
    else
        return ERR0;
    return NO_ERR;
}

bmx160_err_type bmx160_read_config(bmx160_config_t *config)
{
    if (bmx160_read_acc_config(config) != NO_ERR)
        return ERR0;

    if (bmx160_read_gyr_config(config) != NO_ERR)
        return ERR0;

    if (bmx160_read_mag_config(config) != NO_ERR)
        return ERR0;
    bmx160_set_current_config(config);
    return NO_ERR;
}

bmx160_err_type bmx160_write_config(bmx160_config_t *config)
{
    if (bmx160_write_acc_config(config) != NO_ERR)
        return ERR0;
    if (bmx160_write_gyr_config(config) != NO_ERR)
        return ERR0;
    if (bmx160_write_mag_config(config) != NO_ERR)
        return ERR0;
    bmx160_set_current_config(config);
    return NO_ERR;
}

static uint8_t bmx160_get_acc_conf(bmx160_config_t *config)
{
    uint8_t val =
        (((config->bmx160_acc_us & 1U) << 7U) |
         ((config->bmx160_acc_bwp & 7U) << 3U) | (config->bmx160_acc_odr & 15U));
    return val;
}

static uint8_t bmx160_get_acc_range(bmx160_config_t *config)
{
    uint8_t val = (config->bmx160_acc_range & 15U);
    return val;
}

static uint8_t bmx160_get_gyr_conf(bmx160_config_t *config)
{
    uint8_t val =
        (((config->bmx160_gyr_bwp & 3U) << 3U) | (config->bmx160_gyr_odr & 15U));
    return val;
}

static uint8_t bmx160_get_gyr_range(bmx160_config_t *config)
{
    uint8_t val = (config->bmx160_gyr_range & 7U);
    return val;
}

static uint8_t bmx160_get_mag_conf(bmx160_config_t *config)
{
    uint8_t val = (config->bmx160_mag_odr & 15U);
    return val;
}

bmx160_err_type bmx160_write_acc_config(bmx160_config_t *config)
{
    uint8_t val;

    // --- ACC_CONF ---
    val = bmx160_get_acc_conf(config);
    tx_buf[0] = BMX160_ACC_CONF_ADDR;
    tx_buf[1] = val;
    if (hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
        return ERR0;

    // --- ACC_RANGE ---
    val = bmx160_get_acc_range(config);
    tx_buf[0] = BMX160_ACC_RANGE_ADDR;
    tx_buf[1] = val;
    if (hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
        return ERR0;

    return NO_ERR;
}

bmx160_err_type bmx160_write_gyr_config(bmx160_config_t *config)
{
    uint8_t val;

    // --- GYR_CONF ---
    val = bmx160_get_gyr_conf(config);
    tx_buf[0] = BMX160_GYR_CONF_ADDR;
    tx_buf[1] = val;
    if (hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
        return ERR0;

    // --- GYR_RANGE ---
    val = bmx160_get_gyr_range(config);
    tx_buf[0] = BMX160_GYR_RANGE_ADDR;
    tx_buf[1] = val;
    if (hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
        return ERR0;

    return NO_ERR;
}

bmx160_err_type bmx160_write_mag_config(bmx160_config_t *config)
{
    uint8_t val;

    // --- MAG_CONF ---
    val = bmx160_get_mag_conf(config);
    tx_buf[0] = BMX160_MAG_CONF_ADDR;
    tx_buf[1] = val;
    if (hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
        return ERR0;

    return NO_ERR;
}
static float bmx160_range_code_to_g(uint8_t range_code)
{
    switch (range_code)
    {
    case 3:
        return 2.0f; // ±2g
    case 5:
        return 4.0f; // ±4g
    case 8:
        return 8.0f; // ±8g
    case 12:
        return 16.0f; // ±16g
    default:
        return 2.0f; // fallback to ±2g
    }
}

float bmx160_raw_acc_to_mps2(int16_t raw)
{
    bmx160_config_t cfg = bmx160_get_current_config();
    float g_range = bmx160_range_code_to_g(cfg.bmx160_acc_range);

    // Convert raw → g → m/s²
    return ((float)raw / 32768.0f) * g_range * 9.80665f;
}
static float bmx160_range_code_to_dps(uint8_t range_code)
{
    switch (range_code)
    {
    case 0:
        return 2000.0f;
    case 1:
        return 1000.0f;
    case 2:
        return 500.0f;
    case 3:
        return 250.0f;
    case 4:
        return 125.0f;
    default:
        return 2000.0f; // fallback
    }
}

float bmx160_raw_gyr_to_dps(int16_t raw)
{
    bmx160_config_t cfg = bmx160_get_current_config();
    float dps_range = bmx160_range_code_to_dps(cfg.bmx160_gyr_range);

    // Convert raw → dps
    return ((float)raw / 32768.0f) * dps_range;
}

float bmx160_raw_mag_to_uT(int16_t raw)
{
    // Conversion factor from datasheet (LSB → microtesla)
    const float lsb_per_uT = 0.3f;

    return (float)raw * lsb_per_uT;
}

bmx160_config_t bmx160_get_current_config(void) { return bmx160_cfg; }
void bmx160_set_current_config(bmx160_config_t *cfg) { bmx160_cfg = *cfg; }

void run_bmx()
{
    hal_i2c_status_t status = bmx160_init();
    v_log(LOG_DEBUG, "BMX160 init status: %d", status);
    float temp = 0;
    attitude_t orientation;
    bmx160_config_t cfg;
    bmx160_all_reading_t data;
    // --- Accelerometer config ---
    cfg.bmx160_acc_us = 0;    // undersampling disabled
    cfg.bmx160_acc_bwp = 1;   // OSR2 (higher filter bandwidth)
    cfg.bmx160_acc_odr = 12;  // 400 Hz (see datasheet ODR table)
    cfg.bmx160_acc_range = 8; // ±8g

    // --- Gyroscope config ---
    cfg.bmx160_gyr_bwp = 0;   // OSR4 (narrow filter bandwidth, high precision)
    cfg.bmx160_gyr_odr = 13;  // 800 Hz
    cfg.bmx160_gyr_range = 1; // ±1000 dps

    // --- Magnetometer config ---
    cfg.bmx160_mag_odr = 6; // 50 Hz

    // --- Write configuration ---
    bmx160_write_config(&cfg);

    while (1)
    {
        bmx160_read_temp_celcius(&temp);

        bmx160_err_type err = bmx160_read_all_converted(&data);
        if (err != NO_ERR)
        {
            v_log(LOG_ERROR, "Error reading BMX160 data: %d", err);
            continue;
        }
        sf_acc_mag(data.converted.acc[0], data.converted.acc[1],
                   data.converted.acc[2], data.converted.mag[0],
                   data.converted.mag[1], data.converted.mag[2], &orientation);
        v_log(LOG_INFO, "[ATTITUDE] %f, %f, %f",
              orientation.roll,
              orientation.pitch, orientation.yaw);
        v_delay(10);
    }
}