#include "bmx160_test.h"
#include "bmx160/bmx160.h"
#include "utils.h"
#include "vaios.h"
#include <stdint.h>

void run_bmx() {
  hal_i2c_status_t status = bmx160_init();
  v_log(LOG_DEBUG, "BMX160 init status: %d", status);
  float temp = 0;
  int16_t arr[6];
  while (1) {
    bmx160_read_temp_celcius(&temp);
    bmx160_read_acc_raw(arr);
    bmx160_read_gyr_raw(arr);
    bmx160_read_mag_raw(arr);
    // uart2_write(">Temp:");
    // uart2_write(temp);
    // uart2_write("\n\r");
    // v_log(LOG_INFO, "BMX160 CHIP_ID: %x, >Temprature: %f",
    // bmx160_get_chip_id(),
    //       temp);
    v_delay(100);
  }
}
