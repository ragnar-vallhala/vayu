#include "utils/test_file.h"
#include "vaios.h"
#include <stdint.h>


void test_task(void *args) {
  (void)args;


  while (1)
    v_delay(1000);
}