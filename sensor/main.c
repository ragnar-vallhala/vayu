#include "bmx160_test.h"
#include "utils.h"
#include "vaios.h"
#include <stdint.h>

int main() {
  v_init();
    v_log(LOG_INFO, "Hello from Vayu!");
  while (1) {
    run_bmx();
  }
}
