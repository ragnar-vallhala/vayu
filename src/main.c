#include "utils.h"
#include "vaios.h"

int main() {
  v_init();
  v_log(LOG_INFO, "Hello from Vayu!");
  while (1)
    ;
}
