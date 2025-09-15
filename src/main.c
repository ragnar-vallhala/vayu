#include "utils.h"
#include "vaios.h"
#include "navhal.h"

int main() {
  v_init();
  v_log(LOG_INFO, "Hello from Vayu!");
  while (1)
    ;
}
