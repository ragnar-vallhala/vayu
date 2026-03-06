#include "utils/test_file.h"
#include "comm/channel.h"
#include "comm/comm_types.h"
#include "comm/serializer.h"
#include "utils.h"
#include "vaios.h"
#include <stdint.h>

void test_task(void *args) {
  while (1)
    v_delay(1000);
}