#include "comm/channel.h"
#include "memory.h"
#include "task.h"
#include "utils/test_file.h"
#include "vaios.h"

#include "vayu_tasks.h"

int main() {
  v_init();
  v_heap_memory_init();

  scheduler_init();
  // Task Create
  task_create(physical_heartbeat, NULL, 512, 0);
  task_create(comm_processor_task, NULL, 1024, 0);
  task_create(flush_task, NULL, 128, 0);
  task_create(test_task, NULL, 1024, 0);
  scheduler_start();
  while (1)
    ;
}
